import * as vscode from 'vscode';
import Bag, { TimeUtil, open } from 'rosbag';
import * as ROSLIB from 'roslib';
import { exec } from 'child_process';

interface Time {
    sec: number,
    nsec: number
}

interface MessageBuffer {
    start?: Time,
    end?: Time,
    messages: any[]
}



export class Rosbag {

    bagPath: string;
    bag: Bag | undefined;
    // messages: any[];
    publishers: Map<string, ROSLIB.Topic>;
    pointer: number;

    isPaused: boolean;

    view: vscode.Webview;

    rosoutListener: ROSLIB.Topic | undefined;

    buffer: MessageBuffer[] | undefined;
    beginningOfBagCache: MessageBuffer | undefined;

    startTime: any;
    endTime: any;

    static rosapi: ROSLIB.Ros = new ROSLIB.Ros({
        url: "ws://localhost:9090",
    });

    private static unpublishNotify = new Set<string>();

    constructor(bagPath: string, view: vscode.Webview) {
        this.bagPath = bagPath;
        this.publishers = new Map();
        this.view = view;
        this.isPaused = true;
        this.pointer = 0;
        this.openBag(bagPath);
    }

    public async openBag(bagPath: string) {
        await Rosbag.connect();

        this.bag = await open(bagPath);
        this.startTime = this.bag.startTime;
        this.endTime = this.bag.endTime;

        this.checkPublishers(true);


        this.beginningOfBagCache = await this.getMessages(this.startTime);

        // Read messages
        this.buffer = [
            this.beginningOfBagCache,
            await this.getMessages(TimeUtil.add(this.startTime, bufferTime(1))),
        ];       

        this.view.postMessage({type: "createdMessages"});
    }

    public async getMessages(startTime: Time) {
        let buffer: MessageBuffer = {
            start: startTime,
            end: TimeUtil.add(startTime, bufferTime(1)),
            messages: []
        };

        await this.bag!.readMessages({
            topics: [...this.publishers.keys()],
            startTime: startTime,
            endTime: TimeUtil.add(startTime, bufferTime(1))
        }, (result: any) => {

            // Convert Images from UInt8Array into base64 string
            if ("data" in result.message && result.message.data instanceof Uint8Array) {
                try {
                    result.message.data = Buffer.from(result.message.data).toString('base64'); 
                } catch (error) {
                    console.log(result);
                }
            }
            buffer.messages.push(result);
        });

        return buffer;
    }

    private currentIndex: number = 0;
    private leadupRemaining: number = 0;
    
    public async playBag() {
        let leadup: number;
        this.isPaused = false;

        this.checkPublishers();
    
        // console.log(this.messages.length);
        // console.log(this.currentIndex);
        while (TimeUtil.isLessThan(this.buffer![0].start!, this.endTime) && !this.isPaused) {
            const {message, topic, timestamp} = this.buffer![1].messages[this.currentIndex];
            
            if (this.currentIndex === this.buffer![1].messages.length - 1) {
                console.log("buffer switch");
                this.buffer![0] = this.buffer![1];
                this.getMessages(TimeUtil.add(this.buffer![0].end!, bufferTime(1))).then(mb => {
                    console.log(mb.messages.length);
                    console.log("loaded");
                    this.buffer![1] = mb;
                });
                this.currentIndex = 0;
                const nextTimeStamp = this.buffer![1].messages[0].timestamp;
                leadup = ((nextTimeStamp.sec - timestamp.sec) * 1000) + ((nextTimeStamp.nsec >> 20) - (timestamp.nsec >> 20));
            } else if (this.leadupRemaining === 0) {
                const nextTimeStamp = this.buffer![1].messages[this.currentIndex + 1].timestamp;
                leadup = ((nextTimeStamp.sec - timestamp.sec) * 1000) + ((nextTimeStamp.nsec >> 20) - (timestamp.nsec >> 20));
            } else {
                leadup = this.leadupRemaining;
                this.leadupRemaining = 0;
            }
            
            await Rosbag.waitForLeadup(leadup);
    
            this.publishers.get(topic)?.publish(new ROSLIB.Message(message));
    
            this.currentIndex++;
    
        }
    
        if (this.isPaused) {
            this.view.postMessage({type: 'pausedPlaying'});
        } else {
            console.log('finished playing');
            this.view.postMessage({type: 'finishedPlaying'});
            this.currentIndex = 0;
            this.isPaused;
        }
    }

    public async getBagDuration(bagFilePath: string): Promise<number> {
        return new Promise((resolve, reject) => {
          exec(`rosbag info ${bagFilePath}`, (error, stdout, stderr) => {
            if (error) {
              reject(new Error(`Error while processing the ROS Bag: ${stderr.trim()}`));
            } else {
              const durationRegex = /duration:\s+(\d+\.\d+)/;
              const match = stdout.match(durationRegex);
              if (match && match[1]) {
                const duration = parseFloat(match[1]);
                resolve(duration);
              } else {
                reject(new Error("Failed to find duration in rosbag info output."));
              }
            }
          });
        });
    }
      

    public async clearBag() {
        // this.messages = [];
        for (let p of [...this.publishers.values()]) {
            p.unadvertise();
        }

        this.publishers.clear();

        this.buffer = undefined;
    }

    public checkPublishers(forceRepublish: boolean = false) {
        if (!Rosbag.rosapi.isConnected || forceRepublish) {
            if (!Rosbag.rosapi.isConnected){
                Rosbag.connect();
            }

            this.rosoutListener?.unsubscribe();
            
            this.rosoutListener = new ROSLIB.Topic({
                ros: Rosbag.rosapi,
                messageType: "rosgraph_msgs/Log",
                name: "rosout_agg"
            });

            this.rosoutListener.subscribe(message => {
                this.readRosout_agg(message);
            });

            for (let p of this.publishers.values()) {
                p.unadvertise();
            }

            this.publishers.clear();

            // Read all topics and create publishers
            let conn: any;
            for (conn in this.bag?.connections) {

                let newPublisher = new ROSLIB.Topic({
                    ros: Rosbag.rosapi,
                    messageType: this.bag?.connections[conn].type!,
                    name: this.bag?.connections[conn].topic!
                });

                this.publishers.set(this.bag?.connections[conn].topic!, newPublisher);

                newPublisher.advertise();
            }

            // console.log([...this.publishers.values()]);
            console.log('read all connections');
            this.view.postMessage({type: 'createdConnections', value: [...this.publishers.keys()]});

            setTimeout(() => {
                if (Rosbag.unpublishNotify.size > 0) {
                    vscode.window.showErrorMessage(`The selected rosbag contained topics that returned errors. R-IDE has unpublished these topics for now. Check the rosbridge terminal for more information.`);
                    Rosbag.unpublishNotify = new Set();
                }
            }, 1000);
        }
    }

    public pauseBag() {
        this.isPaused = true;
    }

    public stopBag(){
        this.currentIndex = 0;
        this.pauseBag();
    }

    public replayBag(){
        this.currentIndex = 0;
        this.buffer![0] = this.beginningOfBagCache!;
        this.getMessages(TimeUtil.add(this.startTime, bufferTime(1))).then(mb => {
            this.buffer![1] = mb;
        });
    }

    public clone(newBagPath: string, start: Time, end: Time, verbose: boolean, topics: string[]) {
        let cmd = `rosbag filter ${this.bagPath} ${newBagPath}`;
        let filter = ` "${start.sec} <= t.secs <= ${end.sec} and topic in ("${topics.join('", "')}")`;
        if (verbose) {
            cmd += " --print=\"'%s @ %d.%d: %s' % (topic, t.secs, t.nsecs, m.data)\"";
        }
        const terminal = vscode.window.createTerminal();
        terminal.show();
        terminal.sendText(cmd + filter + `"`);
    }

    private static async waitForLeadup (leadup: number) {
        // console.log(leadup);
        return new Promise((resolve) => {
            setTimeout(() => {resolve(true);}, leadup  );
        });
    };

    // eslint-disable-next-line @typescript-eslint/naming-convention
    private readRosout_agg(message: any) {
        const regex = /\[Client \d+\] \[id: (publish|advertise):(?<topic>.*?):\d+\] (?<error_code>.*)/;
        let match = message.msg.match(regex);
        if (match) {
            // console.log(message.msg);
            this.publishers.get(match.groups.topic)?.unadvertise();
            this.publishers.delete(match.groups.topic);
            Rosbag.unpublishNotify.add(message.msg);
        }
    }

    static async connect() {
        try {
            if (!Rosbag.rosapi.isConnected) {
                Rosbag.rosapi.close();
                Rosbag.rosapi = new ROSLIB.Ros({
                    url: "ws://localhost:9090"
                });
                
                Rosbag.rosapi.on('connection', () => {
                    console.log('connected');
                });

                Rosbag.rosapi.on('error', () => {
                    console.log('failed');
                });
            }
        } catch (err) {
            Rosbag.rosapi = new ROSLIB.Ros({
                url: "ws://localhost:9090"
            });
            
            Rosbag.rosapi.on('connection', () => {
                console.log('connected');
            });

            Rosbag.rosapi.on('error', () => {
                console.log('failed');
            });
        }
    }

    static isROSConnected(){
        return new Promise((resolve, reject) => {
            if (!Rosbag.rosapi.isConnected) {
              Rosbag.rosapi.close();
              Rosbag.rosapi = new ROSLIB.Ros({
                url: "ws://localhost:9090"
              });
        
              Rosbag.rosapi.on('connection', () => {
                resolve(true);
              });
        
              Rosbag.rosapi.on('error', () => {
                resolve(false);
              });
            } else {
              resolve(Rosbag.rosapi.isConnected);
            }
          });
    }
}

function bufferTime(n: number): Time {
    return {sec: 0 * n, nsec: 500_000_000 * n};
}