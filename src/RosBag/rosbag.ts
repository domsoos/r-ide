import * as vscode from 'vscode';
import Bag, { TimeUtil, open } from 'rosbag';
import * as ROSLIB from 'roslib';

interface Time {
    sec: number,
    nsec: number
}

interface MessageBuffer {
    start?: Time,
    end?: Time,
    messages: any[],
    toEnd: number
}



export class Rosbag {

    bagPath: string;
    bag: Bag | undefined;
    // messages: any[];
    publishers: Map<string, ROSLIB.Topic>;
    pointer: number;

    isPaused: boolean;

    static view: vscode.Webview;

    rosoutListener: ROSLIB.Topic | undefined;

    buffer: MessageBuffer[] | undefined;
    beginningOfBagCache: MessageBuffer | undefined;

    static rosapi: ROSLIB.Ros = new ROSLIB.Ros({
        url: "ws://localhost:9090",
    });

    private static unpublishNotify = new Set<string>();

    constructor(bagPath: string) {
        this.bagPath = bagPath;
        this.publishers = new Map();
        this.isPaused = true;
        this.pointer = 0;
    }

    public static setView(view: vscode.Webview) {
        Rosbag.view = view;
        
    }

    public async openBag() {
        await Rosbag.connect();

        this.bag = await open(this.bagPath);

        await this.checkPublishers(true);

        this.beginningOfBagCache = await this.getBufferedMessages(this.bag.startTime!);

        // Read messages
        this.buffer = [
            this.beginningOfBagCache,
            await this.getBufferedMessages(TimeUtil.add(this.bag.startTime!, bufferTime(1))),
        ];

        await Rosbag.view.postMessage({type: 'createdMessages'});
    }

    public async getBufferedMessages(startTime: Time) {
        let buffer: MessageBuffer = {
            start: startTime,
            end: TimeUtil.add(startTime, bufferTime(1)),
            messages: [],
            toEnd: 0
        };

        let prev = startTime;

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
                    
                }
            }
            result.leadup = (result.timestamp.sec - prev.sec) * 1e3 + ((result.timestamp.nsec - prev.nsec) >> 20);
            if (result.leadup.nsec < 0) {
                result.leadup.nsec += 1e9;
            }
            prev = result.timestamp;
            buffer.messages.push(result);
            buffer.toEnd = (buffer.end!.sec - prev.sec) * 1e3 + ((buffer.end!.nsec - prev.nsec) >> 20);
        });

        return buffer;
    }

    private currentIndex: number = 0;
    
    public async playBag() {
        this.isPaused = false;

        this.checkPublishers();
    
        while (TimeUtil.isLessThan(this.buffer![0].start!, this.bag!.endTime!) && !this.isPaused) {
            if (this.currentIndex === this.buffer![0].messages.length || this.buffer![0].messages[this.currentIndex] === undefined) {
                
                const wait = this.buffer![0].toEnd;
                this.buffer![0] = this.buffer![1];
                this.getBufferedMessages(TimeUtil.add(this.buffer![0].end!, bufferTime(1))).then(mb => {
                    
                    
                    this.buffer![1] = mb;
                });
                Rosbag.view.postMessage({type: "timeProgress", value: this.buffer![0].start!.sec - this.bag?.startTime!.sec!});
                this.currentIndex = 0;
                await Rosbag.waitForLeadup(wait);
                continue;
            }

            const {message, topic, leadup} = this.buffer![0].messages[this.currentIndex];
            await Rosbag.waitForLeadup(leadup);
    
            this.publishers.get(topic)?.publish(new ROSLIB.Message(message));
    
            this.currentIndex++;
    
        }
    
        if (this.isPaused) {
            Rosbag.view.postMessage({type: 'pausedPlaying'});
        } else {
            
            Rosbag.view.postMessage({type: 'finishedPlaying'});
            this.currentIndex = 0;
            this.isPaused;
        }
    }

    public getBagDuration(): number {
        let duration = {sec: this.bag!.endTime!.sec - this.bag!.startTime!.sec, nsec: this.bag!.endTime!.nsec - this.bag!.startTime!.nsec};

        if (duration.nsec < 0) {
            duration.sec -= 1;
            duration.nsec += 1e9;
        }

        return duration.sec + 1;
    }
      

    public async clearBag() {
        // this.messages = [];
        for (let p of [...this.publishers.values()]) {
            p.unadvertise();
        }

        this.publishers.clear();

        this.buffer = undefined;
    }

    public async checkPublishers(forceRepublish: boolean = false) {
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

            await Rosbag.view.postMessage({type: 'createdConnections', value: [...this.publishers.keys()]});

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
        this.getBufferedMessages(TimeUtil.add(this.bag!.startTime!, bufferTime(1))).then(mb => {
            this.buffer![1] = mb;
        });
    }

    public clone(newBagPath: string, start: Time, end: Time, verbose: boolean, topics: string[]) {
        let cmd = `rosbag filter ${this.bagPath} ${newBagPath}`;
        let filter = ` "${this.bag!.startTime!.sec + start.sec} <= t.secs <= ${this.bag!.startTime!.sec + end.sec} and topic in ('${topics.join("', '")}')"`;
        if (verbose) {
            cmd += " --print=\"'%s @ %d.%d: %s' % (topic, t.secs, t.nsecs, m)\"";
        }       

        return cmd + filter;
    }

    private static async waitForLeadup (leadup: number) {
        // 
        return new Promise((resolve) => {
            setTimeout(() => {resolve(true);}, leadup  );
        });
    };

    // eslint-disable-next-line @typescript-eslint/naming-convention
    private readRosout_agg(message: any) {
        const regex = /\[Client \d+\] \[id: (publish|advertise):(?<topic>.*?):\d+\] (?<error_code>.*)/;
        let match = message.msg.match(regex);
        if (match) {
            // 
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
                    
                });

                Rosbag.rosapi.on('error', () => {
                    
                });
            }
        } catch (err) {
            Rosbag.rosapi = new ROSLIB.Ros({
                url: "ws://localhost:9090"
            });
            
            Rosbag.rosapi.on('connection', () => {
                
            });

            Rosbag.rosapi.on('error', () => {
                
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

    static getPublishedTopics() {
        Rosbag.rosapi.getTopics((res) => {
            let {topics} = res;
            
            
            Rosbag.view.postMessage({type: "publishedTopics", value: topics});
            
        });
    }
}

function bufferTime(n: number): Time {
    return {sec: 0 * n, nsec: 5e8 * n};
}