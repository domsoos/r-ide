import * as vscode from 'vscode';
import Bag, { open } from 'rosbag';
import * as ROSLIB from 'roslib';

export class Rosbag {

    bag: Bag | undefined;
    messages: any[];
    publishers: Map<string, ROSLIB.Topic>;
    pointer: number;

    isPaused: boolean;

    view: vscode.Webview;

    rosoutListener: ROSLIB.Topic | undefined;

    static rosapi: ROSLIB.Ros = new ROSLIB.Ros({
        url: "ws://localhost:9090",
    });

    private static unpublishNotify = new Set<string>();

    constructor(bagPath: string, view: vscode.Webview) {
        this.messages = [];
        this.publishers = new Map();
        this.view = view;
        this.isPaused = true;
        this.pointer = 0;
        this.openBag(bagPath);
    }

    public async openBag(bagPath: string) {
        await Rosbag.connect();

        this.bag = await open(bagPath);

        this.checkPublishers(true);

        // Read all messages
        this.bag.readMessages({topics: [...this.publishers.keys()]}, (result: any) => {

            // Convert Images from UInt8Array into base64 string
            if ("data" in result.message && result.message.data instanceof Uint8Array) {
                try {
                    result.message.data = Buffer.from(result.message.data).toString('base64'); 
                } catch (error) {
                    console.log(result);
                }
            }
            this.messages.push(result);

            if (this.messages.length % 5000 === 0) {
                console.log(`Loaded ${this.messages.length} messages`);
            }
        }).then(() => {
            console.log('read all messages');
            this.view.postMessage({type: 'createdMessages'});
        });
    }

    private currentIndex: number = 0;
    private leadupRemaining: number = 0;
    
    public async playBag() {
        let leadup: number;
        this.isPaused = false;

        this.checkPublishers();
    
        //console.log(this.messages.length);
        console.log(this.currentIndex);
        while (this.currentIndex < this.messages.length - 1 && !this.isPaused) {
            const {message, topic, timestamp} = this.messages[this.currentIndex];
            
            if (this.leadupRemaining === 0) {
                const nextTimeStamp = this.messages[this.currentIndex + 1].timestamp;
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
        }
    }
    

    public async clearBag() {
        this.messages = [];
        for (let p of [...this.publishers.values()]) {
            p.unadvertise();
        }

        this.publishers.clear();
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
        // Calculate remaining leadup time when paused
        if (this.currentIndex < this.messages.length - 1) {
            const currentTimestamp = this.messages[this.currentIndex].timestamp;
            const now = Date.now();
            const playedTime = now - currentTimestamp.sec * 1000 - currentTimestamp.nsec / 1e6;
            const nextTimestamp = this.messages[this.currentIndex + 1].timestamp;
            const totalLeadup = ((nextTimestamp.sec - currentTimestamp.sec) * 1000) + ((nextTimestamp.nsec >> 20) - (currentTimestamp.nsec >> 20));
            this.leadupRemaining = totalLeadup - playedTime;
        }
    }

    private static async waitForLeadup (leadup: number) {
        // console.log(leadup);
        return new Promise((resolve) => {
            setTimeout(() => {resolve(true);}, leadup);
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