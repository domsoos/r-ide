import * as vscode from 'vscode';
import Bag, { open } from 'rosbag';
import * as ROSLIB from 'roslib';

export class Rosbag {

    messages: any[];
    publishers: Map<string, ROSLIB.Topic>;
    pointer: number;

    isPaused: boolean;

    view: vscode.Webview;

    static rosapi: ROSLIB.Ros = new ROSLIB.Ros({
        url: "ws://localhost:9090"
    });

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

        let bag: Bag = await open(bagPath);

        // Read all messages
        bag.readMessages({}, (result: any) => {
            // Convert Images from UInt8Array into base64 string
            if ('data' in result.message) {
                result.message.data = Buffer.from(result.message.data).toString('base64');
            }
            this.messages.push(result);
        }).then(() => {
            console.log('read all messages');
            this.view.postMessage({type: 'createdMessages'});
        });

        // Read all topics and create publishers
        for (let conn in bag.connections) {

            let newPublisher = new ROSLIB.Topic({
                ros: Rosbag.rosapi,
                messageType: bag.connections[conn].type!,
                name: bag.connections[conn].topic
            });

            newPublisher.advertise();

            this.publishers.set(bag.connections[conn].topic, newPublisher);
        }

        // console.log([...this.publishers.values()]);
        console.log('read all connections');
        this.view.postMessage({type: 'createdConnections', value: [...this.publishers.keys()]});
    }

    private currentIndex: number = 0;
    private leadupRemaining: number = 0;
    
    public async playBag() {
        let leadup: number;
        this.isPaused = false;
    
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
        }
    }
    

    public async clearBag() {
        this.messages = [];
        for (let p of [...this.publishers.values()]) {
            p.unadvertise();
        }

        this.publishers.clear();
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
}