import Bag, { open } from "rosbag";
import ROSLIB = require("roslib");
import { ROS } from "../ROSManagers/rosmanager";
import * as vscode from "vscode";
import { TimeUtil } from "rosbag";

export class RosBagStatusBar {
    // TODO: Add more buttons/features? Restart? Slider? Playback speed?
    static paused: boolean;
    static playPause: vscode.StatusBarItem;
    static step: vscode.StatusBarItem;
    static bag: Bag | undefined;
    static bagName: string | undefined;
    static messages: any[];
    static publishers: Map<string, ROSLIB.Topic | undefined>;
    static topics: string[];

    constructor() {
        // Play/Pause
        RosBagStatusBar.playPause = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
        RosBagStatusBar.step = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
    }

    static async setBag(bagPath: vscode.Uri) {
        // Open the bag
        RosBagStatusBar.bag = await open(bagPath.fsPath);
        RosBagStatusBar.bagName = bagPath.fsPath.split('/').at(-1);

        // Load all the messages in timestamp order
        RosBagStatusBar.messages = [];
        RosBagStatusBar.publishers = new Map();
        RosBagStatusBar.topics = [];
        await RosBagStatusBar.bag.readMessages({}, result => {
            RosBagStatusBar.messages.push(result);
        });

        // Setup publishers
        for (let conn in RosBagStatusBar.bag.connections) {
            RosBagStatusBar.publishers.set(RosBagStatusBar.bag.connections[conn].topic, new ROSLIB.Topic({
                ros: ROS.getROSApi(),
                messageType: RosBagStatusBar.bag.connections[conn].type!,
                name: RosBagStatusBar.bag.connections[conn].topic
            }));
            RosBagStatusBar.topics.push(RosBagStatusBar.bag.connections[conn].topic);
        }

        // Set text and tooltip
        RosBagStatusBar.playPause.text = `${RosBagStatusBar.paused ? '$(debug-pause)' : '$(debug-play)'}${RosBagStatusBar.bagName}`;
        RosBagStatusBar.playPause.tooltip = `${RosBagStatusBar.paused ? 'Pause' : 'Play'} ${RosBagStatusBar.bagName}`;

        RosBagStatusBar.step.text = `$(debug-continue)`;
        RosBagStatusBar.step.tooltip = `Step ${RosBagStatusBar.bagName}`;

        RosBagStatusBar.playPause.show();
        RosBagStatusBar.step.show();

        console.log('show');

        RosBagStatusBar.paused = true;
    }

    static async clearBag() {
        RosBagStatusBar.bag = undefined;
        RosBagStatusBar.messages = [];
        RosBagStatusBar.bagName = '';

        // Clear the publishers
        for (let p of RosBagStatusBar.publishers.values()) {
            p?.unadvertise;
        }
        RosBagStatusBar.publishers.clear();

        RosBagStatusBar.playPause.hide();
        RosBagStatusBar.step.hide();
    }

    static async playback(start: number) {
        let i = start;
        while (!RosBagStatusBar.paused && i < RosBagStatusBar.messages.length - 1) {
            const {message, topic, timestamp} = RosBagStatusBar.messages[i];
            setInterval(() => {
                
                console.log(topic);
                console.log(message);
                RosBagStatusBar.publishers.get(topic)?.publish(
                    new ROSLIB.Message(message)
                );
            },
                // Sec and nano second into milliseconds
                (RosBagStatusBar.messages[i+1].timestamp.sec - timestamp.sec) * 1000 +
                (RosBagStatusBar.messages[i+1].timestamp.nsec - timestamp.nsec) / 1000000
            );

            i++;
        }
        
    }
}