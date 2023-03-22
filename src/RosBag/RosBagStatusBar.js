import * as ROSLIB from 'roslib';
import * as vscode from "vscode";

export class RosBagStatusBar {
    // TODO: Add more buttons/features? Restart? Slider? Playback speed?
    static paused;
    static playPause;
    static step;
    static bag;
    static bagName;
    static messages;
    static publishers;
    static topics;

    constructor() {
        // Play/Pause
        RosBagStatusBar.playPause = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
        RosBagStatusBar.step = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100);
    }

    static async setBag(bagPath) {

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

    static async playback(start) {
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

module.exports = RosBagStatusBar;