import * as vscode from "vscode";
import Ros, { rosAPI } from '../ROSManagers/rosmanager';
import { Bag } from '@foxglove/rosbag';

class Player{
    resume = false;
    publishers = new Map();
    currentTime = null;
    startTime = null;
    endTime = null;
    bag = null;

    constructor(bagPath) {
        this.bag = new Bag(bagPath);
        console.log(bag.header);
    }

    static clearAll() {
        resume = false;
        this.clearPublishers();
        currentTime = null;
        startTime = null;
        endTime = null;
        bag = null;

    }

    static addPublisher(name, messageType) {
        const topic = new Ros.rosAPI.Topic({
            ros: rosAPI,
            name: name,
            messageType: messageType
        });

        this.publishers.push(topic);
        topic.advertise();
    }

    static removePublisher(name) {
        this.publishers.get(name).unadvertise();
        this.publishers.delete(name);
    }

    static clearPublishers() {
        for (let p of this.publishers.values()) {
            p.unadvertise();
        } 
        this.publishers.clear();
    }

    static publishMessage(publisher, message) {
        this.publishers.get(publisher, new Ros.rosAPI.Message(message));
    }

    static play() {
        this.resume = true;
    }

    static pause() {
        this.resume = false;
    }

    
}

module.exports = Player;