import { Rosbag, Time } from '../../RosBag/rosbag';
import { describe, it, before, afterEach } from "mocha";
import { expect } from "chai";
import * as fs from 'fs';
import ROSLIB = require('roslib');
import Bag, { open } from 'rosbag';

import * as vscode from 'vscode';

describe('Rosbag class', () => {
    let rosbag: Rosbag;

    const fakeWebviewView: vscode.WebviewView = {
      webview: {
        asWebviewUri: (uri) => uri,
        postMessage: (message) => Promise.resolve(true),
        onDidReceiveMessage: new vscode.EventEmitter<any>().event,
        cspSource: '',
        html: '',
        options: { enableScripts: false },
      },
      viewType: '',
      title: '',
      description: '',
      visible: false,
      onDidChangeVisibility: new vscode.EventEmitter<void>().event,
      onDidDispose: new vscode.EventEmitter<void>().event,
      show: function (preserveFocus?: boolean | undefined): void {
        throw new Error('Function not implemented.');
      }
    };

    //path of ros bag for testing 
    const bagPath = '/mnt/c/Users/Josh/Desktop/CS-411/ridev3/r-ide/src/test/2023-03-17-20-22-17.bag';
  
    before(async () => {
      // instance of the Rosbag class with a sample bag path and a fake WebView object
      Rosbag.setView(fakeWebviewView.webview);
      rosbag = new Rosbag(bagPath);
      await rosbag.openBag();
    });
  
    afterEach(async () => {
      await rosbag.clearBag();
    });
  
    it('should initialize Rosbag with correct properties', () => {
      expect(rosbag).to.be.instanceOf(Rosbag);
      expect(rosbag.publishers).to.be.instanceOf(Map);
      expect(rosbag.isPaused).to.be.true;
      expect(rosbag.pointer).to.equal(0);
    });
  
    it('should open a ROS bag file and load messages', async () => {
      expect(rosbag.bag).to.exist;
      expect(rosbag.bag?.readMessages).to.exist;
      expect(rosbag.buffer).to.exist;
      expect(rosbag.buffer?.[0].messages.length).to.be.greaterThan(0);
    });
  
    it('should return the duration of the ROS bag file', () => {
      expect(rosbag.getBagDuration()).to.be.a('number');
    });
  
    it('should clear the ROS bag', async () => {
      await rosbag.clearBag();
      expect(rosbag.publishers.size).to.equal(0);
      expect(rosbag.buffer).to.not.exist;
    });
  
    it('should pause playing the ROS bag', async () => {
      rosbag.isPaused = false;
      rosbag.pauseBag();
      expect(rosbag.isPaused).to.be.true;
    });
  
    it('should stop playing the ROS bag', async () => {
      rosbag.setCurrentIndex(10);
      rosbag.stopBag();
      expect(rosbag.getCurrentIndex()).to.equal(0);
      expect(rosbag.isPaused).to.be.true;
    });
  
    it('should replay the ROS bag', async () => {
      rosbag.setCurrentIndex(10);
      await rosbag.replayBag();
      expect(rosbag.getCurrentIndex()).to.equal(0);
      expect(rosbag.buffer?.[0]).to.deep.equal(rosbag.beginningOfBagCache);
      expect(rosbag.buffer?.[1].start).to.deep.equal({ sec: 1, nsec: 0 });
      expect(rosbag.buffer?.[1].messages.length).to.be.greaterThan(0);
    });

    function isWithin(timestamp: Time, startTime: Time, endTime: Time): boolean {
      const startTimeValue = startTime.sec + startTime.nsec * 1e-9;
      const endTimeValue = endTime.sec + endTime.nsec * 1e-9;
      const timestampValue = timestamp.sec + timestamp.nsec * 1e-9;
      
      return startTimeValue <= timestampValue && timestampValue <= endTimeValue;
    }

    async function clone(originalBagPath: string, newBagPath: string, startTime: Time, endTime: Time, verbose: boolean, topics: string[]): Promise<void> {
      const originalBag = new Rosbag(originalBagPath);
      await originalBag.openBag();
  
      const newBag = new Rosbag(newBagPath);
      await newBag.openBag();
  
      await originalBag.bag!.readMessages({}, async (result) => {
          const { message, topic, timestamp } = result;
  
          if (isWithin(timestamp, startTime, endTime) && topics.includes(topic)) {
              if (verbose) {
                  console.log(`Message on ${topic} at ${JSON.stringify(timestamp)}: ${JSON.stringify(message)}`);
              }
              // This part is a workaround, as there is no direct writeMessage function available in the Rosbag class
              const publisher = newBag.publishers.get(topic);
              if (publisher) {
                  publisher.publish(new ROSLIB.Message(message));
              }
          }
      });
  
      await originalBag.clearBag();
      await newBag.clearBag();
  }
  
    
    it('should clone the ROS bag', async () => {
      const originalBagPath = '/mnt/c/Users/Josh/Desktop/CS-411/ridev3/r-ide/src/test/2023-03-17-20-22-17.bag';
      const newBagPath = '/mnt/c/Users/Josh/Desktop/CS-411/ridev3/r-ide/src/test/2023-03-17-20-23-17.bag'; // Use a different path for the cloned bag
      // maybe say cloned messages is a subset of messages
      const startTime: Time = { sec: 1, nsec: 0 };
      const endTime: Time = { sec: 2, nsec: 0 };

      const verbose = true;
      const topics = ['topic1', 'topic2'];
    
      // Clone the bag
      await clone(originalBagPath, newBagPath, startTime, endTime, verbose, topics);
    
      // Check if the cloned bag exists
      const clonedBag = await open(newBagPath);
      expect(clonedBag).to.exist;

      // Check if the messages in the cloned bag are within the specified time range
      const originalBag = await open(originalBagPath);
      const messages: any = [];
      await originalBag.readMessages({topics: topics, startTime: startTime, endTime: endTime}, (result: any) => {messages.push(result);});
      const clonedMessages: any = [];
      await clonedBag.readMessages({topics: topics, startTime: startTime, endTime: endTime}, (result: any) => {messages.push(result);});
      expect(clonedMessages).to.deep.equal(messages);


       // Check if the messages are within the specified time range and topics
       expect(messages.length).to.be.greaterThan(0);
       for (const message of messages) {
         expect(message.topic).to.be.oneOf(topics);
         expect(message.timestamp.sec).to.be.at.least(startTime.sec);
         expect(message.timestamp.sec).to.be.at.most(endTime.sec);
        }

});
    
    

    it('should throw an error when getting messages from a non-existent bag', async () => {
      const startTime: Time = { sec: 1, nsec: 0 };
      const endTime: Time = { sec: 2, nsec: 0 };
      const verbose = true;
      const topics = ['topic1', 'topic2'];

      // Attempt to get messages from a non-existent bag
      const nonExistentBag = 'non-existent-bag';
      let error: Error | undefined;
      try {
        const nonExistentRosbag = new Rosbag(nonExistentBag);
        await nonExistentRosbag.openBag(); // This should throw an error
        console.error("bag should not exist");
        // await nonExistentRosbag.getMessages({ startTime, endTime, topics }); // This line will not be executed
      } catch (e) {
        error = e as Error;
      }

      // Check if an error was thrown
      expect(error).to.exist;
      expect(error?.message).to.equal(`Bag ${nonExistentBag} does not exist`);
    });
    
    it('should throw an error when cloning a bag to an existing path', async () => {
      const newBagPath = 'test new bag path';
      const startTime = { sec: 1, nsec: 0 };
      const endTime = { sec: 2, nsec: 0 };
      const verbose = true;
      const topics = ['topic1', 'topic2'];
    
      // Clone the bag to a new path
      await expect(() => rosbag.clone(newBagPath, startTime, endTime, verbose, topics)).to.throw(Error, `Cannot clone bag to existing path: ${newBagPath}`);
    });
    
    it('should clone the ROS bag to a new path', async () => {
      const newBagPath = 'test new bag path';
      const startTime = { sec: 1, nsec: 0 };
      const endTime = { sec: 2, nsec: 0 };
      const verbose = true;
      const topics = ['topic1', 'topic2'];
    
      // Clone the bag to a new path
      await rosbag.clone(newBagPath, startTime, endTime, verbose, topics);
    
      // Check that the new bag file exists
      const newBagExists = await fs.promises.access(newBagPath)
      .then(() => true)
      .catch(() => false);
      expect(newBagExists).to.be.true;
    
      // Open the new bag and check its contents
      const clonedBag = new Rosbag(newBagPath);
      await clonedBag.openBag();
      expect(clonedBag.buffer?.[0].start).to.deep.equal(startTime);
      expect(clonedBag.buffer?.[1].end).to.deep.equal(endTime);
      expect(clonedBag.buffer?.[0].messages.length).to.be.greaterThan(0);
    });
    
    
    it('should return the current time of the ROS bag', () => {
      const currentTime = rosbag.getCurrentIndex();
      expect(currentTime).to.be.an('object').that.includes({ sec: 0, nsec: 0 });
    });
    
  });