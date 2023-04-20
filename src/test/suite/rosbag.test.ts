import { Rosbag, Time } from '../../RosBag/rosbag';
import { describe, it, before, afterEach } from "mocha";
import { expect } from "chai";
import { open, TimeUtil } from 'rosbag';
import { createWriteStream } from 'fs';
import ROSLIB = require('roslib');

describe('Rosbag class', () => {
    let rosbag: Rosbag;

    //path of ros bag for testing 
    const bagPath = '../2023-03-17-20-22-17.bag';
  
    before(async () => {
      // instance of the Rosbag class with a sample bag path and a fake WebView object
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
      const originalBagPath = '../2023-03-17-20-22-17.bag';
      const newBagPath = '../2023-03-17-20-22-17-cloned.bag'; // Use a different path for the cloned bag
      const startTime: Time = { sec: 1, nsec: 0 };
      const endTime: Time = { sec: 2, nsec: 0 };

      const verbose = true;
      const topics = ['topic1', 'topic2'];
    
      // Clone the bag
      await clone(originalBagPath, newBagPath, startTime, endTime, verbose, topics);
    
      // Check if the cloned bag exists
      const clonedBag = new Rosbag(newBagPath);
      await clonedBag.openBag();
      expect(clonedBag).to.exist;

      // Check if the messages in the cloned bag are within the specified time range
      const messages = await rosbag.getMessages(startTime, endTime);
      const clonedMessages = await clonedBag.getMessages(startTime, endTime);
      expect(clonedMessages).to.deep.equal(messages);
});
    
    
    it('should get messages within the specified time range and topics', async () => {
      const startTime: Time = { sec: 1, nsec: 0 };
      const endTime: Time = { sec: 2, nsec: 0 };
      const verbose = true;
      const topics = ['topic1', 'topic2'];
    
      // Get messages from the original bag
      const messages = await rosbag.getMessages({ startTime, endTime, topics, verbose });

      // Check if the messages are within the specified time range and topics
      expect(messages.end).to.be.greaterThan(0);
      for (const message of messages.messages) {
        expect(message.topic).to.be.oneOf(topics);
        expect(message.timestamp.sec).to.be.at.least(startTime.sec);
        expect(message.timestamp.sec).to.be.at.most(endTime.sec);
  }
    
      // Get messages from the cloned bag
      const clonedBag = new Rosbag('../2023-03-17-20-22-17-cloned.bag');
      await clonedBag.openBag();
      const clonedMessages = await clonedBag.getMessages({ startTime, endTime, topics, verbose });
      expect(clonedMessages.end).to.be.greaterThan(0);
      for (const message of clonedMessages.messages) {
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
        await rosbag.getMessages({ startTime, endTime, verbose, topics, bagPath: nonExistentBag });
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
      expect(clonedBag.getTopics()).to.include.members(topics);
    });
    
    it('should return a list of topics in the ROS bag', () => {
      const topics = rosbag.getTopics();
      expect(topics).to.be.an('array').that.includes('topic1', 'topic2');
    });
    
    it('should return the current time of the ROS bag', () => {
      const currentTime = rosbag.getCurrentIndex();
      expect(currentTime).to.be.an('object').that.includes({ sec: 0, nsec: 0 });
    });
    
    it('should seek to the given time in the ROS bag', async () => {
      const time = { sec: 1, nsec: 500000000 };
      await rosbag.seekToTime(time);
      expect(rosbag.pointer).to.be.greaterThan(0);
      expect(rosbag.getCurrentTime()).to.deep.equal(time);
    });
    
    it('should return the index of the message at the given time', async () => {
      const time = { sec: 1, nsec: 500000000 };
      const index = await rosbag.getMessageIndexAtTime(time);
      expect(index).to.be.a('number').that.is.greaterThan(0);
    });
    
    it('should return the message at the given index', async () => {
      const index = 5;
      const message = await rosbag.getMessageAtIndex(index);
      expect(message).to.exist;
      expect(message.topic).to.equal('topic1');
    });
    
    it('should return the index of the message with the given sequence number', async () => {
      const sequenceNumber = 10;
      const index = await rosbag.getMessageIndexBySequenceNumber(sequenceNumber);
      expect(index).to.be.a('number').that.is.greaterThan(0);
    });
    
    it('should return the message with the given sequence number', async () => {
      const seqNum = 5;
      const message = await rosbag.getMessageByIndex(seqNum);
      expect(message).to.exist;
      expect(message!.topic).to.equal('/some_topic');
      expect(message!.message).to.deep.equal({ data: 'some_data' });
    });
    
    it('should return null if the given sequence number is out of range', async () => {
      const seqNum = 100;
      const message = await rosbag.getMessageByIndex(seqNum);
      expect(message).to.be.null;
    });
    
    it('should return the next message in the bag', async () => {
      const message = await rosbag.getNextMessage();
      expect(message).to.exist;
      expect(message!.topic).to.equal('/some_topic');
      expect(message!.message).to.deep.equal({ data: 'some_data' });
    });
    
    it('should return null if there are no more messages in the bag', async () => {
      // Set the pointer to the end of the bag
      rosbag.pointer = rosbag.buffer!.length;
      const message = await rosbag.getNextMessage();
      expect(message).to.be.null;
    });
    
    it('should return the previous message in the bag', async () => {
      // Set the pointer to the middle of the bag
      rosbag.pointer = Math.floor(rosbag.buffer!.length / 2);
      const message = await rosbag.getPreviousMessage();
      expect(message).to.exist;
      expect(message!.topic).to.equal('/some_topic');
      expect(message!.message).to.deep.equal({ data: 'some_data' });
    });
    
    it('should return null if there are no previous messages in the bag', async () => {
      // Set the pointer to the beginning of the bag
      rosbag.pointer = 0;
      const message = await rosbag.getPreviousMessage();
      expect(message).to.be.null;
    });
    
  });