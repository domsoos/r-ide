const ROSLIB = require('roslib');

class ROS {
    static rosAPI;
    static rosLib;

    constructor() {
        if (ROS.rosAPI && ROS.rosAPI.isConnected) {
            return Promise.resolve();
        } else {
            return new Promise((resolve, reject) => {
                ROS.rosAPI = new ROSLIB.Ros({
                    url: "ws://localhost:9090"
                });
                console.log('connected');
                ROS.rosAPI.on('connection', () => {
                    console.log('ROS connected');
                    ROS.rosLib = ROSLIB;
                    resolve();
                });
                ROS.rosAPI.on('error', (error) => {
                    console.log(error);
                    reject(error);
                });
            });
        }
    }

    static getROSApi() {
        return ROS.rosAPI;
    }

    static getROSLib() {
        return ROS.rosLib;
    }

    static reconnect() {
        ROS.rosAPI.close();
        ROS.rosAPI = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        });

        return new Promise((resolve, reject) => {
            ROS.rosAPI.on('connection', () => {
                console.log('ROS connected');
                ROS.rosLib = ROSLIB;
                resolve();
            });
            ROS.rosAPI.on('error', (error) => {
                console.log(error);
                reject(error);
            });
        });
    }
}

module.exports = ROS;