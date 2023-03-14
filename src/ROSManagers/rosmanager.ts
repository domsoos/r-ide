import * as ROSLIB from "roslib";

export class ROS {
    static rosAPI: ROSLIB.Ros;

    constructor() {
        if (ROS.rosAPI && ROS.rosAPI.isConnected) {
            return Promise.resolve();
        } else {
            return new Promise(() => {
                ROS.rosAPI = new ROSLIB.Ros({
                    url: "ws://localhost:9090"
                });
                ROS.rosAPI.on('connection', () => {
                    console.log('ROS connected');
                });
                ROS.rosAPI.on('error', (error: string) => {
                    console.log(error);
                });
            });
        }
    }

    static getROSApi() {
        return ROS.rosAPI;
    }

    static reconnect() {
        ROS.rosAPI.close();
        ROS.rosAPI = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        });

        return new Promise((resolve, reject) => {
            ROS.rosAPI.on('connection', () => {
                console.log('ROS connected');
            });
            ROS.rosAPI.on('error', (error: string) => {
                console.log(error);
            });
        });
    }
}

module.exports = ROS;