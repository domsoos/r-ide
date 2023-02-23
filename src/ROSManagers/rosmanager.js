const ROSLIB = require('roslib');

class ROS {
    static rosAPI;
    static rosLib;

    constructor() {
        if (!ROS.rosAPI) {
            ROS.rosAPI = new ROSLIB.Ros({
                url: "ws://localhost:9090"
            });
            ROS.rosLib = ROSLIB;
        }
    }

    static getROSApi() {
        return ROS.rosAPI;
    }

    static getRosLib(){
        return ROS.rosLib;
    }

    static reconnect() {
        ROS.rosAPI.close();
        ROS.rosAPI = new ROSLIB.Ros({
            url: "ws://localhost:9090"
        });
    }
}

module.exports = ROS;