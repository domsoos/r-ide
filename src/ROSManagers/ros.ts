import { Ros, Topic } from "roslib";
import ROSLIB = require("roslib");

export class ROSManager {
    private static instance: ROSManager;
    private ros: Ros;
    private rosUrl = 'ws://localhost:9090';
    private reconnectInterval: any;

    private constructor() {
        this.ros = new Ros({
            url: this.rosUrl,
        });
    }

    public static getInstance(): ROSManager {
        if (!ROSManager.instance) {
            ROSManager.instance = new ROSManager();
        }
        return ROSManager.instance;
    }

    public getROSApi(): any{
        return {rosAPI: this.ros, rosLib: ROSLIB};
    }

    public isConnected(){
        if(this.ros.isConnected){
            return true;
        }else{
            this.reconnect();
            //this.startReconnectInterval();
            return false;
        }
            
    }

    private startReconnectInterval(){
        this.reconnectInterval = setInterval(() => {
            if(!this.ros.isConnected){
                this.reconnect();
            }
            else{
                clearInterval(this.reconnectInterval);
                this.reconnectInterval = null;
            }
        }, 1000);
    }

    public reconnect() {
        this.ros.close();
        this.ros = new Ros({
            url: this.rosUrl,
        });
    }
}


    /*
    public async getTopics(): Promise<any> {
        return new Promise((resolve, reject) => {
            this.ros.getTopics((res) => {
                if (res) {
                resolve(res);
                } else {
                reject(new Error("Failed to retrieve topics"));
                }
            }, (err)=>{
                reject(err);
            });
        });
    }
    */
