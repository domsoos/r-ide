/*
import * as xmlrpc from "xmlrpc";
import * as extension from "../../extension";
import * as ROSLIB from 'roslib';


export async function roslibTest() {
    
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
      });

      ros.on('connection', () => {
        
      });

      ros.on('error', (error: any) => {
        console.error('Error connecting to ROS server:', error);
      });

      ros.on('close', () => {
        
      });

      ros.getTopics((topics: any) => {
        
      });
}


export async function rosTest() {
    const masterApi = new XmlRpcApi("http://localhost:11311");
    masterApi.check().then((status: boolean) => {
        if (status) {
            //let getParameters = masterApi.getParam("/");
            let getSystemState = masterApi.getSystemState();
            masterApi.getTopics();
            Promise.all([getSystemState]).then(([ systemState]) => {
                //let parametersJSON = JSON.stringify(parameters);
                let systemStateJSON = JSON.stringify(systemState);
            });
        }
    });
}

interface ISystemState {
    publishers: { [topic: string]: string[] };
    subscribers: { [topic: string]: string[] };
    services: { [service: string]: string[] };
}

const CALLER_ID = "vscode-ros";

//Exposes the ROS master XML-RPC api.

export class XmlRpcApi {
    private client: xmlrpc.Client;

    public constructor(uri: string) {
        this.client = xmlrpc.createClient(uri);
    }

    
    // Returns true if a master process is running.
    public check(): Promise<boolean> {
        return this.getPid().then(() => true, () => false);
    }

    public getPid(): Promise<number> {
        return this.methodCall("getPid");
    }

    public getTopics(): any{
        return this.methodCall("getPublishedTopics", "").then((res) =>{
            
        });
    }

    public getSystemState(): Promise<ISystemState> {
        const responseReducer = (acc: any, cur: any[]) => {
            const k: string = cur[0] as string;
            const v: string[] = cur[1] as string[];
            acc[k] = v;
            return acc;
        };
        return this.methodCall("getSystemState").then((res) => {
            const systemState: ISystemState = {
                publishers: res[0].reduce(responseReducer, {}),
                services: res[2].reduce(responseReducer, {}),
                subscribers: res[1].reduce(responseReducer, {}),
            };
            return systemState;
        });
    }

    public getParamNames(): Promise<string[]> {
        return this.methodCall("getParamNames");
    }

    public getParam(name: string): Promise<any> {
        return this.methodCall("getParam", name);
    }

    private methodCall(method: string, ...args: any[]): Promise<any> {
        return new Promise((resolve, reject) => {
            this.client.methodCall(method, [CALLER_ID, ...args], (err: any, val: any[]) => {
                if (err) {
                    reject(err);
                } else if (val[0] !== 1) {
                    reject(val);
                } else {
                    resolve(val[2]);
                }
            });
        });
    }
}
*/    

