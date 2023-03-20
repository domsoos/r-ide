import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import Bag, { open } from 'rosbag';
import * as ROSLIB from 'roslib';
import { Buffer } from "buffer";
import { encodeMono8 } from "./utils/encoder";

export class SidebarBagsProvider implements vscode.WebviewViewProvider {
  _view?: vscode.WebviewView;
  _doc?: vscode.TextDocument;

  messages: any[];
  publishers: Map<string, ROSLIB.Topic>;
  rosapi: ROSLIB.Ros;

  constructor(private readonly _extensionUri: vscode.Uri) {

    this.rosapi = new ROSLIB.Ros({
      url: "ws://localhost:9090"
    });

    this.messages = [];
    this.publishers = new Map();
  }

  private async openBag(bagPath: string) {
    this.clearBag();

    if (!this.rosapi.isConnected) {
      this.rosapi.close();
      this.rosapi = new ROSLIB.Ros({
        url: "ws://localhost:9090"
      });
    }
    
    let bag: Bag = await open(bagPath);

    // Read all messages
    bag.readMessages({}, result => {
      // Convert Images from UInt8Array into base64 string
      if ('data' in result.message) {
        result.message.data = Buffer.from(result.message.data).toString('base64');
      }
      this.messages.push(result);
    }).then(() => {
      console.log('read all messages');
      this._view?.webview.postMessage({type: 'createdMessages'});
    });

    // Read all topics and create publishers
    for (let conn in bag.connections) {
      // if(bag.connections[conn].type === "sensor_msgs/Image"){
        // console.log(bag.connections[conn]);
        // this.rosapi.getMessageDetails(bag.connections[conn].type!, (details) => {
        //   console.log(this.rosapi.decodeTypeDefs(details));
        // }, (error) => {
        //   console.log(error);
        // });
        let newPublisher = new ROSLIB.Topic({
          ros: this.rosapi,
          messageType: bag.connections[conn].type!,
          name: bag.connections[conn].topic
        });

        newPublisher.advertise();

        this.publishers.set(bag.connections[conn].topic, newPublisher);
      // }
    }

    // console.log([...this.publishers.values()]);

    this._view?.webview.postMessage({type: 'createdConnections'});
    console.log('read all connections');
  }

  private waitForLeadup(leadup: number) {
    // console.log(leadup);
    return new Promise((resolve) => {
        setTimeout(() => {resolve(true);}, leadup);
    });
  }

  private async playBag(startTime: number) {
    if (!this.rosapi.isConnected) {
      this.rosapi.close();
      this.rosapi = await new ROSLIB.Ros({
        url: "ws://localhost:9090"
      });
    }
    let i = 0, leadup = 0;

    console.log(this.messages.length);
    while (i < this.messages.length - 1) {
        const {message, topic, timestamp} = this.messages[i];
        await this.waitForLeadup(leadup);

        // if (topic === "/zed2/zed_node/obj_det/objects") {
        //   console.log(message);
        // } else 
        if (this.publishers.has(topic)) {
          // let output: Uint8Array = new Uint8Array(message.data.length);
          this.publishers.get(topic)?.publish(new ROSLIB.Message(message));
          // console.log(this.messages[i]);
        }

        
        // console.log(message);

        i++;

        // fast convert seconds and nanoseconds into milliseconds
        // console.log(messages[i]);
        const nextTimeStamp = this.messages[i].timestamp;
        leadup = ((nextTimeStamp.sec - timestamp.sec) * 1000) + ((nextTimeStamp.nsec >> 20) - (timestamp.nsec >> 20));
    }

    console.log('finished playing');
  }

  private clearBag() {
    this.messages = [];
    for (let p of [...this.publishers.values()]) {
      p.unadvertise();
    }
    this.publishers.clear();
  }

  public resolveWebviewView(webviewView: vscode.WebviewView) {
    this._view = webviewView;

    webviewView.webview.options = {
      // Allow scripts in the webview
      enableScripts: true,

      localResourceRoots: [this._extensionUri],
    };

    webviewView.webview.html = this._getHtmlForWebview(webviewView.webview);

    webviewView.webview.onDidReceiveMessage(async (data) => {
      console.log(data);
      switch (data.type) {
        case "onInfo": {
          if (!data.value) {
            return;
          }
          vscode.window.showInformationMessage(data.value);
          break;
        }
        case "onError": {
          if (!data.value) {
            return;
          }
          vscode.window.showErrorMessage(data.value);
          break;
        }
        case "getSelectedBag" :{
          await vscode.window.showOpenDialog({
            canSelectFiles: true, 
            canSelectFolders: false, 
            canSelectMany: false,
            // eslint-disable-next-line @typescript-eslint/naming-convention
            // filters: {'Bags': ['.bag']}
          }).then(async (result) =>{
            if(result && result[0].path){
              this.openBag(result[0].fsPath);

              webviewView.webview.postMessage({
                type: 'setSelectedBag',
                value: {
                  path: result[0].fsPath,
                },
              });
            }
          });
          
          break;
        }
        case "getCloneBagPath" :{
          vscode.window.showOpenDialog({
            canSelectFiles: false, 
            canSelectFolders: true, 
            canSelectMany: false, 
            defaultUri: vscode.Uri.file(data.value),
          }).then((result) =>{
            if(result && result[0].path){
              webviewView.webview.postMessage({
                type: 'setCloneBagPath',
                value: result[0].fsPath,
              });
            }
          });
          break;
        }
        case "r-ide.noConnection":{
          vscode.commands.executeCommand('r-ide.no-ros-connection');
          break;
        }
        case "playBag": {
          this.playBag(0);
        }
      }
    });
  }

  public revive(panel: vscode.WebviewView) {
    this._view = panel;
  }

  private _getHtmlForWebview(webview: vscode.Webview) {
    const styleResetUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "reset.css")
    );
    const styleVSCodeUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "styles", "vscode.css")
    );

    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "out", "compiled/sidebarbags.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "sidebarbags.css")
    );
    


    // Use a nonce to only allow a specific script to be run.
    const nonce = getNonce();

    return `<!DOCTYPE html>
			<html lang="en">
			<head>
				<meta charset="UTF-8">
          <!--
					Use a content security policy to only allow loading images from https or from our extension directory,
					and only allow scripts that have a specific nonce.
          -->
        <meta http-equiv="Content-Security-Policy" content="img-src https: data:; style-src 'unsafe-inline' ${
      webview.cspSource
    }; script-src 'nonce-${nonce}';">
				<meta name="viewport" content="width=device-width, initial-scale=1.0">
				<link href="${styleResetUri}" rel="stylesheet">
				<link href="${styleVSCodeUri}" rel="stylesheet">
                <link href="${styleMainUri}" rel="stylesheet">  
		    </head>
        <body>
            <script nonce="${nonce}" src="${scriptUri}"></script>
        </body>
        </html>`;
  }
}