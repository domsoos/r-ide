import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import * as cp from "child_process";

interface ParsedObject {
  [key: string]: any;
}

export class TopicMonitorProvider {
  /**
   * Track the currently panel. Only allow a single panel to exist at a time.
   */
  public static currentPanel: TopicMonitorProvider | undefined;

  public static readonly viewType = "topic-monitor";

  private readonly _panel: vscode.WebviewPanel;
  private readonly _extensionUri: vscode.Uri;
  private _disposables: vscode.Disposable[] = [];
  

  public static createOrShow(extensionUri: vscode.Uri) {
    const column = vscode.window.activeTextEditor
      ? vscode.window.activeTextEditor.viewColumn
      : undefined;

    // If we already have a panel, show it.
    if (TopicMonitorProvider.currentPanel) {
        TopicMonitorProvider.currentPanel._panel.reveal(column);
        TopicMonitorProvider.currentPanel._update();
      return;
    }

    // Otherwise, create a new panel.
    const panel = vscode.window.createWebviewPanel(
        TopicMonitorProvider.viewType,
      "ROS Topic Monitor",
      column || vscode.ViewColumn.One,
      {
        // Enable javascript in the webview
        enableScripts: true,

        // And restrict the webview to only loading content from our extension's `media` directory.
        localResourceRoots: [
          extensionUri
        ],
      }
    );

    TopicMonitorProvider.currentPanel = new TopicMonitorProvider(panel, extensionUri);
  }

  public static kill() {
    TopicMonitorProvider.currentPanel?.dispose();
    TopicMonitorProvider.currentPanel = undefined;
  }

  public static revive(panel: vscode.WebviewPanel, extensionUri: vscode.Uri) {
    TopicMonitorProvider.currentPanel = new TopicMonitorProvider(panel, extensionUri);
  }

  private constructor(panel: vscode.WebviewPanel, extensionUri: vscode.Uri) {
    this._panel = panel;
    this._extensionUri = extensionUri;

    // Set the webview's initial html content
    this._update();

    // Listen for when the panel is disposed
    // This happens when the user closes the panel or when the panel is closed programatically
    this._panel.onDidDispose(() => this.dispose(), null, this._disposables);

    // // Handle messages from the webview
    // this._panel.webview.onDidReceiveMessage(
    //   (message) => {
    //     switch (message.command) {
    //       case "alert":
    //         vscode.window.showErrorMessage(message.text);
    //         return;
    //     }
    //   },
    //   null,
    //   this._disposables
    // );
  }

  public dispose() {
    TopicMonitorProvider.currentPanel = undefined;

    // Clean up our resources
    this._panel.dispose();

    while (this._disposables.length) {
      const x = this._disposables.pop();
      if (x) {
        x.dispose();
      }
    }
  }

  private async _update() {
    const webview = this._panel.webview;

    this._panel.webview.html = this._getHtmlForWebview(webview);
    webview.onDidReceiveMessage(async (data) => {
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
        case "r-ide.noConnection":{
          vscode.commands.executeCommand('r-ide.no-ros-connection');
          break;
        }
        /*
        case "getMessageTypes":{
          //this.generateMessageFormat();
          let messageField = await this.getMessageField('actionlib_msgs/GoalID');
          break;
        }
        */
        case "getMessageTypeFormat": {
          if(data?.value?.type && data?.value?.fulltopic){
            let messageField = await this.getMessageField(data.value.type);
            
            webview.postMessage({
              type: 'setPublishMessageFormat',
              data: messageField 
            });
          }
          break;
        }
      }
    });
  }

  private async generateMessageFormat(){
    try {
      const messageTypes: any = await this.getMessageTypes();
      const messageFields: Map<any, any> = await Promise.all(messageTypes.map(async (messageType: any) => {
        return {
          [messageType]: await this.getMessageField(messageType)
        };
      })).then((results) => {
        return Object.assign({}, ...results);
      });

      
    } catch (error) {
      
    }
  }

  private async getMessageField(messageType: any){
    return new Promise((resolve, reject)=>{
      //let cpRosMsg = cp.spawn('rosmsg', ['show', messageType]);
      let cpRosMsg = cp.exec(`rosmsg show ${messageType}`);
      let messageFields:any = {};

      cpRosMsg.stdout?.on('data', (data) =>{
        if(data){
          messageFields = this.parseStringToObject(data.toString());
        }
      });
      cpRosMsg.on('close', (code) => {
        if (code === 0) {
          resolve(messageFields);
        } else {
          reject(new Error(`rosmsg show exited with code ${code}`));
        }
      });
    });
  }

  parseStringToObject(str: string): ParsedObject {
    const lines = str.split('\n');
    const parsedObject: ParsedObject = {};
    const stack: ParsedObject[] = [parsedObject];
  
    for (const line of lines) {
      const trimmedLine = line.trim();
      if (trimmedLine !== "") {
        const depth = line.search(/\S/);
        const match = trimmedLine.match(/^([\w/]+(?:\[\])?)\s+(\w+)(?:\s+\w+)?$/);
  
        if (match) {
          const [_, type, name] = match;
          const isArray = type.endsWith('[]');
          const valueType = isArray ? type.slice(0, -2) : type;
          const value = isArray ? [] : this.defaultValue(valueType);
          //const value = isArray ? [{}] : this.defaultValue(valueType);
  
          // Find the parent object or array for this property
          let parent = stack[Math.floor(depth / 2)];
          if (!parent) {
            throw new Error(`Invalid depth ${depth}`);
          }

          if (Array.isArray(parent)) {
            if(!parent[0]){
              parent[0] = {};
            }
            //parent.push({[name] : value});
            parent[0][name] = value;
          } 
          else{
            parent[name] = value;
          }
            
          stack[Math.floor(depth / 2) + 1] = value;
        }
      }
    }

    return parsedObject;
  }

 defaultValue(type: string): any {
    switch (type) {
      case 'bool':
        return false;
      case 'byte':
      case 'int8':
      case 'uint8':
      case 'int16':
      case 'uint16':
        return 0;
      case 'char':
        return '';
      case 'int32':
      case 'uint32':
        return 0;
      case 'int64':
      case 'uint64':
        return BigInt(0);
      case 'float32':
      case 'float64':
        return 0.0;
      case 'string':
        return '';
      case 'time':
      case 'duration':
        return { sec: 0, nsec: 0 };
      default:
        return {};
    }
  }

  private async getMessageTypes(){
    return new Promise((resolve, reject) => {
      let cpRosList = cp.spawn('rosmsg', ['list']);
      let messageTypes: any = [];
  
      cpRosList.stdout?.on('data', (data) => {
        if (data) {
          const msgList = data.toString().trim().split('\n');
          messageTypes.push(...msgList);
        }
      });
  
      cpRosList.on('close', (code) => {
        if (code === 0) {
          resolve(messageTypes);
        } else {
          reject(new Error(`rosmsg list exited with code ${code}`));
        }
      });
    });
  }

  private _getHtmlForWebview(webview: vscode.Webview) {
    const styleResetUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "reset.css")
    );
    const styleVSCodeUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "styles", "vscode.css")
    );

    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "out", "compiled/topicmonitor.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "topicmonitor.css")
    );

    // // Use a nonce to only allow specific scripts to be run
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