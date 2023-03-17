import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import Bag, { open } from 'rosbag';

export class SidebarBagsProvider implements vscode.WebviewViewProvider {
  _view?: vscode.WebviewView;
  _doc?: vscode.TextDocument;

  constructor(private readonly _extensionUri: vscode.Uri) {}

  private async openBag(bagPath: string) {
    let bag: Bag = await open(bagPath);

    const packetSize = 10000;
    let messagePacket: any[] = [];

    // let messages: any[] = [];

    bag.readMessages({}, result => {
      const {topic, message, timestamp} = result;
      messagePacket.push({topic: topic, message: message, timestamp: timestamp});
      if (messagePacket.length >= packetSize) {
        // console.log(messagePacket);
        this._view?.webview.postMessage({type: 'getMessages', value: messagePacket});
        // this.messages = this.messages.concat(messagePacket);
        messagePacket = [];
      }
    }).then(() => {
      console.log('read all messages');
      this._view?.webview.postMessage({type: 'getMessages', value: messagePacket});
    });

    let connections = [];
    for (let conn in bag.connections) {
      connections.push(bag.connections[conn]);
    }

    this._view?.webview.postMessage({type: 'getConnections', value: connections});
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