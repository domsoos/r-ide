import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import { Rosbag } from "./RosBag/rosbag";

export class SidebarBagsProvider implements vscode.WebviewViewProvider {
  _view?: vscode.WebviewView;
  _doc?: vscode.TextDocument;

  bag: Rosbag | undefined;

  constructor(private readonly _extensionUri: vscode.Uri) {
    this.bag = undefined;
    Rosbag.connect();
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
              await this.bag?.clearBag();
              this.bag = new Rosbag(result[0].fsPath, webviewView.webview);
              await this.bag.openBag();
              webviewView.webview.postMessage({
                type: 'setSelectedBag',
                value: {
                  path: result[0].fsPath,
                  start: this.bag.bag!.startTime,
                  end: this.bag.bag!.endTime
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
          this.bag?.playBag();
          break;
        }
        case "pauseBag": {
          this.bag?.pauseBag();
          break;
        }
        case "stopBag": {
          this.bag?.stopBag();
        }
        case "replayBag" : {
          this.bag?.replayBag();
          break;
        }
        case "isROSConnected":{
          let isConnected = await this.isROSConnected();
          if(!isConnected){
            vscode.commands.executeCommand('r-ide.no-ros-connection');
          }else{
            webviewView.webview.postMessage({
              type: 'ROSConnectionSuccessful',
            });
          }
          break;
        }
        case "replayBag": {
          this.bag?.replayBag();
          break;
        }
        case "closeBag": {
          this.bag?.clearBag();
          break;
        }
        case "cloneConfirm": {
          const {newBagPath, startTime, endTime, verbose, topics, copy} = data.values;
          let result: any = this.bag!.clone(newBagPath, startTime, endTime, verbose, topics);

          if (copy) {
            // Copy the contents to the clipboard
            await vscode.env.clipboard.writeText(result);
          } else {
            // Write contents to the terminal
            const terminal = vscode.window.createTerminal();
            terminal.show();
            terminal.sendText(result);
          }

          break;
        }
      }
    });
  }

  private async isROSConnected(){
    return await Rosbag.isROSConnected();
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