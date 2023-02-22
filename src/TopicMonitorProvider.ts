import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import { ROSManager } from "./ROSManagers/ros";

export class TopicMonitorProvider {
  /**
   * Track the currently panel. Only allow a single panel to exist at a time.
   */
  public static currentPanel: TopicMonitorProvider | undefined;
  activeTopics: any = [];
  activeMediaTopic: any = null;

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
        case "getROSTopics": {
          let ros = this.getROS();
          if(ros.rosAPI){
            ros.rosAPI.getTopics((res: any) => {
                if (res) {
                  webview.postMessage({
                    type: 'setROSTopics',
                    success: true,
                    data: res,
                  });
                } else {
                  webview.postMessage({
                    type: 'setROSTopics',
                    success: false,
                  });
                }
            }, (err: any)=>{
              webview.postMessage({
                type: 'setROSTopics',
                success: false,
                data: err,
              });
            });
          }else{
            webview.postMessage({
              type: 'setROSTopics',
              success: false,
            });
          }
          break;
        }
        case 'pushActiveTopic':{
          let ros = this.getROS();
          if(ros.rosAPI && ros.rosLib){
            let topic = new ros.rosLib.Topic({
              ros : ros.rosAPI,
              name : data.value.topic,
              messageType : data.value.type
            });

            topic.subscribe(function(message: any) {
              webview.postMessage({
                type: 'messageFromTopic',
                data: message
              });
            });

            this.activeTopics.push(topic);
          }
          break;

        }
        case 'popActiveTopic':{
          let index = this.activeTopics.findIndex((obj: { name: any; }) => obj.name === data.value.topic);
          this.activeTopics[index].unsubscribe();
          this.activeTopics.splice(index, 1);
          break;
        }
        case 'subActiveMediaTopic':{
          let ros = this.getROS();
          if(ros.rosAPI && ros.rosLib){
            
            if(this.activeMediaTopic !== null){
              this.activeMediaTopic.unsubscribe();
              this.activeMediaTopic = null;
            }

            let topic = new ros.rosLib.Topic({
              ros : ros.rosAPI,
              name : data.value.topic,
              messageType : data.value.type
            });

            topic.subscribe(function(message: any) {
              webview.postMessage({
                type: 'setActiveMediaTopic',
                data: message
              });
            });

            this.activeMediaTopic = topic;
          }
          break;
        }
      }
    });
  }

  private getROS(){
    let ROS = ROSManager.getInstance();
    if(ROS.isConnected()){
      return ROS.getROSApi();
    }
    else{
      return false;
    }
  }

  private _getHtmlForWebview(webview: vscode.Webview) {
    const styleResetUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "src", "styles/reset.css")
    );
    const styleVSCodeUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "src", "styles/vscode.css")
    );

    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "out", "compiled/topicmonitor.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "src", "styles/topicmonitor.css")
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