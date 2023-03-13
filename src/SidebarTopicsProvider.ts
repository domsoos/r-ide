import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import * as path from 'path';



export class SidebarTopicsProvider implements vscode.WebviewViewProvider {
  _view?: vscode.WebviewView;
  _doc?: vscode.TextDocument;
  //activeTopics: any = [];

  constructor(private readonly _extensionUri: vscode.Uri) {}


  public resolveWebviewView(webviewView: vscode.WebviewView) {
    this._view = webviewView;

    webviewView.webview.options = {
      // Allow scripts in the webview
      enableScripts: true,

      localResourceRoots: [this._extensionUri],
    };

    webviewView.webview.html = this._getHtmlForWebview(webviewView.webview);

    webviewView.webview.onDidReceiveMessage(async (data) => {
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
        case "openTopicMonitor":{
          vscode.commands.executeCommand("r-ide.open-topic-monitor");
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
      vscode.Uri.joinPath(this._extensionUri, "out", "compiled/sidebartopics.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "sidebartopics.css")
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



/*
let ROS = ROSManager.getInstance();
if(ROS.isConnected()){
  let rosAPI = ROS.getROSApi();
*/
































/**
 * TREE VIEW  
 
// tree_view will created in our entry point
export class TreeView implements vscode.TreeDataProvider<TreeItem>
{
    // treeData holds all tree items 
    private treeData : TreeItem [] = [];
    // with the vscode.EventEmitter we can refresh our  tree view
    private _onDidChangeTreeData: vscode.EventEmitter<TreeItem | undefined | void> = new vscode.EventEmitter<TreeItem | undefined | void>();
    readonly onDidChangeTreeData: vscode.Event<TreeItem | undefined | void> = this._onDidChangeTreeData.event;
    
    
    // in the constructor we register a refresh and item clicked function
    public constructor() 
    {
        vscode.commands.registerCommand('r-ide.topic-clicked', r => this.itemClicked(r));
        vscode.commands.registerCommand('r-ide.topic-refresh', () => this.refresh());
        vscode.commands.registerCommand('r-ide.reconnect-ros-master', () =>{
          let ros = ROSManager.getInstance();
          ros.reconnect();
          this.refresh();
        });
        this.refresh();
    }
    // this is called when we click an item
    public itemClicked(item: TreeItem) {
      // we implement this later
  }
    // this is called whenever we refresh the tree view
    public async refresh() {
      this.treeData = [];
      let ros = ROSManager.getInstance();
      if(ros.isConnected()){
        await ros.getTopics().then( res => {
          if(res){
            for (let i = 0; i < res.topics.length; i++){
              this.treeData.push(new TreeItem(res.topics[i], res.topics[i] , res.types[i] ));
            }
          }
        });
      }
      this._onDidChangeTreeData.fire();
    }
    // we need to implement getTreeItem to receive items from our tree view
    public getTreeItem(element: TreeItem): vscode.TreeItem|Thenable<vscode.TreeItem> {
      const item = new vscode.TreeItem(element.label!, element.collapsibleState);
      return item;
    }
    // and getChildren
    public getChildren(element : TreeItem | undefined): vscode.ProviderResult<TreeItem[]> {
      if (element === undefined) {
          return this.treeData;
      } else {
          return element.children;
      }
    }
}
export class TreeItem extends vscode.TreeItem 
{ 
  readonly topic: string | undefined;
  readonly type: string | undefined;
  public children: TreeItem[] = [];
  constructor(label: string, topic: string, type: string) {
    super(label, vscode.TreeItemCollapsibleState.None);
    this.topic = topic;
    this.type = type;
    this.iconPath = {
      light: path.join(__filename, '..', '..', 'resources', 'light', 'dependency.svg'),
      dark: path.join(__filename, '..', '..', 'resources', 'dark', 'dependency.svg')
    };
  }
  contextValue = 'tree-topic';
  // a public method to add childs, and with additional branches
  // we want to make the item collabsible
  public addChild (child : TreeItem) {
    this.collapsibleState = vscode.TreeItemCollapsibleState.Collapsed;
    this.children.push(child);
  }
}
**Package.Json
    "viewsWelcome": [
      {
        "view": "topic-tree-view",
        "contents": "Not Connected to ROS Master. \n[Try again](command:r-ide.reconnect-ros-master)"
      }
    ]
    "menus": {
      "view/title": [
        {
          "command": "r-ide.topic-refresh",
          "when": "view == topic-tree-view",
          "group": "navigation"
        }
          ]
      "view/item/context": [
        {
          "command": "r-ide.topic-listen",
          "when": "view == topic-tree-view",
          "group": "inline"
        }
      ]
    }
      views-
              {
          "id": "topic-tree-view",
          "name": "ROS Topic Monitor",
          "visibility": "collapsed",
          "initialSize": 1
        }
      commands- 
            {
        "command": "r-ide.topic-clicked",
        "title": "R-IDE: Select Topic"
      },
      {
        "command": "r-ide.topic-refresh",
        "title": "R-IDE: Refresh Topics",
        "icon": {
          "light": "resources/light/refresh.svg",
          "dark": "resources/dark/refresh.svg"
        }
      },
      {
        "command": "r-ide.topic-listen",
        "title": "R-IDE: Listen to Topic",
        "icon": "$(eye-watch)"
      },
      {
        "command": "r-ide.topic-stop-listen",
        "title": "R-IDE: Stop Listening to Topic",
        "icon": "$(eye-closed)"
      },
      {
        "command": "r-ide.reconnect-ros-master",
        "title": "R-IDE: Reconnect ROS Master"
      }
  **Extension.js
  		vscode.window.registerTreeDataProvider('topic-tree-view', new TreeView())
*/