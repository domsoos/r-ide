import * as vscode from "vscode";
import { getNonce } from "./getNonce";

export namespace SBTP{

    // this represents an item and it's children (like nested items)
    // we implement the item later
    class tree_item extends vscode.TreeItem 
    {
      readonly file: string | undefined;
      readonly line: number | undefined;

      // children represent branches, which are also items 
      public children: tree_item[] = [];

      constructor(label: string, file: string, line: number) {
        super(label, vscode.TreeItemCollapsibleState.None);
        this.file = file;
        this.line = line;
        this.collapsibleState = vscode.TreeItemCollapsibleState.None;
      }

      // a public method to add childs, and with additional branches
      // we want to make the item collabsible
      public add_child (child : tree_item) {
        this.collapsibleState = vscode.TreeItemCollapsibleState.Collapsed;
        this.children.push(child);
      }
    }
    
    // tree_view will created in our entry point
    export class tree_view implements vscode.TreeDataProvider<tree_item>
    {
        // m_data holds all tree items 
        private m_data : tree_item [] = [];
        // with the vscode.EventEmitter we can refresh our  tree view
        private m_onDidChangeTreeData: vscode.EventEmitter<tree_item | undefined> = new vscode.EventEmitter<tree_item | undefined>();
        // and vscode will access the event by using a readonly onDidChangeTreeData (this member has to be named like here, otherwise vscode doesnt update our treeview.
        readonly onDidChangeTreeData ? : vscode.Event<tree_item | undefined> = this.m_onDidChangeTreeData.event;
        
        
        // in the constructor we register a refresh and item clicked function
        public constructor() 
        {
            vscode.commands.registerCommand('r-ide.topic-clicked', r => this.item_clicked(r));
            vscode.commands.registerCommand('r-ide.topic-refresh', () => this.refresh());
        }
    
        // this is called when we click an item
        public item_clicked(item: tree_item) {
          // we implement this later
      }
    
        // this is called whenever we refresh the tree view
        public refresh() {
          if (vscode.workspace.workspaceFolders) {
              this.m_data = [];
              this.m_data.push(new tree_item("/rosout_agg","" ,0 ));
              this.m_data.push(new tree_item("/tf","" ,0 ));
              let item = new tree_item("/zed2","" ,0);
              let item2 = new tree_item("/zed_node", "", 0);
              item.add_child(item2);
              item2.add_child(new tree_item("/atm_press", "", 0));
              item2.add_child(new tree_item("/confidence", "", 0));
              item2.add_child(new tree_item("/depth", "", 0));
              item2.add_child(new tree_item("/disparity", "", 0));
              item2.add_child(new tree_item("/imu", "", 0));
              item2.add_child(new tree_item("/left", "", 0));
              this.m_data.push(item);
              
              //this.read_directory(vscode.workspace.workspaceFolders[0].uri.fsPath);
              this.m_onDidChangeTreeData.fire(undefined);
          } 
        }
        // we need to implement getTreeItem to receive items from our tree view
        public getTreeItem(element: tree_item): vscode.TreeItem|Thenable<vscode.TreeItem> {
          const item = new vscode.TreeItem(element.label!, element.collapsibleState);
          return item;
        }
    
        // and getChildren
        public getChildren(element : tree_item | undefined): vscode.ProviderResult<tree_item[]> {
          if (element === undefined) {
              return this.m_data;
          } else {
              return element.children;
          }
        }

        /*
        // read the directory recursively over all files
        private read_directory(dir: string) {
          fs.readdirSync(dir).forEach(file => {
              let current = path.join(dir,file);
              if (fs.statSync(current).isFile()) {
                  if(current.endsWith('.feature')) {
                      this.parse_feature_file(current);
                  } 
              } else {
                  this.read_directory(current)
              }
          });
      }

      private parse_feature_file(file: string) {
        const regex_feature = new RegExp("(?<=Feature:).*");
        const regex_scenario = new RegExp("(?<=Scenario:).*");
        let reader = rd.createInterface(fs.createReadStream(file))
        const line_counter = ((i = 0) => () => ++i)();

        // let's loop over every line
        reader.on("line", (line : string, line_number : number = line_counter()) => {
            let is_feature = line.match(regex_feature);
            if (is_feature) {
                // we found a feature and add this to our tree view data
                this.m_data.push(new tree_item(is_feature[0], file, line_number));
            }
            let is_scenario = line.match(regex_scenario);
            if (is_scenario) {
                // every following scenario will be added to the last added feature with add_children from the tree_item
                this.m_data.at(-1)?.add_child(new tree_item(is_scenario[0], file, line_number));
            }
        });
    }
    */

    }

  export class SidebarTopicsProvider implements vscode.WebviewViewProvider {
    _view?: vscode.WebviewView;
    _doc?: vscode.TextDocument;

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
        }
      });
    }

    public revive(panel: vscode.WebviewView) {
      this._view = panel;
    }

    private _getHtmlForWebview(webview: vscode.Webview) {
      const styleResetUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "src", "styles/reset.css")
      );
      const styleVSCodeUri = webview.asWebviewUri(
          vscode.Uri.joinPath(this._extensionUri, "src", "styles/vscode.css")
      );

      const scriptUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "out", "compiled/sidebartopics.js")
      );

      const styleMainUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, "src", "styles/sidebartopics.css")
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
}