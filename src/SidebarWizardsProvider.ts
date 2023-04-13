import * as vscode from "vscode";
import { getNonce } from "./getNonce";
import * as dbcontroller from "./database/dbcontroller";
import { RosPackageQuickPick } from "./RosPackages/RosPackage";


export class SidebarWizardsProvider implements vscode.WebviewViewProvider {
  _view?: vscode.WebviewView;
  _doc?: vscode.TextDocument;

  constructor(private readonly _extensionUri: vscode.Uri) {}

  public resolveWebviewView(webviewView: vscode.WebviewView) {
    this._view = webviewView;

    webviewView.webview.options = {
      // Allow scripts in the webview
      enableScripts: true,
      enableCommandUris: true,

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
        case "r-ide.command": {
          vscode.commands.executeCommand(data.value.command, ...data.value.args);
          break;
        }
        case "getWorkspace": {
          // TODO: 0 or 2+ workspaces breaks this
          webviewView.webview.postMessage({
            type: 'setWorkspace',
            value: vscode.workspace.workspaceFolders?.map(folder => folder.uri.path).toString(),
          });
          break;
        }
        case "openFileExplorer" :{
          RosPackageQuickPick(true).then((result) => {
            console.log(result);
            if(result !== undefined){
              const folderPath = result.value?.rootDirectory.fsPath;
              webviewView.webview.postMessage({
                type: 'setWorkspace',
                value: folderPath,
              });
            }
          });
          break;
        }
        case "addEventToDB" : {
          dbcontroller.addNewEvent(data.value, true);
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

    /*
    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "src", "webviews/sidebarwizards.js")
    );
    */
    const scriptUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "out", "compiled/sidebarwizards.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "styles", "sidebarwizards.css")
    );

    // TODO: Handle 0 or 2+ folder workspaces open
    //const workspaceDirectory = vscode.workspace.workspaceFolders?.map(folder => folder.uri.path);


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
			<script nonce="${nonce}" src="${scriptUri}">
      </script>
			</body>
			</html>`;
  }
}