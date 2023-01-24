import * as vscode from "vscode";
import { getNonce } from "./getNonce";


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
      vscode.Uri.joinPath(this._extensionUri, "src", "webviews/sidebarwizards.js")
    );

    const styleMainUri = webview.asWebviewUri(
      vscode.Uri.joinPath(this._extensionUri, "src", "styles/sidebarwizards.css")
    );

    const workspaceDirectory = vscode.workspace.workspaceFolders?.map(folder => folder.uri.path);
    


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
      <div class="dropdown">
        <button id="create-new-btn">Create New</button>
        <div class="dropdown-content" id="dropdown-content">
          <button id="ros-node-btn">ROS Node</button>
          <button id="ros-msg-btn">ROS Msg</button>
          <button id="ros-srv-btn">ROS Srv</button>
        </div>
      </div>
      <div class="wizard-container" id="wizard-container">
        <h3 style="text-align: center;" id="wizard-title">Creation Wizard</h3>
        <hr>
        <label for="wizard-file-type">File type:</label>
        <br>
        <select name="wizard-file-type" id="wizard-file-type" class="width-100 margin-top-5">
          <option value="cplusplus">C++</option>
          <option value="python">Python</option>
        </select>
        <br>
        <br>
        <label for="wizard-node-name">Node name:</label>
        <input type="text" id="wizard-node-name" class="margin-top-5">
        <br>
        <label for="wizard-node-location">Node location:</label>
        <input type="text" id="wizard-node-location" class="margin-top-5" value="${workspaceDirectory}">
        <br>
        <input type="checkbox" name="publisher" id="publisher">
        <label for="wizard-node-publisher">Publisher</label>
        <br>
        <input type="checkbox" name="subscriber" id="subscriber" class="margin-top-5">
        <label for="wizard-node-publisher">Subscriber</label>
        <br>
        <br>
        <button class="cancel-btn" id="cancel-btn">Cancel</button>
        <button class="next-btn" id="next-btn">Next</button>
      </div>

			<script nonce="${nonce}" src="${scriptUri}"></script>
			</body>
			</html>`;
  }
}