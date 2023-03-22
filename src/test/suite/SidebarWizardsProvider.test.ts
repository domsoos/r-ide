import * as vscode from 'vscode';
import { SidebarWizardsProvider } from "../../SidebarWizardsProvider";
import { expect } from "chai";
import { beforeEach, afterEach, describe, it } from "mocha";
import * as sinon from "sinon";
<<<<<<< HEAD
=======
import * as path from 'path';

console.log(process.env);
>>>>>>> 4b2db5459bac18256214c535f00566827b9a9ec0

describe("SidebarWizardsProvider", () => {
  let sandbox: sinon.SinonSandbox;
  let webviewView: vscode.WebviewView;
  let sidebarWizardsProvider: SidebarWizardsProvider;
<<<<<<< HEAD

  beforeEach(() => {
    sandbox = sinon.createSandbox();

    const mockWebview = {
      postMessage: sandbox.stub().resolves(true),
      onDidReceiveMessage: sandbox.stub(),
    } as unknown as vscode.Webview;

    webviewView = {
      webview: mockWebview,
      show: sandbox.stub(),
      onDidChangeVisibility: new vscode.EventEmitter<void>().event,
    } as unknown as vscode.WebviewView;

    sidebarWizardsProvider = new SidebarWizardsProvider(vscode.Uri.file(""));
=======
  const extensionUri = vscode.Uri.file("");

  beforeEach(() => {
    
    sandbox = sinon.createSandbox();
    sidebarWizardsProvider = new SidebarWizardsProvider(extensionUri);
    webviewView = sidebarWizardsProvider._view!;
    
>>>>>>> 4b2db5459bac18256214c535f00566827b9a9ec0
  });

  afterEach(() => {
    sandbox.restore();
  });

  describe("resolveWebviewView", () => {
    it("should set webview options and html", () => {
      const options = {
        enableScripts: true,
        enableCommandUris: true,
        localResourceRoots: [vscode.Uri.file("")],
      };

      const expectedHtml = "<html><body><h1>Hello World!</h1></body></html>";

<<<<<<< HEAD
=======
      sandbox.stub(webviewView.webview, "postMessage");
      sandbox.stub(webviewView.webview, "onDidReceiveMessage");

>>>>>>> 4b2db5459bac18256214c535f00566827b9a9ec0
      sidebarWizardsProvider.resolveWebviewView(webviewView);

      expect(webviewView.webview.options).to.deep.equal(options);
      expect(webviewView.webview.html).to.equal(expectedHtml);
    });

    it("should handle onDidReceiveMessage messages", () => {
      const message = {
        type: "onInfo",
        value: "Hello World!",
      };

      const onDidReceiveMessageStub = sandbox.stub(
        webviewView.webview,
        "onDidReceiveMessage"
      );

      sidebarWizardsProvider.resolveWebviewView(webviewView);

      const [, onDidReceiveMessageCallback] = onDidReceiveMessageStub.firstCall.args;

      onDidReceiveMessageCallback(message);

<<<<<<< HEAD
=======

>>>>>>> 4b2db5459bac18256214c535f00566827b9a9ec0
      const postMessageSpy = sinon.spy(webviewView.webview, 'postMessage');

      sinon.assert.calledWith(postMessageSpy, {
        command: 'refresh',
        type: "onInfo",
        value: "Hello World!",
      });
    });
  });

  describe("revive", () => {
    it("should set the view", () => {
      sidebarWizardsProvider.revive(webviewView);

      expect(sidebarWizardsProvider["_view"]).to.equal(webviewView);
    });
  });
});
