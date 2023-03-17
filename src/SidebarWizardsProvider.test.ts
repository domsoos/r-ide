import * as vscode from "vscode";
import { SidebarWizardsProvider } from "./SidebarWizardsProvider";
import { expect } from "chai";
import { beforeEach, afterEach, describe, it } from "mocha";
import * as sinon from "sinon";

describe("SidebarWizardsProvider", () => {
  let sandbox: sinon.SinonSandbox;
  let webviewView: vscode.WebviewView;
  let sidebarWizardsProvider: SidebarWizardsProvider;

  beforeEach(() => {
    sandbox = sinon.createSandbox();
    webviewView = {} as vscode.WebviewView;
    sidebarWizardsProvider = new SidebarWizardsProvider(vscode.Uri.file(""));
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

      sandbox.stub(webviewView.webview, "postMessage");
      sandbox.stub(webviewView.webview, "onDidReceiveMessage");

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
