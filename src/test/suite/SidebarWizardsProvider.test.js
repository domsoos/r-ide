"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
Object.defineProperty(exports, "__esModule", { value: true });
const vscode = __importStar(require("vscode"));
const SidebarWizardsProvider_1 = require("../../SidebarWizardsProvider");
const chai_1 = require("chai");
const sinon = __importStar(require("sinon"));
describe("SidebarWizardsProvider", () => {
    let sandbox;
    let webviewView;
    let sidebarWizardsProvider;
    beforeEach(() => {
        sandbox = sinon.createSandbox();
        const mockWebview = {
            postMessage: sandbox.stub().resolves(true),
            onDidReceiveMessage: sandbox.stub(),
        };
        webviewView = {
            webview: mockWebview,
            show: sandbox.stub(),
            onDidChangeVisibility: new vscode.EventEmitter().event,
        };
        sidebarWizardsProvider = new SidebarWizardsProvider_1.SidebarWizardsProvider(vscode.Uri.file(""));
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
            sidebarWizardsProvider.resolveWebviewView(webviewView);
            (0, chai_1.expect)(webviewView.webview.options).to.deep.equal(options);
            (0, chai_1.expect)(webviewView.webview.html).to.equal(expectedHtml);
        });
        it("should handle onDidReceiveMessage messages", () => {
            const message = {
                type: "onInfo",
                value: "Hello World!",
            };
            const onDidReceiveMessageStub = sandbox.stub(webviewView.webview, "onDidReceiveMessage");
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
            (0, chai_1.expect)(sidebarWizardsProvider["_view"]).to.equal(webviewView);
        });
    });
});
//# sourceMappingURL=SidebarWizardsProvider.test.js.map