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
const sinon = __importStar(require("sinon"));
const chai_1 = require("chai");
const TopicMonitorProvider_1 = require("../../TopicMonitorProvider");
describe('TopicMonitorProvider', () => {
    let sandbox;
    let panel;
    let extensionUri;
    beforeEach(() => {
        sandbox = sinon.createSandbox();
        panel = {};
        extensionUri = vscode.Uri.parse('extension://test');
    });
    afterEach(() => {
        sandbox.restore();
    });
    test('createOrShow should create a new panel if current panel is not defined', () => {
        const createWebviewPanelStub = sandbox.stub(vscode.window, 'createWebviewPanel').returns(panel);
        TopicMonitorProvider_1.TopicMonitorProvider.createOrShow(extensionUri);
        (0, chai_1.expect)(createWebviewPanelStub.calledOnce).to.be.true;
        (0, chai_1.expect)(TopicMonitorProvider_1.TopicMonitorProvider.currentPanel).to.be.an.instanceOf(TopicMonitorProvider_1.TopicMonitorProvider);
    });
    test('createOrShow should reveal the current panel if it exists', () => {
        TopicMonitorProvider_1.TopicMonitorProvider.currentPanel = new TopicMonitorProvider_1.TopicMonitorProvider(panel, extensionUri);
        const revealStub = sandbox.stub(panel, 'reveal');
        TopicMonitorProvider_1.TopicMonitorProvider.createOrShow(extensionUri);
        (0, chai_1.expect)(revealStub.calledOnce).to.be.true;
    });
    test('kill should dispose the current panel and set it to undefined', () => {
        TopicMonitorProvider_1.TopicMonitorProvider.currentPanel = new TopicMonitorProvider_1.TopicMonitorProvider(panel, extensionUri);
        const disposeStub = sandbox.stub(panel, 'dispose');
        TopicMonitorProvider_1.TopicMonitorProvider.kill();
        (0, chai_1.expect)(disposeStub.calledOnce).to.be.true;
        (0, chai_1.expect)(TopicMonitorProvider_1.TopicMonitorProvider.currentPanel).to.be.undefined;
    });
    test('revive should create a new instance of TopicMonitorProvider', () => {
        const instance = TopicMonitorProvider_1.TopicMonitorProvider.revive(panel, extensionUri);
        (0, chai_1.expect)(instance).to.be.an.instanceOf(TopicMonitorProvider_1.TopicMonitorProvider);
    });
    test('dispose should dispose the panel and all the disposables', () => {
        const disposable1 = { dispose: sandbox.stub() };
        const disposable2 = { dispose: sandbox.stub() };
        const disposeStub = sandbox.stub(panel, 'dispose');
        const instance = new TopicMonitorProvider_1.TopicMonitorProvider(panel, extensionUri);
        instance['_disposables'] = [disposable1, disposable2];
        instance.dispose();
        (0, chai_1.expect)(disposeStub.calledOnce).to.be.true;
        (0, chai_1.expect)(TopicMonitorProvider_1.TopicMonitorProvider.currentPanel).to.be.undefined;
        (0, chai_1.expect)(disposable1.dispose.calledOnce).to.be.true;
        (0, chai_1.expect)(disposable2.dispose.calledOnce).to.be.true;
    });
    test('_getHtmlForWebview should return valid HTML', () => {
        const nonce = 'test-nonce';
        sandbox.stub(panel.webview, 'asWebviewUri').callsFake((uri) => uri);
        const html = new TopicMonitorProvider_1.TopicMonitorProvider(panel, extensionUri)['_getHtmlForWebview'](panel.webview);
        (0, chai_1.expect)(html).to.include(`nonce="${nonce}"`);
    });
});
//# sourceMappingURL=TopicMonitorProvider.test.js.map