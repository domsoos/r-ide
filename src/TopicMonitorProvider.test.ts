import * as vscode from 'vscode';
import * as sinon from 'sinon';
import { expect } from 'chai';
import { TopicMonitorProvider } from './TopicMonitorProvider';


suite('TopicMonitorProvider', () => {
  let sandbox: sinon.SinonSandbox;
  let panel: vscode.WebviewPanel;
  let extensionUri: vscode.Uri;

  setup(() => {
    sandbox = sinon.createSandbox();
    panel = {} as vscode.WebviewPanel;
    extensionUri = vscode.Uri.parse('extension://test');
  });

  teardown(() => {
    sandbox.restore();
  });

  test('createOrShow should create a new panel if current panel is not defined', () => {
    const createWebviewPanelStub = sandbox.stub(vscode.window, 'createWebviewPanel').returns(panel);
    TopicMonitorProvider.createOrShow(extensionUri);

    expect(createWebviewPanelStub.calledOnce).to.be.true;
    expect(TopicMonitorProvider.currentPanel).to.be.an.instanceOf(TopicMonitorProvider);
  });

  test('createOrShow should reveal the current panel if it exists', () => {
    TopicMonitorProvider.currentPanel = new TopicMonitorProvider(panel, extensionUri);
    const revealStub = sandbox.stub(panel, 'reveal');
    TopicMonitorProvider.createOrShow(extensionUri);

    expect(revealStub.calledOnce).to.be.true;
  });

  test('kill should dispose the current panel and set it to undefined', () => {
    TopicMonitorProvider.currentPanel = new TopicMonitorProvider(panel, extensionUri);
    const disposeStub = sandbox.stub(panel, 'dispose');
    TopicMonitorProvider.kill();

    expect(disposeStub.calledOnce).to.be.true;
    expect(TopicMonitorProvider.currentPanel).to.be.undefined;
  });

  test('revive should create a new instance of TopicMonitorProvider', () => {
    const instance = TopicMonitorProvider.revive(panel, extensionUri);

    expect(instance).to.be.an.instanceOf(TopicMonitorProvider);
  });

  test('dispose should dispose the panel and all the disposables', () => {
    const disposable1 = { dispose: sandbox.stub() };
    const disposable2 = { dispose: sandbox.stub() };
    const disposeStub = sandbox.stub(panel, 'dispose');
    const instance = new TopicMonitorProvider(panel, extensionUri);
    instance['_disposables'] = [disposable1, disposable2];
    instance.dispose();

    expect(disposeStub.calledOnce).to.be.true;
    expect(TopicMonitorProvider.currentPanel).to.be.undefined;
    expect(disposable1.dispose.calledOnce).to.be.true;
    expect(disposable2.dispose.calledOnce).to.be.true;
  });

  test('_getHtmlForWebview should return valid HTML', () => {
    const nonce = 'test-nonce';
    sandbox.stub(panel.webview, 'asWebviewUri').callsFake((uri) => uri);
    const html = new TopicMonitorProvider(panel, extensionUri)['_getHtmlForWebview'](panel.webview);

    expect(html).to.include(`nonce="${nonce}"`);
  });
});
