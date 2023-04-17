import { before, describe, it, after } from 'mocha';
import { expect } from "chai";
import { SidebarVisualsProvider } from '../../SidebarVisualsProvider';
import { SidebarWizardsProvider } from '../../SidebarWizardsProvider';
import { SidebarTopicsProvider } from '../../SidebarTopicsProvider';
import { SidebarBagsProvider } from '../../SidebarBagsProvider';
import { NodeGraphProvider } from '../../NodeGraphProvider';
import { TopicMonitorProvider } from '../../TopicMonitorProvider';

import * as vscode from 'vscode';

suite('Provider Creation Tests', () => {
    let mockUri: vscode.Uri;
    let sidebarVisualsProvider: any;
    let sidebarWizardsProvider: any;
    let sidebarTopicsProvider: any;
    let sidebarBagsProvider: any;
    let nodeGraphProvider: any;
    let topicMonitorProvider: any;

    const fakeWebviewView: vscode.WebviewView = {
      webview: {
        asWebviewUri: (uri) => uri,
        postMessage: (message) => Promise.resolve(true),
        onDidReceiveMessage: new vscode.EventEmitter<any>().event,
        cspSource: '',
        html: '',
        options: { enableScripts: false },
      },
      viewType: '',
      title: '',
      description: '',
      visible: false,
      onDidChangeVisibility: new vscode.EventEmitter<void>().event,
      onDidDispose: new vscode.EventEmitter<void>().event,
      show: function (preserveFocus?: boolean | undefined): void {
        throw new Error('Function not implemented.');
      }
    };

    before(async () => {
      mockUri = vscode.Uri.parse('file:///path/to/mock.txt');
      sidebarVisualsProvider = new SidebarVisualsProvider(mockUri);
      sidebarWizardsProvider = new SidebarWizardsProvider(mockUri);
      sidebarTopicsProvider = new SidebarTopicsProvider(mockUri);

      //sidebarBagsProvider = new SidebarBagsProvider(mockUri);

      //console.warn = originalWarn;
      //nodeGraphProvider = new NodeGraphProvider(mockUri);
      //topicMonitorProvider = new TopicMonitorProvider(mockUri);
    });

    it('create an instance of SidebarVisualsProvider', async () => {
        expect(sidebarVisualsProvider).to.be.an.instanceOf(SidebarVisualsProvider);
    });

    it('create an instance of SidebarWizardsProvider', async () => {
      expect(sidebarWizardsProvider).to.be.an.instanceOf(SidebarWizardsProvider);
    });

    it('create an instance of SidebarTopicsProvider', async () => {
      expect(sidebarTopicsProvider).to.be.an.instanceOf(SidebarTopicsProvider);
    });

    it('create an instance of SidebarBagsProvider', async () => {
      //expect(sidebarBagsProvider).to.be.an.instanceOf(SidebarBagsProvider);
    });



    it('create a webview for SidebarVisualsProvider', async () => {
      let webview = vscode.window.registerWebviewViewProvider(
        "sidebar-visuals",
        sidebarVisualsProvider
		  );
      
      sidebarVisualsProvider.resolveWebviewView(fakeWebviewView);

      expect(webview).to.be.an.instanceOf(vscode.Disposable);
    });

    it('create a webview for SidebarWizardsProvider', async () => {
      let webview = vscode.window.registerWebviewViewProvider(
        "sidebar-wizards",
        sidebarWizardsProvider
		  );

      expect(webview).to.be.an.instanceOf(vscode.Disposable);
    });

    it('create a webview for SidebarTopicsProvider', async () => {
      let webview = vscode.window.registerWebviewViewProvider(
        "sidebar-topics",
        sidebarTopicsProvider
		  );

      expect(webview).to.be.an.instanceOf(vscode.Disposable);
    });

    it('create a webview for SidebarBagsProvider', async () => {
      let webview = vscode.window.registerWebviewViewProvider(
        "sidebar-bags",
        sidebarBagsProvider
		  );

      expect(webview).to.be.an.instanceOf(vscode.Disposable);
    });

});

  /*
  describe('Options tests', () => { // the tests container
    it('checking default options', () => { // the single test
        let goat = "goat";

        expect(false).to.be.false; // Do I need to explain anything? It's like writing in English!
        expect(goat).to.be.a("string");

        expect(30).to.equal(30); // As I said 3 lines above

        expect(["1"]).to.be.empty; // emitters property is an array and for this test must be empty, this syntax works with strings too
        expect({color: "#fff"}).to.be.an("object").to.have.property("value").to.equal("#fff"); // this is a little more complex, but still really clear
    });
  });
  */