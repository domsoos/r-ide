// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
import * as vscode from 'vscode';
import { SidebarBagsProvider } from './SidebarBagsProvider';
import { SBTP } from './SidebarTopicsProvider';
import { SidebarVisualsProvider } from './SidebarVisualsProvider';
import { SidebarWizardsProvider } from './SidebarWizardsProvider';
import { TopicMonitorProvider } from './TopicMonitorProvider';
import * as dbcontroller from './database/dbcontroller';
import { 
	createFileFromTemplate,
	createMessage,
	createSrv
} from './commands/commands';
import { 
	addMsgToPackage, addSrvToPackage, createRosPackage, loadPackages, registerPackage
} from './commands/RosPackage';

// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
export function activate(context: vscode.ExtensionContext) {
	dbcontroller.connectToDB();

	const sidebarWizardsProvider = new SidebarWizardsProvider(context.extensionUri);
	const sidebarBagsProvider = new SidebarBagsProvider(context.extensionUri);
	const sidebarVisualsProvider = new SidebarVisualsProvider(context.extensionUri);
	//const sidebarTopicsProvider = new SBTP.SidebarTopicsProvider(context.extensionUri);
	const topicTree = new SBTP.tree_view();

	loadPackages();
	

	context.subscriptions.push(
		// Webviews
		vscode.window.registerWebviewViewProvider(
			"sidebar-wizards",
			sidebarWizardsProvider
		),
		vscode.window.registerWebviewViewProvider(
		  "sidebar-bags",
		  sidebarBagsProvider
		),
		vscode.window.registerWebviewViewProvider(
			"sidebar-visuals",
			sidebarVisualsProvider
		),
		/*
		vscode.window.registerWebviewViewProvider(
			"sidebar-topics",
			sidebarTopicsProvider
		),

		// Commands

		// Create templates and directories
		*/
		vscode.commands.registerCommand(
			"r-ide.create-file-from-template",
			createFileFromTemplate
		),
		vscode.commands.registerCommand(
			"r-ide.create-package",
			createRosPackage
		),
		vscode.commands.registerCommand(
			"r-ide.create-msg", 
			createMessage
		),
		vscode.commands.registerCommand(
			"r-ide.create-srv",
			createSrv
		),

		// Register files and packages
		vscode.commands.registerCommand(
			"r-ide.register-package",
			registerPackage
		),
		vscode.commands.registerCommand(
			"r-ide.register-msg",
			addMsgToPackage
		),
		vscode.commands.registerCommand(
			"r-ide.register-srv",
			addSrvToPackage
		),

		// Workspace listener
		vscode.workspace.onDidChangeTextDocument((event) => {
			switch (event.document.languageId) {
				case ('python'): {
					break;
				}

				case ('cpp'): {
					break;
				}

				case ('ros.msg'): {
					break;
				}

				default: {
					console.log(event.document.languageId);
					break;
				}
			}
		}),
		/*
		vscode.commands.registerCommand("r-ide.open-topic-monitor", () => {
			TopicMonitorProvider.createOrShow(context.extensionUri);
		}),
		*/
		vscode.window.registerTreeDataProvider('topic-tree-view', topicTree)
	  );
	  topicTree.refresh();
}

// This method is called when your extension is deactivated
export function deactivate() {}
