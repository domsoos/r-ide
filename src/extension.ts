// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
import * as vscode from 'vscode';
import { SidebarBagsProvider } from './SidebarBagsProvider';
import { SidebarTopicsProvider } from './SidebarTopicsProvider';
import { SidebarVisualsProvider } from './SidebarVisualsProvider';
import { SidebarWizardsProvider } from './SidebarWizardsProvider';
import { 
	createFileFromTemplate,
	createMessage,
	createSrv
} from './commands/commands';
import { 
	addMsgToProject, addSrvToProject, createRosProject, registerProject, RosProject
} from './commands/RosProject';
import { identity } from 'svelte/internal';

// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
export function activate(context: vscode.ExtensionContext) {
	
	const sidebarWizardsProvider = new SidebarWizardsProvider(context.extensionUri);
	const sidebarBagsProvider = new SidebarBagsProvider(context.extensionUri);
	const sidebarVisualsProvider = new SidebarVisualsProvider(context.extensionUri);
	const sidebarTopicsProvider = new SidebarTopicsProvider(context.extensionUri);

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
		vscode.window.registerWebviewViewProvider(
			"sidebar-topics",
			sidebarTopicsProvider
		),

		// Commands

		// Create templates and directories
		vscode.commands.registerCommand(
			"r-ide.create-file-from-template",
			createFileFromTemplate
		),
		vscode.commands.registerCommand(
			"r-ide.create-package",
			createRosProject
		),
		vscode.commands.registerCommand(
			"r-ide.create-msg", 
			createMessage
		),
		vscode.commands.registerCommand(
			"r-ide.create-srv",
			createSrv
		),

		// Register files and projects
		vscode.commands.registerCommand(
			"r-ide.register-project",
			registerProject
		),
		vscode.commands.registerCommand(
			"r-ide.register-msg",
			addMsgToProject
		),
		vscode.commands.registerCommand(
			"r-ide.register-srv",
			addSrvToProject
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
	  );
}

// This method is called when your extension is deactivated
export function deactivate() {}
