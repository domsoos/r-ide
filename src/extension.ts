// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
import { TextEncoder } from 'util';
import * as vscode from 'vscode';
import { SidebarBagsProvider } from './SidebarBagsProvider';
import { SidebarTopicsProvider } from './SidebarTopicsProvider';
import { SidebarVisualsProvider } from './SidebarVisualsProvider';
import { SidebarWizardsProvider } from './SidebarWizardsProvider';

// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
export function activate(context: vscode.ExtensionContext) {
	
	const sidebarWizardsProvider = new SidebarWizardsProvider(context.extensionUri);
	const sidebarBagsProvider = new SidebarBagsProvider(context.extensionUri);
	const sidebarVisualsProvider = new SidebarVisualsProvider(context.extensionUri);
	const sidebarTopicsProvider = new SidebarTopicsProvider(context.extensionUri);

	context.subscriptions.push(
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
		vscode.commands.registerCommand(
			"r-ide.create-file-from-template",
			async (path: string, options: any) => {
				const uri = vscode.Uri.file(path);
				vscode.window.showInformationMessage(uri.path);
				await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
				//let document = await vscode.workspace.openTextDocument(vscode.Uri.file(uri.path).with({ scheme: 'untitled'}));
				await vscode.window.showTextDocument(uri);

				switch (options.language) {
					case ('py'): {
						if (options.isPublisher) {
							vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "python", name: "def talker" });
						}
						if (options.isSubscriber) {
							vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "python", name: "def listener" });
						}
						break;
					}
					case ('cpp'): {

						break;
					}

				}
				vscode.window.activeTextEditor?.document.save();
				/*
				document.save().then( (result) => {
					vscode.window.showTextDocument(uri);
				});
				*/
				
			}
		)
		// vscode.commands.registerCommand(
		// 	"r-ide.update-cmake-lists-txt.python", 
		// 	() => {
		// 		const editor = vscode.window.activeTextEditor;
		// 		if (!editor) {
		// 			return;
		// 		}

		// 		const document = editor.document;
		// 		console.log(document.languageId);
		// 	}
		// )
	  );
}

// This method is called when your extension is deactivated
export function deactivate() {}
