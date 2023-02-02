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

				// DEBUG
				// console.log(path);
				// console.log(options);

				const uri = vscode.Uri.file(path);
				vscode.window.showInformationMessage(uri.path);

				await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
				await vscode.window.showTextDocument(uri);

				switch (options.language.id) {
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
					// TODO: Find proper value for langID for ros message and service files
					case ("srv"): {
						vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "srv example"});
						break;
					}	
					case("msg"): {
						vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "msg example"});
						vscode.commands.executeCommand("r-ide.add-msg", uri.path);
						break;
					}
				}
				vscode.window.activeTextEditor?.document.save();
			}
		),
		vscode.commands.registerCommand(
			"r-ide.add-msg", 
			(msgPath: string) => {
				// TODO: Assumes that msg is placed in ./msg/*.msg, and that the package is at ./package.xml relative to the root of the project
				let packageLocation = vscode.Uri.joinPath(vscode.Uri.file(msgPath), '../../package.xml');
				vscode.workspace.openTextDocument(packageLocation).then(document => {
					vscode.window.showTextDocument(document, 1, true).then(editor => {
						// Regex used
						const dependTag = /<.*?_depend>.*?<\/.*?_depend>/sg;
						let edit = '';

						// XML document
						let text = document.getText();

						// Assumes some depend tags already exist
						let lastDepend = 0;
						while (dependTag.exec(text) !== null) {
							lastDepend = dependTag.lastIndex;
							console.log(lastDepend);
						}

						console.log(text.substring(0, lastDepend));

						// Check if build exists
						const buildRegEx: RegExp = /(?<!<!--.*?)<build_depend>message_generation<\/build_depend>/;
						if (!buildRegEx.test(text)) {
							// Add the text 
							edit += '\n  <build_depend>message_generation<\/build_depend>';
						}

						// Check if runtime exists
						const runRegEx: RegExp = /(?<!<!--.*?)<run_depend>message_runtime<\/run_depend>/;
						if (!runRegEx.test(text)) {
							// Add the text
							edit += '\n  <run_depend>message_runtime</run_depend>';
						}

						editor.edit(editBuilder => {
							editBuilder.insert(document.positionAt(lastDepend), edit);
						});
					});
				});
			}
		)
	  );
}

// This method is called when your extension is deactivated
export function deactivate() {}
