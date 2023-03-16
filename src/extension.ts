// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
import * as vscode from 'vscode';
import { SidebarBagsProvider } from './SidebarBagsProvider';
import { SidebarVisualsProvider } from './SidebarVisualsProvider';
import { SidebarWizardsProvider } from './SidebarWizardsProvider';
import * as dbcontroller from './database/dbcontroller';
import { TopicMonitorProvider } from './TopicMonitorProvider';
import { 
	createFileFromTemplate,
	createMessage,
	createSrv
} from './commands/commands';
import { 
	addMsgToPackage, addNewFindPackage, addSrvToPackage, createRosPackage, loadPackages, registerPackage, RosPackage, updateExistingPackages
} from './RosPackages/RosPackage';
import { SidebarTopicsProvider } from './SidebarTopicsProvider';

// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
export function activate(context: vscode.ExtensionContext) {
	dbcontroller.connectToDB();

	const sidebarWizardsProvider = new SidebarWizardsProvider(context.extensionUri);
	const sidebarBagsProvider = new SidebarBagsProvider(context.extensionUri);
	const sidebarVisualsProvider = new SidebarVisualsProvider(context.extensionUri);
	const sidebarTopicsProvider = new SidebarTopicsProvider(context.extensionUri);

	loadPackages();
	updateExistingPackages();	

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
		vscode.commands.registerCommand(
			"r-ide.update-package-list",
			updateExistingPackages
		),
		vscode.commands.registerCommand(
			"r-ide.add-new-find-package",
			addNewFindPackage
		),
		vscode.commands.registerCommand("r-ide.open-topic-monitor", () => {
			TopicMonitorProvider.createOrShow(context.extensionUri);
		}),
		vscode.commands.registerCommand("r-ide.no-ros-connection", ()=>{
			vscode.window.showErrorMessage(
				'ROS Bridge not detected. Would you like to try to start ROS Bridge?',
				'Start ROS Bridge', 'Close'
			  ).then((res) =>{
				if (res === 'Start ROS Bridge') {

					let term = vscode.window.terminals.find(item => item.name === 'ROS Bridge');

					if(term){
						term.sendText('\x03');
						term.dispose();
						setTimeout(()=>{
							const terminal = vscode.window.createTerminal({
								name: 'ROS Bridge',
								shellPath: '/bin/bash',
								shellArgs: ['-c', 'roslaunch rosbridge_server rosbridge_websocket.launch']
							});
							terminal.show();
						},1500);
					}
					else{
						const terminal = vscode.window.createTerminal({
							name: 'ROS Bridge',
							shellPath: '/bin/bash',
							shellArgs: ['-c', 'roslaunch rosbridge_server rosbridge_websocket.launch']
						});
						terminal.show();
					}
				}
			  });
		})
		
	  );
}

// This method is called when your extension is deactivated
export function deactivate() {
	dbcontroller.closeConnection();
}
