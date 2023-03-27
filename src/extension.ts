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
	addNewFindPackage, identifyPackages, RosPackageQuickPick, updateExistingPackages
} from './RosPackages/RosPackage';
import { SidebarTopicsProvider } from './SidebarTopicsProvider';
import * as cp from 'child_process';

// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
export function activate(context: vscode.ExtensionContext) {
	dbcontroller.connectToDB();

	const sidebarWizardsProvider = new SidebarWizardsProvider(context.extensionUri);
	const sidebarBagsProvider = new SidebarBagsProvider(context.extensionUri);
	const sidebarVisualsProvider = new SidebarVisualsProvider(context.extensionUri);
	const sidebarTopicsProvider = new SidebarTopicsProvider(context.extensionUri);

	updateExistingPackages();	

	// Get the package in the workspace
	// TODO: We should not have to do this everytime, we can probably save this as a workspace configuration
	if (vscode.workspace.workspaceFolders !== undefined) {
		for (let f of vscode.workspace.workspaceFolders) {
			identifyPackages(f.uri);
		}
	}

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
			"r-ide.create-msg", 
			createMessage
		),
		vscode.commands.registerCommand(
			"r-ide.create-srv",
			createSrv
		),
		vscode.commands.registerCommand(
			"r-ide.update-package-list",
			updateExistingPackages
		),
		vscode.commands.registerCommand(
			"r-ide.add-new-find-package",
			addNewFindPackage
		),
		vscode.commands.registerCommand(
			"r-ide.add-executable",
			() => {
				RosPackageQuickPick(false).then((myPackage) => {
					if (myPackage?.value !== undefined) {
						myPackage.value.addExecutable();
					}
				});
			}
		),
		vscode.commands.registerCommand(
			"r-ide.add-library",
			() => {
				RosPackageQuickPick(false).then((myPackage) => {
					if (myPackage?.value !== undefined) {
						myPackage.value.addLibrary();
					}
				});
			}
		),
		vscode.commands.registerCommand(
			"r-ide.run-catkin-make",
			() => {
				// for (let p of RosPackage.packages.values()) {

				// }
				const terminal = vscode.window.createTerminal();
				terminal.show();
				terminal.sendText('catkin_make');
				terminal.sendText('source ./devel/setup.bash');
			}
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
	stopROSBridge();
}

function stopROSBridge(){
	const child = cp.exec(`ps aux | grep rosbridge_websocket.launch | grep -v grep | awk '{ print $2 }'`);

    child?.stdout?.on('data', (data) => {
		const pid = parseInt(data.toString().trim());
		console.log(`Found ROS Bridge process with PID ${pid}`);
  
		// Send the TERM signal to stop the process
		process.kill(pid, 'SIGTERM');
		console.log('ROS Bridge stopped');
	});

	child.on('exit', (code, signal) => {
		if (code !== 0) {
		  console.error(`ps exited with code ${code}, signal ${signal}`);
		}else{
			console.log("Ros Bridge finder exited gracefully");
		}
	});
}
