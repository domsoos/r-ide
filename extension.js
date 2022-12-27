// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
const vscode = require('vscode');

// This function is called when the extension is activated
function activate(context) {
  console.log('Congratulations, your extension "my-extension" is now active!');

  // Create a new ROS node button
  let newNodeButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 100);
  newNodeButton.command = 'extension.createNewNode';
  newNodeButton.text = 'Create new ROS node';
  newNodeButton.show();

  // Create a new ROS msg button
  let newMsgButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 90);
  newMsgButton.command = 'extension.createNewMsg';
  newMsgButton.text = 'Create new ROS msg';
  newMsgButton.show();

  // Create a new ROS srv button
  let newSrvButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 80);
  newSrvButton.command = 'extension.createNewSrv';
  newSrvButton.text = 'Create new ROS srv';
  newSrvButton.show();

  // Create a button for recording ROS bags
  let recordBagsButton = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Right, 70);
  recordBagsButton.command = 'extension.recordBags';
  recordBagsButton.text = 'Record ROS bags';
  recordBagsButton.show();

  // Register commands for each button
  context.subscriptions.push(vscode.commands.registerCommand('extension.createNewNode', () => {
    // Add code here to create a new ROS node
  }));
  context.subscriptions.push(vscode.commands.registerCommand('extension.createNewMsg', () => {
    // Add code here to create a new ROS msg
  }));
  context.subscriptions.push(vscode.commands.registerCommand('extension.createNewSrv', () => {
    // Add code here to create a new ROS srv
  }));
  context.subscriptions.push(vscode.commands.registerCommand('extension.recordBags', () => {
    // Add code here to record ROS bags
  }));
}

exports.activate = activate;
