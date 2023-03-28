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
exports.deactivate = exports.activate = void 0;
// The module 'vscode' contains the VS Code extensibility API
// Import the module and reference it with the alias vscode in your code below
const vscode = __importStar(require("vscode"));
const SidebarBagsProvider_1 = require("./SidebarBagsProvider");
const SidebarVisualsProvider_1 = require("./SidebarVisualsProvider");
const SidebarWizardsProvider_1 = require("./SidebarWizardsProvider");
const dbcontroller = __importStar(require("./database/dbcontroller"));
const TopicMonitorProvider_1 = require("./TopicMonitorProvider");
const commands_1 = require("./commands/commands");
const RosPackage_1 = require("./RosPackages/RosPackage");
const SidebarTopicsProvider_1 = require("./SidebarTopicsProvider");
// This method is called when your extension is activated
// Your extension is activated the very first time the command is executed
function activate(context) {
    dbcontroller.connectToDB();
    const sidebarWizardsProvider = new SidebarWizardsProvider_1.SidebarWizardsProvider(context.extensionUri);
    const sidebarBagsProvider = new SidebarBagsProvider_1.SidebarBagsProvider(context.extensionUri);
    const sidebarVisualsProvider = new SidebarVisualsProvider_1.SidebarVisualsProvider(context.extensionUri);
    const sidebarTopicsProvider = new SidebarTopicsProvider_1.SidebarTopicsProvider(context.extensionUri);
    (0, RosPackage_1.loadPackages)();
    (0, RosPackage_1.updateExistingPackages)();
    console.log(RosPackage_1.RosPackage.existingPackages);
    context.subscriptions.push(
    // Webviews
    vscode.window.registerWebviewViewProvider("sidebar-wizards", sidebarWizardsProvider), vscode.window.registerWebviewViewProvider("sidebar-bags", sidebarBagsProvider), vscode.window.registerWebviewViewProvider("sidebar-visuals", sidebarVisualsProvider), vscode.window.registerWebviewViewProvider("sidebar-topics", sidebarTopicsProvider), 
    // Commands
    // Create templates and directories
    vscode.commands.registerCommand("r-ide.create-file-from-template", commands_1.createFileFromTemplate), vscode.commands.registerCommand("r-ide.create-package", RosPackage_1.createRosPackage), vscode.commands.registerCommand("r-ide.create-msg", commands_1.createMessage), vscode.commands.registerCommand("r-ide.create-srv", commands_1.createSrv), 
    // Register files and packages
    vscode.commands.registerCommand("r-ide.register-package", RosPackage_1.registerPackage), vscode.commands.registerCommand("r-ide.register-msg", RosPackage_1.addMsgToPackage), vscode.commands.registerCommand("r-ide.register-srv", RosPackage_1.addSrvToPackage), vscode.commands.registerCommand("r-ide.update-package-list", RosPackage_1.updateExistingPackages), vscode.commands.registerCommand("r-ide.open-topic-monitor", () => {
        TopicMonitorProvider_1.TopicMonitorProvider.createOrShow(context.extensionUri);
    }));
}
exports.activate = activate;
// This method is called when your extension is deactivated
function deactivate() {
    dbcontroller.closeConnection();
}
exports.deactivate = deactivate;
//# sourceMappingURL=extension.js.map