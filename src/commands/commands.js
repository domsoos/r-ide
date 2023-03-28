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
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.createSrv = exports.createMessage = exports.createFileFromTemplate = void 0;
const vscode = __importStar(require("vscode"));
function createFileFromTemplate(path, options) {
    var _a;
    return __awaiter(this, void 0, void 0, function* () {
        // DEBUG
        // console.log(path);
        // console.log(options);
        const uri = vscode.Uri.file(path);
        vscode.window.showInformationMessage(uri.path);
        yield vscode.workspace.fs.writeFile(uri, Buffer.from(''));
        yield vscode.window.showTextDocument(uri);
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
        }
        (_a = vscode.window.activeTextEditor) === null || _a === void 0 ? void 0 : _a.document.save();
    });
}
exports.createFileFromTemplate = createFileFromTemplate;
function createMessage(path) {
    var _a, _b;
    return __awaiter(this, void 0, void 0, function* () {
        if (!path) {
            path = yield vscode.window.showSaveDialog({
                // eslint-disable-next-line @typescript-eslint/naming-convention
                filters: { "Text Files": ['msg'] },
                // TODO: This default Uri should be smarter
                defaultUri: vscode.Uri.file(((_a = vscode.workspace.workspaceFolders) === null || _a === void 0 ? void 0 : _a.map(folder => folder.uri.path).toString()) + "/src/untitled.msg")
            });
            // Check the user provided a path
            if (!path) {
                return;
            }
        }
        let uri;
        if (!(path instanceof vscode.Uri)) {
            uri = vscode.Uri.file(path);
        }
        else {
            uri = path;
        }
        // Update package.xml
        // TODO: Assumes that msg is placed in ./msg/*.msg, and that the package is at ./package.xml relative to the root of the package
        let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
        updatePackageXml(packageLocation);
        // Create Msg File
        vscode.window.showInformationMessage(uri.path);
        yield vscode.workspace.fs.writeFile(uri, Buffer.from(''));
        yield vscode.window.showTextDocument(uri);
        vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "msg example" });
        (_b = vscode.window.activeTextEditor) === null || _b === void 0 ? void 0 : _b.document.save();
    });
}
exports.createMessage = createMessage;
function createSrv(path) {
    var _a, _b;
    return __awaiter(this, void 0, void 0, function* () {
        if (!path) {
            path = yield vscode.window.showSaveDialog({
                // eslint-disable-next-line @typescript-eslint/naming-convention
                filters: { "Text Files": ['srv'] },
                // TODO: This default Uri should be smarter
                defaultUri: vscode.Uri.file(((_a = vscode.workspace.workspaceFolders) === null || _a === void 0 ? void 0 : _a.map(folder => folder.uri.path).toString()) + "/src/untitled.srv")
            });
            // Check the user provided a path
            if (!path) {
                return;
            }
        }
        let uri;
        if (!(path instanceof vscode.Uri)) {
            uri = vscode.Uri.file(path);
        }
        else {
            uri = path;
        }
        // Update package.xml
        // TODO: Assumes that srv is placed in ./srv/*.srv, and that the package is at ./package.xml relatively
        let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
        updatePackageXml(packageLocation);
        // Create Srv File
        vscode.window.showInformationMessage(uri.path);
        yield vscode.workspace.fs.writeFile(uri, Buffer.from(''));
        yield vscode.window.showTextDocument(uri);
        vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros-srv", name: "Create New ROS Service Definition" });
        (_b = vscode.window.activeTextEditor) === null || _b === void 0 ? void 0 : _b.document.save();
    });
}
exports.createSrv = createSrv;
function updatePackageXml(packagePath) {
    return __awaiter(this, void 0, void 0, function* () {
        let packageLocation;
        if (!(packagePath instanceof vscode.Uri)) {
            packageLocation = vscode.Uri.file(packagePath);
        }
        else {
            packageLocation = packagePath;
        }
        vscode.workspace.openTextDocument(packageLocation).then(document => {
            vscode.window.showTextDocument(document, 1, true).then(editor => {
                const dependTag = /<.*?_depend>.*?<\/.*?_depend>/sg;
                let edit = '';
                // XML document
                let text = document.getText();
                // Assumes some depend tags already exist
                let lastDepend = 0;
                while (dependTag.exec(text) !== null) {
                    lastDepend = dependTag.lastIndex;
                }
                // Check if build exists
                const buildRegEx = /(?<!<!--.*?)<build_depend>message_generation<\/build_depend>/;
                if (!buildRegEx.test(text)) {
                    // Add the text 
                    edit += '\n  <build_depend>message_generation<\/build_depend>';
                }
                // Check if runtime exists
                const runRegEx = /(?<!<!--.*?)<run_depend>message_runtime<\/run_depend>/;
                if (!runRegEx.test(text)) {
                    // Add the text
                    edit += '\n  <run_depend>message_runtime</run_depend>';
                }
                editor.edit(editBuilder => {
                    editBuilder.insert(document.positionAt(lastDepend), edit);
                });
                editor.document.save();
            });
        });
    });
}
//# sourceMappingURL=commands.js.map