import * as vscode from "vscode";
import { TextEncoder } from "util";

export async function createFileFromTemplate(path: string, options: any) {
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
    }
    vscode.window.activeTextEditor?.document.save();
}

export async function createMessage(path: string) {
    // Create Msg File
    const uri = vscode.Uri.file(path);
    vscode.window.showInformationMessage(uri.path);

    await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "msg example"});

    vscode.window.activeTextEditor?.document.save();

    // Update package.xml
    // TODO: Assumes that msg is placed in ./msg/*.msg, and that the package is at ./package.xml relative to the root of the project
    let packageLocation = vscode.Uri.joinPath(vscode.Uri.file(path), '../../package.xml');
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
            }

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

            editor.document.save();
        });
    });
}

export async function createSrv(path: string) {
    // Create Srv File
    const uri = vscode.Uri.file(path);
    vscode.window.showInformationMessage(uri.path);

    await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "srv example"});

    vscode.window.activeTextEditor?.document.save();

    // Update package.xml
    // TODO: Assumes that srv is placed in ./srv/*.srv, and that the package is at ./package.xml relatively
    let packageLocation = vscode.Uri.joinPath(vscode.Uri.file(path), '../../package.xml');
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
            }

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

            editor.document.save();
        });
    });
    
}