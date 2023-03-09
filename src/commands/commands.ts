import * as vscode from "vscode";
import * as path from 'path';

export async function createFileFromTemplate(path: string, options: any) {
    // DEBUG
    // console.log(path);
    // console.log(options);

    const uri = vscode.Uri.file(path);
    vscode.window.showInformationMessage(uri.path);

    await vscode.workspace.fs.writeFile(uri, Buffer.from(''));
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
    }
    vscode.window.activeTextEditor?.document.save();
}

export async function createMessage(path?: string | vscode.Uri) {
    if (!path) {
        path = await vscode.window.showSaveDialog({
            // eslint-disable-next-line @typescript-eslint/naming-convention
            filters: {"Text Files": ['msg']},
            // TODO: This default Uri should be smarter
            defaultUri: vscode.Uri.file(vscode.workspace.workspaceFolders?.map(folder => folder.uri.path).toString() + "/src/untitled.msg")
        });

        // Check the user provided a path
        if (!path) {
            return;
        }
    }

    let uri;

    if (!(path instanceof vscode.Uri)) {
        uri = vscode.Uri.file(path);
    } else {
        uri = path;
    }
    
    

    // Update package.xml
    // TODO: Assumes that msg is placed in ./msg/*.msg, and that the package is at ./package.xml relative to the root of the package
    let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
    updatePackageXml(packageLocation);

    // Create Msg File
    
    vscode.window.showInformationMessage(uri.path);
    await vscode.workspace.fs.writeFile(uri, Buffer.from(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "msg example"});

    vscode.window.activeTextEditor?.document.save();
    
}

export async function createSrv(path?: string | vscode.Uri) {
    if (!path) {
        path = await vscode.window.showSaveDialog({
            // eslint-disable-next-line @typescript-eslint/naming-convention
            filters: {"Text Files": ['srv']},
            // TODO: This default Uri should be smarter
            defaultUri: vscode.Uri.file(vscode.workspace.workspaceFolders?.map(folder => folder.uri.path).toString() + "/src/untitled.srv")
        });

        // Check the user provided a path
        if (!path) {
            return;
        }
    }

    let uri;

    if (!(path instanceof vscode.Uri)) {
        uri = vscode.Uri.file(path);
    } else {
        uri = path;
    }

    // Update package.xml
    // TODO: Assumes that srv is placed in ./srv/*.srv, and that the package is at ./package.xml relatively
    let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
    updatePackageXml(packageLocation);

    // Create Srv File
    vscode.window.showInformationMessage(uri.path);

    await vscode.workspace.fs.writeFile(uri, Buffer.from(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros-srv", name: "Create New ROS Service Definition"});


    vscode.window.activeTextEditor?.document.save();
    
}

async function updatePackageXml(packagePath: string | vscode.Uri) {

    let packageLocation;
    if (!(packagePath instanceof vscode.Uri)) {
        packageLocation = vscode.Uri.file(packagePath);
    } else {
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
