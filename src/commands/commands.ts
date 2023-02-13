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
    // TODO: Assumes that msg is placed in ./msg/*.msg, and that the package is at ./package.xml relative to the root of the project
    let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
    updatePackageXml(packageLocation);

    // Update CMakeLists.txt
    let cmakeLocation = vscode.Uri.joinPath(uri, '../../CMakeLists.txt');
    updateCMakeLists(cmakeLocation);

    // Create Msg File
    
    vscode.window.showInformationMessage(uri.path);
    await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "msg example"});

    vscode.window.activeTextEditor?.document.save();
    
}

export async function createSrv(path?: string | vscode.Uri) {
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
    // TODO: Assumes that srv is placed in ./srv/*.srv, and that the package is at ./package.xml relatively
    let packageLocation = vscode.Uri.joinPath(uri, '../../package.xml');
    updatePackageXml(packageLocation);

    let cmakeLocation = vscode.Uri.joinPath(uri, '../../CMakeLists.txt');
    updateCMakeLists(cmakeLocation);

    // Create Srv File
    vscode.window.showInformationMessage(uri.path);

    await vscode.workspace.fs.writeFile(uri, new TextEncoder().encode(''));
    await vscode.window.showTextDocument(uri);

    vscode.commands.executeCommand("editor.action.insertSnippet", { langId: "ros.msg", name: "srv example"});

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

async function updateCMakeLists(cmakePath: string | vscode.Uri) {
    let cmakeLocation;
    if (!(cmakePath instanceof vscode.Uri)) {
        cmakeLocation = vscode.Uri.file(cmakePath);
    } else {
        cmakeLocation = cmakePath;
    }

    vscode.workspace.openTextDocument(cmakeLocation).then(document => {
        vscode.window.showTextDocument(document, 1, true).then(editor => {
            let text = document.getText();
            let edits = [];
            
            // find_package function
            let findPackageRegEx = /find_package\(catkin REQUIRED COMPONENTS\n(?:[^)]*?\n)*\)/;
            let findPackageFunc = findPackageRegEx.exec(text);
            console.log(findPackageFunc);
            if (!findPackageFunc) {
                // Build function
            } else if (!/message_generation/.test(findPackageFunc[0])) {
                // Add value
                edits.push({
                    'location': findPackageRegEx.lastIndex,
                    'text': '\n  message_generation'
                });
            } // else already exists
        
            editor.document.save();
        });
    });
    
}