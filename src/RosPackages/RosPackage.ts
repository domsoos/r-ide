import * as vscode from "vscode";
import * as cp from "child_process";
import { Ros } from "roslib";
import { relative } from "path";

export class RosPackage {

    public static packages = new Map<string, RosPackage>();

    // Get all exisiting packages installed on system
    public static existingPackages: Set<string>;

    msg: Set<vscode.Uri>;
    srv: Set<vscode.Uri>;
    actions: Set<vscode.Uri>;
    pyFiles: Set<vscode.Uri>;
    cppFiles: Set<vscode.Uri>;

    rootDirectory: vscode.Uri;
    projectName: string;

    // File watchers
    msgWatcher: vscode.FileSystemWatcher;
    srvWatcher: vscode.FileSystemWatcher;
    pyWatcher: vscode.FileSystemWatcher;
    cppWatcher: vscode.FileSystemWatcher;

    constructor(directory: vscode.Uri, name: string) {
        RosPackage.packages.set(directory.fsPath, this);
        this.rootDirectory = directory;
        this.projectName = name;

        // File sets
        this.msg = new Set();
        this.srv = new Set();
        this.actions = new Set();
        this.pyFiles = new Set();
        this.cppFiles = new Set();

        // File watchers
        this.msgWatcher = vscode.workspace.createFileSystemWatcher(`${directory.fsPath}/**/*.msg`);
        this.srvWatcher = vscode.workspace.createFileSystemWatcher(`${directory.fsPath}/**/*.srv`);
        this.pyWatcher = vscode.workspace.createFileSystemWatcher(`${directory.fsPath}/**/*.py`);
        this.cppWatcher = vscode.workspace.createFileSystemWatcher(`${directory.fsPath}/**/*.cpp`);

        // On creates
        this.msgWatcher.onDidCreate((uri) => {
            this.msg.add(uri);
        });

        this.srvWatcher.onDidCreate((uri) => {
            this.srv.add(uri);
        });

        this.pyWatcher.onDidCreate((uri) => {
            this.pyFiles.add(uri);
        });

        this.cppWatcher.onDidCreate((uri) => {
            this.cppFiles.add(uri);
        });

        // On delete
        this.msgWatcher.onDidDelete((uri) => {
            this.msg.delete(uri);
        });

        this.srvWatcher.onDidDelete((uri) => {
            this.srv.delete(uri);
        });

        this.pyWatcher.onDidDelete((uri) => {
            this.pyFiles.delete(uri);
        });

        this.cppWatcher.onDidDelete((uri) => {
            this.cppFiles.delete(uri);
        });


        // Update config
        // TODO: Register config
        const config = vscode.workspace.getConfiguration('r-ide.packages');
        config.update('RosPackage.packages', RosPackage.packages);
    } 

    public updateMsgFiles() {
        // Assumes all python scripts are in scripts folder
        let files: string[] = [];
        this.msg.forEach((uri) => {
            files.push(relative(vscode.Uri.joinPath(this.rootDirectory, "/msg/").fsPath, uri.fsPath));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?add_message_files\s*?\(.*?\)/sgmi;
        const replace = `add_message_files(${["FILES", ...files].join('\n    ')}\n)`;
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }

    public updateSrvFiles() {
        // Assumes all python scripts are in scripts folder
        let files: string[] = [];
        this.srv.forEach((uri) => {
            files.push(relative(vscode.Uri.joinPath(this.rootDirectory, "/srv/").fsPath, uri.fsPath));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?add_service_files\s*?\(.*?\)/sgmi;
        const replace = `add_service_files(${["FILES", ...files].join('\n    ')}\n)`;
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }
    
    public updatePythonFiles() {
        // Assumes all python scripts are in scripts folder
        let files: string[] = [];
        this.pyFiles.forEach((uri) => {
            files.push(relative(vscode.Uri.joinPath(this.rootDirectory, "/scripts/").fsPath, uri.fsPath));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?catkin_install_python\s*?\(.*?\)/sgmi;
        const replace = `catkin_install_python(${["PROGRAMS", ...files].join('\n    ')}\n    DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}\n)`;
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }
}

/**
 * A quick pick object that allows a user to quickly select an existing package
 * @returns 
 */
// eslint-disable-next-line @typescript-eslint/naming-convention
export async function RosPackageQuickPick() {
    let options: any[] = [];

    console.log(RosPackage.packages.size);

    RosPackage.packages.forEach((p, name) => {
        options.push({
            label: p.projectName,
            value: p
        });
    });

    console.log(options);

    return await vscode.window.showQuickPick(
        [...options],
        {
            title: "Select a package",
            canPickMany: false
        }
    );
}

/**
 * Identifies the packages in a root directory
 * @param root The root directory to identify packages
 */
export async function identifyPackages(root: vscode.Uri) {
    console.log(root.fsPath.endsWith("/catkin_ws"));
    if (root.fsPath.endsWith("/catkin_ws")) {
        // catkin_ws is the workspace
        root = vscode.Uri.joinPath(root, '/src/');
        console.log(root.fsPath);
        for (let [folder, type] of await vscode.workspace.fs.readDirectory(root)) {
            console.log(`${folder}, ${type}`);
            // TODO: Assumes all folders are ros packages, this probably isn't true, check for CMakeLists.txt and package.xml
            if (type === 2) {
                new RosPackage(vscode.Uri.joinPath(root, `/${folder}`), folder);
            }
            console.log(RosPackage.packages.size);
        }
    } else {
        // Entire workspace must be one package
        const name = root.fsPath.split('/')[-1];
        RosPackage.packages.set(name, new RosPackage(root, name));
    }

    console.log(RosPackage.packages.entries());
}

/**
 * Updates the RosProject existing packages based on the results of `rospack list-names`
 */
export function updateExistingPackages() {
    // This command might be limited to some later ros distros
    RosPackage.existingPackages = new Set(cp.execSync(`rospack list-names`).toString().split('\n'));
}

export async function addNewFindPackage(myPackage?: vscode.Uri, newPackage?: string[]) {
    // Select a package
    if (!myPackage) {
        let selectedPackage = await vscode.window.showOpenDialog({
            canSelectFolders: true,
            canSelectFiles: false,
            canSelectMany: false
        });

        if (!selectedPackage) {
            return;
        }
        myPackage = selectedPackage[0];
    }

    // Find cmake file
    const cmake = vscode.Uri.joinPath(myPackage, './CMakeLists.txt');

    // Get existing packages
    const regex = /^[^\n#]*find_package\(catkin\s+REQUIRED\s+COMPONENTS\s+(.*?)\s*\)/sgmi;
    let packages = new Set<string>();
    await vscode.workspace.openTextDocument(cmake).then(document => {
        const text = [...document.getText().matchAll(regex)][0][1];

        console.log(text);

        packages = new Set(text.split(/\s+/));
        console.log([...packages.keys()]);
        
    });

    // Select new packages
    if (!newPackage) {
        let options = [];

        // Make sure existing are already picked
        for (let p of RosPackage.existingPackages.keys()) {
            options.push({
                label: p,
                value: p,
                picked: packages.has(p)
            });
        }
        console.log(options);
        let selectedPackage = await vscode.window.showQuickPick(
            [...options], 
            {
                title: "Select an existing ROS Package",
                canPickMany: true,
        });

        if (!selectedPackage) {
            return;
        }

        newPackage = [];
        for (let p of selectedPackage) {
            newPackage.push(p.value);
        }
    }

    const replace = `find_package(catkin REQUIRED ${["COMPONENTS", ...newPackage].join('\n    ')}\n)`;
    replaceTextInDocument(cmake, regex, replace);
}

function replaceTextInDocument(uri: vscode.Uri, regexp: RegExp, replaceText: string) {
    let start: vscode.Position;
    let end: vscode.Position;

    vscode.workspace.openTextDocument(uri).then(document => {
        let match = regexp.exec(document.getText());

        console.log(match?.index);

        if (match?.index) {

            start = document.positionAt(match.index);
            end = document.positionAt(match.index + match[0].length);

            const edit = new vscode.WorkspaceEdit();
            const editRange = new vscode.Range(start, end);
            const makeEdit = new vscode.TextEdit(editRange, replaceText);

            edit.set(uri, [makeEdit]);
            vscode.workspace.applyEdit(edit);

            vscode.window.showInformationMessage(`Updated ${uri.fsPath}`, `Open Document`).then(answer => {
                if (answer === 'Open Document') {
                    vscode.window.showTextDocument(uri, {selection: new vscode.Range(start, start.translate({characterDelta: replaceText.length}))});
                }
            });
        } else {
            throw new Error(`Could not find expression matching ${regexp} in ${uri.fsPath}`);
        }
    });
}
