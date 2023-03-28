import * as vscode from "vscode";
import * as cp from "child_process";
import { relative } from "path";

export class RosPackage {

    public static packages = new Map<string, RosPackage>();

    // Get all exisiting packages installed on system
    public static existingPackages: Set<string>;

    msg: Set<string>;
    srv: Set<string>;
    actions: Set<string>;
    pyFiles: Set<string>;
    cppFiles: Set<string>;

    rootDirectory: vscode.Uri;
    projectName: string;

    // File watchers
    fsWatcher: vscode.FileSystemWatcher;

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
        this.fsWatcher = vscode.workspace.createFileSystemWatcher(`${directory.fsPath}/**`);

        // On creates
        this.fsWatcher.onDidCreate((uri) => {
            if (uri.fsPath.endsWith('.msg')) {
                this.msg.add(uri.fsPath);
                this.updateMsgFiles();
            } else if (uri.fsPath.endsWith('.srv')) {
                this.srv.add(uri.fsPath);
                this.updateSrvFiles();
            } else if (uri.fsPath.endsWith('.cpp')) {
                this.cppFiles.add(uri.fsPath);
            } else if (uri.fsPath.endsWith('.py')) {
                this.pyFiles.add(uri.fsPath);
                this.updatePythonFiles();
            } else {
                console.log(uri);
            }
        });

        this.fsWatcher.onDidDelete((uri) => {
            if (uri.fsPath.endsWith('.msg')) {
                this.msg.delete(uri.fsPath);
                this.updateMsgFiles();
            } else if (uri.fsPath.endsWith('.srv')) {
                this.srv.delete(uri.fsPath);
                this.updateSrvFiles();
            } else if (uri.fsPath.endsWith('.cpp')) {
                this.cppFiles.delete(uri.fsPath);
            } else if (uri.fsPath.endsWith('.py')) {
                this.pyFiles.delete(uri.fsPath);
                this.updatePythonFiles();
            } else {
                console.log(uri);
            }
        });

        const relativePattern = new vscode.RelativePattern(vscode.workspace.getWorkspaceFolder(this.rootDirectory)!, `src/${this.projectName}/**/*.{msg,srv,py,cpp}`);
        vscode.workspace.findFiles(relativePattern).then((files) => {
            for (let uri of files) {
                // console.log(uri);
                if (uri.fsPath.endsWith('.msg')) {
                    this.msg.add(uri.fsPath);
                } else if (uri.fsPath.endsWith('.srv')) {
                    this.srv.add(uri.fsPath);
                } else if (uri.fsPath.endsWith('.py')) {
                    this.pyFiles.add(uri.fsPath);
                } else if (uri.fsPath.endsWith('.cpp')) {
                    this.cppFiles.add(uri.fsPath);
                }
            }

            console.log(this.projectName);
            console.log(this.msg.entries());
            console.log(this.srv.entries());
            console.log(this.pyFiles.entries());
            console.log(this.cppFiles.entries());
        });
    } 

    public updateMsgFiles() {
        let files: string[] = [];
        this.msg.forEach((uri) => {
            files.push(relative(this.rootDirectory.fsPath, uri));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?add_message_files\s*?\(.*?\)/sgmi;

        let replace;
        if (this.msg.size > 0) {
            replace = `add_message_files(\n    ${["FILES", ...files].join('\n    ')}\n)`;
        } else {
            replace = "# add_message_files()";
        }
        
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }

    public updateSrvFiles() {
        let files: string[] = [];
        this.srv.forEach((uri) => {
            files.push(relative(this.rootDirectory.fsPath, uri));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?add_service_files\s*?\(.*?\)/sgmi;

        let replace;
        if (this.srv.size > 0) {
            replace = `add_service_files(\n    ${["FILES", ...files].join('\n    ')}\n)`;
        } else {
            replace = "# add_service_files()";
        }
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }
    
    public updatePythonFiles() {
        let files: string[] = [];
        this.pyFiles.forEach((uri) => {
            files.push(relative(this.rootDirectory.fsPath, uri));
        });

        console.log(files);

        const regex = /^\s*?#?\s*?catkin_install_python\s*?\(.*?\)/sgmi;
        let replace;
        if (this.pyFiles.size > 0) {
            replace = `catkin_install_python(\n    ${["PROGRAMS", ...files].join('\n    ')}\n    DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}\n)`;
        } else {
            replace = "# catkin_install_python()";
        }
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, '/CMakeLists.txt'), regex, replace);
    }

    public checkCppFiles() {
        let files: string[] = [];
        this.cppFiles.forEach((uri) => {
            files.push(relative(this.rootDirectory.fsPath, uri));
        });

        console.log(files);

        const regex = /^(\s*?#?\s*?add_(executable|library)\((.*?)\)\s+target_link_libraries\((.*?)\))*/sgmi;
        vscode.workspace.openTextDocument(vscode.Uri.joinPath(this.rootDirectory, 'CMakeLists.txt')).then((document) => {
            const text = document.getText();

            let match = text.matchAll(regex);
            console.log(match);

        });
    }

    public async addExecutable() {
        let prefill = this.projectName + "_myExecutable";
        const name = await vscode.window.showInputBox({
            prompt: "Enter a name for the executable",
            title: "Executable name",
            value: prefill,
            valueSelection: [this.projectName.length + 1, prefill.length]
        });

        if (name === undefined) {
            return;
        }

        const options = [];
        for (let cpp of this.cppFiles) {
            options.push(relative(this.rootDirectory.fsPath, cpp));
        }

        const selectedCppFiles = await vscode.window.showQuickPick(
            options,
            {
                canPickMany: true,
                title: "Select the source files"
            }
        );

        if (selectedCppFiles === undefined) {
            return;
        }

        const regex = /# add_executable\(.*?\)/sgmi;
        const replace = `# ${name}\nadd_executable(${[name, ...selectedCppFiles].join('\n    ')}\n)\ntarget_link_libraries(${name} \${catkin_LIBRARIES})\n`;
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, "CMakeLists.txt"), regex, replace, true);
    }

    public async addLibrary() {
        let prefill = this.projectName + "_myLibrary";
        const name = await vscode.window.showInputBox({
            prompt: "Enter a name for the library",
            title: "Library name",
            value: prefill,
            valueSelection: [this.projectName.length + 1, prefill.length]
        });

        if (name === undefined) {
            return;
        }

        const options = [];
        for (let cpp of this.cppFiles) {
            options.push(relative(this.rootDirectory.fsPath, cpp));
        }

        const selectedCppFiles = await vscode.window.showQuickPick(
            options,
            {
                canPickMany: true,
                title: "Select the source files"
            }
        );

        if (selectedCppFiles === undefined) {
            return;
        }

        const regex = /^# add_library\(.*?\)/sgmi;
        const replace = `# ${name}\nadd_library(${[name, ...selectedCppFiles].join('\n    ')}\n)\ntarget_link_libraries(${name} \${catkin_LIBRARIES})\n`;
        replaceTextInDocument(vscode.Uri.joinPath(this.rootDirectory, "CMakeLists.txt"), regex, replace, true);
    }
}

/**
 * A quick pick object that allows a user to quickly select an existing package
 * @param newPackage A boolean on whether to include a new package option
 * @returns The user selected option
 */
// eslint-disable-next-line @typescript-eslint/naming-convention
export async function RosPackageQuickPick(newPackage: boolean = true) {
    let options: {label: string, value: RosPackage | undefined}[] = [];

    RosPackage.packages.forEach((p, name) => {
        options.push({
            label: p.projectName,
            value: p
        });
    });

    if (newPackage) {
        options.push({label: "New RosPackage", value: undefined});
    }

    if (options.length === 0) {
        vscode.window.showErrorMessage("There are no packages available");
    }

    let selected = await vscode.window.showQuickPick(
        [...options],
        {
            title: "Select a package",
            canPickMany: false
        }
    );

    if (selected?.value === undefined && selected?.label === "New RosPackage") {
        await vscode.commands.executeCommand("ros.createCatkinPackage");
        return;
    }

    return selected;
}

/**
 * Identifies the packages in a root directory
 * @param root The root directory to identify packages
 */
export async function identifyPackages(root: vscode.Uri) {
    RosPackage.packages.clear();
    console.log(root.fsPath.endsWith("/catkin_ws"));
    let myPackages = [];
    if (root.fsPath.endsWith("/catkin_ws")) {
        // catkin_ws is the workspace
        root = vscode.Uri.joinPath(root, '/src/');

        for (let [folder, type] of await vscode.workspace.fs.readDirectory(root)) {

            // If its a folder, check that it has a CMakeLists.txt and package.xml
            if (type === 2) {
                let hasCmake = false, hasPackageXml = false;
                let uri = vscode.Uri.joinPath(root, folder);

                for (let [file, _] of await vscode.workspace.fs.readDirectory(uri)) {
                    if (file === "CMakeLists.txt") {
                        hasCmake = true;
                    } else if (file === "package.xml") {
                        hasPackageXml = true;
                    }
                }
                
                if (hasCmake && hasPackageXml) {
                    new RosPackage(uri, folder);
                    myPackages.push(uri);
                }
            }
        }
    } else {
        // Entire workspace must be one package
        const name = root.fsPath.split('/')[-1];
        RosPackage.packages.set(name, new RosPackage(root, name));
    }
}

/**
 * Updates the RosProject existing packages based on the results of `rospack list-names`
 */
export function updateExistingPackages() {
    // This command might be limited to some later ros distros
    RosPackage.existingPackages = new Set(cp.execSync(`rospack list-names`).toString().split('\n'));
}

/**
 * Updates find package command in CMakeLists.txt
 * @param myPackage Select a package
 * @param newPackage 
 * @returns 
 */
export async function addNewFindPackage(myPackage?: vscode.Uri, newPackage?: string[]) {
    // Select a package
    if (!myPackage) {
        await RosPackageQuickPick().then((result) => {
            console.log(result);
            myPackage = result?.value?.rootDirectory;
        });

        console.log(myPackage);

        if (!myPackage) {
            return;
        }
    }

    console.log(myPackage);

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

/**
 * Replaces some text in a document and displays a link to the user
 * @param uri The document to change
 * @param regexp The expression to match and replace
 * @param replaceText The text to be placed
 */
function replaceTextInDocument(uri: vscode.Uri, regexp: RegExp, replaceText: string, addBelow: boolean = false) {
    let start: vscode.Position;
    let end: vscode.Position;

    vscode.workspace.openTextDocument(uri).then(document => {
        let match = regexp.exec(document.getText());

        console.log(match?.index);

        if (match?.index) {

            start = document.positionAt(addBelow ? match.index + match[0].length : match.index);
            end = document.positionAt(match.index + match[0].length);

            const edit = new vscode.WorkspaceEdit();
            const editRange = new vscode.Range(start, end);
            const makeEdit = new vscode.TextEdit(editRange, (addBelow ? "\n" : "") + replaceText);

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
