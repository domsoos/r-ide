import { expect } from "chai";
import * as vscode from "vscode";
import { RosPackage } from "../../RosPackages/RosPackage";

describe("RosPackage", () => {
    // You need to create a sample directory with a sample package for testing purposes
    const sampleDirectoryUri = vscode.Uri.file("path/to/sample/directory");

    let rosPackage: RosPackage;

    beforeEach(() => {
        rosPackage = new RosPackage(sampleDirectoryUri);
    });

    afterEach(() => {
        rosPackage.msgWatcher.dispose();
        rosPackage.srvWatcher.dispose();
        rosPackage.srcWatcher.dispose();
    });

    it("should add message files correctly", () => {
        const msgFiles = [
            vscode.Uri.file("path/to/msg/file1.msg"),
            vscode.Uri.file("path/to/msg/file2.msg")
        ];

        rosPackage.addMsg(...msgFiles);
        msgFiles.forEach((msgFile) => {
            expect(rosPackage.msg.has(msgFile.fsPath)).to
        });
    });

    describe("generateCMakeLists", () => {
        it("should return a CMakeLists.txt string", () => {
            const rosPackage = new RosPackage(vscode.Uri.file("/test/directory"));
            const cmakeString = rosPackage.generateCMakeLists();
            expect(cmakeString).toMatch(/cmake_minimnum_required\(.+\)/);
        });
   

    });

    describe("generatePackageXml", () => {
        it("should return an empty string", () => {
            const rosPackage = new RosPackage(vscode.Uri.file("/test/directory"));
            const packageXml = rosPackage.generatePackageXml();
            expect(packageXml).toBe("");
        });
    });
    

    describe("createRosPackage", () => {
        it("should create a new ROS package", async () => {
            // Mock the user inputs and file system operations
            // You might need to use a library like jest or sinon for mocking these functions
            const newPackage = await createRosPackage();
            expect(newPackage).toBeInstanceOf(RosPackage);
        });
    });
    
    describe("selectPackage", () => {
        it("should return a selected ROS package", async () => {
            // Mock the user inputs and file system operations
            const selectedPackage = await selectPackage();
            expect(selectedPackage
            });
        });
    });