import { expect } from "chai";
import sinon, { SinonStub, SinonFakeTimers } from 'sinon';
import * as vscode from "vscode";
import { RosPackage, createRosPackage, selectPackage } from "../../RosPackages/RosPackage";

describe("RosPackage", () => {
    // create a sample directory with a sample package for testing purposes
    const sampleDirectoryUri = vscode.Uri.file("/Users/gavinst.clair/Library/Mobile\ Documents/com\~apple\~CloudDocs/ODU/Spring\ 2023/411");

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
            expect(rosPackage.msg.has(msgFile.fsPath)).to.be.true;
        });
    });

    describe("generateCMakeLists", () => {
        it("should return a CMakeLists.txt string", () => {
            const rosPackage = new RosPackage(vscode.Uri.file("/test/directory"));
            const cmakeString = rosPackage.generateCMakeLists();
            expect(cmakeString).to.match(/cmake_minimum_required\(.+\)/);
        });
   

    });

    describe("generatePackageXml", () => {
        it("should return an empty string", () => {
            const rosPackage = new RosPackage(vscode.Uri.file("/test/directory"));
            const packageXml = rosPackage.generatePackageXml();
            expect(packageXml).to.equal("");
        });
    });
    

    // Create a stub for vscode.window.showInputBox()
    const showInputBoxStub: SinonStub = sinon.stub(vscode.window, 'showInputBox');
    showInputBoxStub.resolves('my_package');
    
    // Create a stub for vscode.window.showInformationMessage()
    const showInformationMessageStub: SinonStub = sinon.stub(vscode.window, 'showInformationMessage');
    showInformationMessageStub.resolves('Yes');
    
    // Create a stub for vscode.window.showOpenDialog()
    const showOpenDialogStub: SinonStub = sinon.stub(vscode.window, 'showOpenDialog');
    showOpenDialogStub.resolves([vscode.Uri.file('/home/user/ros_ws')]);
    
    // Create a fake timer for the setTimeout function used in the showInformationMessage callback
    const clock: SinonFakeTimers = sinon.useFakeTimers();
    
    describe("createRosPackage", () => {
        afterEach(() => {
            sinon.restore();
            clock.reset();
        });
    
        it("should create a new ROS package", async () => {
            const newPackage = await createRosPackage();
    
            expect(showInputBoxStub.calledOnce).to.be.true;
            expect(showInformationMessageStub.calledOnce).to.be.true;
            expect(showOpenDialogStub.calledOnce).to.be.true;
            expect(newPackage).to.be.instanceOf(RosPackage);
        });
    });
    
    describe("selectPackage", () => {
        it("should return a selected ROS package", async () => {
            // Mock the user inputs and file system operations
            const selectedPackage = await selectPackage();
            expect(selectedPackage).to.be.instanceOf(RosPackage);
        });
    });

});