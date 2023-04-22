// import { expect } from "chai";
// import { RosPackage } from "../../RosPackages/RosPackage";
// import * as vscode from "vscode";

// describe("RosPackage", () => {
//   let rosPackage: RosPackage;
//   const testUri = vscode.Uri.file("/test/directory");
//   const testProjectName = "test_project";

//   beforeEach(() => {
//     rosPackage = new RosPackage(testUri, testProjectName);
//   });

//   afterEach(() => {
//     // Clean up the created instance to avoid side effects
//     RosPackage.packages.delete(testUri.fsPath);
//   });

//   describe("constructor", () => {
//     it("should initialize an instance with the given parameters", () => {
//       expect(rosPackage.rootDirectory).to.deep.equal(testUri);
//       expect(rosPackage.projectName).to.equal(testProjectName);
//     });

//     it("should add the instance to the packages map", () => {
//       expect(RosPackage.packages.get(testUri.fsPath)).to.equal(rosPackage);
//     });

//     it("should initialize empty Sets for msg, srv, actions, pyFiles, and cppFiles", () => {
//       expect(rosPackage.msg).to.be.instanceOf(Set);
//       expect(rosPackage.msg.size).to.equal(0);
//       expect(rosPackage.srv).to.be.instanceOf(Set);
//       expect(rosPackage.srv.size).to.equal(0);
//       expect(rosPackage.actions).to.be.instanceOf(Set);
//       expect(rosPackage.actions.size).to.equal(0);
//       expect(rosPackage.pyFiles).to.be.instanceOf(Set);
//       expect(rosPackage.pyFiles.size).to.equal(0);
//       expect(rosPackage.cppFiles).to.be.instanceOf(Set);
//       expect(rosPackage.cppFiles.size).to.equal(0);
//     });
//   });

//   // more tests
// });
