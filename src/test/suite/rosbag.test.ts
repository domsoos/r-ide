// import { Rosbag } from '../../RosBag/rosbag';
// import { describe, it, beforeEach, afterEach } from "mocha";
// import { expect } from "chai";
// import { Webview, EventEmitter } from 'vscode';

// describe('Rosbag class', () => {
//     let rosbag: Rosbag;

//     //path of ros bag for testing 
//     const bagPath = 'test bag path';
  
//     // Create a fake Webview object
//     const fakeWebView: Webview = {
//       asWebviewUri: () => { throw new Error("Not implemented"); },
//       postMessage: () => { throw new Error("Not implemented"); },
//       onDidReceiveMessage: new EventEmitter<any>().event,
//       cspSource: "",
//       html: "",
//       options: { enableScripts: false },
//       onDidChange: new EventEmitter<any>().event,
//     };
  
//     before(() => {
//       // instance of the Rosbag class with a sample bag path and a fake WebView object
//       rosbag = new Rosbag(bagPath, fakeWebView);
//     });
  
//     afterEach(() => {
//       // Clean up after each test
//     });
  
//     it('should initialize Rosbag with correct properties', () => {
//       expect(rosbag).to.be.instanceOf(Rosbag);
//       expect(rosbag.messages).to.deep.equal([]);
//       expect(rosbag.publishers).to.be.instanceOf(Map);
//       expect(rosbag.view).to.equal(fakeWebView);
//       expect(rosbag.isPaused).to.be.true;
//       expect(rosbag.pointer).to.equal(0);
//     });
  


//     // Add more tests
//   });