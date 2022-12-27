const vscode = require('vscode');

// Function for creating a new ROS node
function createNewNode() {
    // Prompt the user for the name of the new node
    vscode.window.showInputBox({
      prompt: 'Enter the name of the new ROS node:'
    }).then(nodeName => {
      if (nodeName) {
        // Generate the code for the new node
        let nodeCode = `#include <ros/ros.h>
  
    int main(int argc, char** argv) {
      ros::init(argc, argv, "${nodeName}");
      ros::NodeHandle nh;
  
     // Add more code here
  
      ros::spin();
      return 0;
    }`;
    }}};