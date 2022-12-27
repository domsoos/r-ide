const path = require('path');

const { runTests } = require('@vscode/test-electron');

async function main() {
	try {
		// The folder containing the Extension Manifest package.json
		// Passed to `--extensionDevelopmentPath`
		const extensionDevelopmentPath = path.resolve(__dirname, '../');

		// The path to the extension test script
		// Passed to --extensionTestsPath
		const extensionTestsPath = path.resolve(__dirname, './suite/index');

		// Download VS Code, unzip it and run the integration test
		await runTests({ extensionDevelopmentPath, extensionTestsPath });
	} catch (err) {
		console.error('Failed to run tests');
		process.exit(1);
	}
}

async function testCreateNewROSNode() {
	// Use the extension's functionality to create a new ROS node
	const nodeName = "test_node";
	createNewNode(nodeName);
  
	// Verify that the node has been created and is functional
	assert(checkNodeExists(nodeName), `Node ${nodeName} was not created.`);
	assert(checkNodeIsFunctional(nodeName), `Node ${nodeName} is not functional.`);
  }

main();
