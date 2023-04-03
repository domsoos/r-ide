import { runTests } from 'vscode-test';

async function main(): Promise<void> {
  try {
    // The folder containing the Extension Manifest package.json
    // Passed to `--extensionDevelopmentPath`
    const extensionDevelopmentPath = __dirname;

    // The path to test runner
    // Passed to --extensionTestsPath
    const extensionTestsPath = __dirname + '/out/test/suite/index';

    // The launch configuration
    const launchArgs = ['--disable-extensions'];

    // Run the tests
    await runTests({ extensionDevelopmentPath, extensionTestsPath, launchArgs });
  } catch (err) {
    console.error('Failed to run tests');
    process.exit(1);
  }
}

main();
