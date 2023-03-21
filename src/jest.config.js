module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'node',
  transform: {
    '^.+\\.tsx?$': 'ts-jest',
  },
  moduleNameMapper: {
    '^vscode$': '<rootDir>/__mocks__/vscode.js',
  },
  modulePaths: [
    "<rootDir>/node_modules",
    "<rootDir>/node_modules/vscode/dist",
  ]
};