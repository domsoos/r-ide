/* eslint-disable @typescript-eslint/naming-convention */
module.exports = {
  preset: 'ts-jest',
  testEnvironment: 'node',
  rootDir: '.',
  testMatch: ['**/src/test/**/*.test.ts'],
  globals: {
    'ts-jest': {
      tsconfig: 'tsconfig.test.json',
    },
  },
  moduleNameMapper: {
    '^vscode$': '<rootDir>/src/test/vscode.mock.ts',
  },
  transform: {
    '^.+\\.ts?$': ['ts-jest', { tsConfig: './tsconfig.json' }],
  },
  transformIgnorePatterns: ['/node_modules/'],
  moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json', 'node'],
};
