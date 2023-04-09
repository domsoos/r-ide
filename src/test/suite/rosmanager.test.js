const ROSLIB = require('roslib');
const sinon = require('sinon');
const ROS = require('../ROSManager/rosmanager');

describe('ROS', () => {
  let rosAPIStub;
  let rosInstance;

  beforeEach(() => {
    rosAPIStub = sinon.stub(ROSLIB, 'Ros');
    rosInstance = new ROS();
  });

  afterEach(() => {
    rosAPIStub.restore();
  });

  describe('#constructor()', () => {
    it('should create a ROS object with default URL', () => {
      expect(rosAPIStub.calledOnce).toBeTruthy();
      expect(rosAPIStub.calledWith({ url: 'ws://localhost:9090' })).toBeTruthy();
    });

    it('should resolve immediately if ROS is already connected', async () => {
      ROS.rosAPI = { isConnected: true };
      const instance = new ROS();
      expect(await instance).toBeUndefined();
    });

    it('should resolve on successful connection', async () => {
      ROS.rosAPI = { isConnected: false };
      const promise = new ROS();
      ROS.rosAPI.onConnection();
      await expect(promise).resolves.toBeUndefined();
    });

    it('should reject on connection error', async () => {
      ROS.rosAPI = { isConnected: false };
      const promise = new ROS();
      ROS.rosAPI.onError(new Error('Failed to connect to ROS'));
      await expect(promise).rejects.toThrowError('Failed to connect to ROS');
    });
  });

  describe('#getROSApi()', () => {
    it('should return the ROS API object', () => {
      ROS.rosAPI = { isConnected: true };
      const api = ROS.getROSApi();
      expect(api).toBe(ROS.rosAPI);
    });
  });

  describe('#getROSLib()', () => {
    it('should return the ROSLIB object', () => {
      const lib = ROS.getROSLib();
      expect(lib).toBe(ROSLIB);
    });
  });

  describe('#reconnect()', () => {
    it('should close the current connection and create a new one', async () => {
      ROS.rosAPI = { close: sinon.spy() };
      await ROS.reconnect();
      expect(rosAPIStub.calledOnce).toBeTruthy();
      expect(ROS.rosAPI.close.calledOnce).toBeTruthy();
    });

    it('should resolve on successful reconnection', async () => {
      ROS.rosAPI = { close: sinon.spy(), isConnected: false };
      const promise = ROS.reconnect();
      ROS.rosAPI.onConnection();
      await expect(promise).resolves.toBeUndefined();
    });

    it('should reject on reconnection error', async () => {
      ROS.rosAPI = { close: sinon.spy(), isConnected: false };
      const promise = ROS.reconnect();
      ROS.rosAPI.onError(new Error('Failed to connect to ROS'));
      await expect(promise).rejects.toThrowError('Failed to connect to ROS');
    });
  });
});
