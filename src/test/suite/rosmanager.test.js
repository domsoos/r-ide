const assert = require('assert');
const ROS = require('../../ROSManagers/rosmanager');

describe('ROS', function() {
  it('should return a resolved promise if already connected', function() {
    ROS.rosAPI = {
      isConnected: true
    };
    const ros = new ROS();
    return ros.then(() => {
      assert.ok(true);
    });
  });

  it('should return a rejected promise if there is an error', function() {
    ROS.rosAPI = null;
    const ros = new ROS();
    return ros.catch((error) => {
      assert.strictEqual(error.message, 'Cannot read property \'isConnected\' of null');
    });
  });

  it('should connect to ROS', function(done) {
    ROS.rosAPI = null;
    const ros = new ROS();
    ros.then(() => {
      assert.ok(ROS.rosAPI instanceof ROSLIB.Ros);
      assert.ok(ROS.rosLib instanceof ROSLIB);
      done();
    });
  });

  it('should get the ROS API', function() {
    const rosAPI = ROS.getROSApi();
    assert.strictEqual(rosAPI, ROS.rosAPI);
  });

  it('should get the ROSLib', function() {
    const rosLib = ROS.getROSLib();
    assert.strictEqual(rosLib, ROS.rosLib);
  });

  it('should reconnect to ROS', function(done) {
    ROS.rosAPI = {
      close: function() {}
    };
    const ros = ROS.reconnect();
    ros.then(() => {
      assert.ok(ROS.rosAPI instanceof ROSLIB.Ros);
      assert.ok(ROS.rosLib instanceof ROSLIB);
      done();
    });
  });
});
