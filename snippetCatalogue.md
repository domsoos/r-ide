# R-IDE Code Snippets and Code Templates

This catalog provides a list of code snippets and code templates in the R-IDE extension. The catalog is organized into two main categories with subcategories for each:

1. R-IDE Code Snippets
 - C++ Snippets
 - Python Snippets
2. R-IDE Code Templates
 - C++ Templates
 - Python Templates

Each code snippet will be displayed as: 

*call to snippet* 

Brief description of code snippet.

```
code snippet name

block of code

```
Each code template will be displayed as: 


Brief description of code template.

```
code template name

block of code

```


## R-IDE Code Snippets

___
###    C++ Snippets
___
*initialize ROS node*

Initialize the node through this function, provides line arguments to ROS and allows the user to name the node. 

```
ros::init

ros::init(argc, argv, "my_node");

```
*nodeHandle*

Starting a node is often done through the creation of the nodeHandle function. 
```
ros::NodeHandle

ros::NodeHandle nh("my_namespace");

```
*publisher*

A node in ROS that broadcasts a message, often referred to as the talker node. 
```
ros::Publisher 

ros::Publisher /*pub_name*/;
/*pub_name*/ = nh.advertise< /*msg_type*/>("/*topic_name*/", 10);

```
*Rate::sleep*

Used in a loop that processes ROS messages, ensures the loop runs at a consistent frequency. 
```
ros::Rate::sleep

ros::Rate loop_rate(int);
loop_rate.sleep();

```
*spinOnce*

Called in a loop to check for messages associated with callback functions. 
```
ros::spinOnce

ros::spinOnce();

```
*subscriber*

A node in ROS that receives a message, often referred to as the listener node. 
```
ros::Subscriber

ros::Subscriber /*sub_name*/;
/*sub_name*/ = nh.subscribe</*msg_type*/>("/*topic_name*/", 10, /*subscribe_callback_name*/);

```
*ROS_INFO*

Used to log information to the console or to a file, provides feedback about the state of a node. 
```
ROS_INFO

ROS_INFO("Message");

```
___
###   Python Snippets
___
*get_param*

Provides a way to retrieve parameter values, parameters can be used to store configuration data that can be accessed by a node. 
```
get_param

rospy.get_param(param_name)

```
*initialize ROS node*

Initialize the node through this function, provides line arguments to ROS and allows the user to name the node. 
```
Initialize

rospy.init_node('node_name', anonymous=True)

```
*is_shutdown*

Returns a boolean value indicating whether a ROS node is still running. 
```
is_shutdown

rospy.is_shutdown()

```
*logging messages*

Used to log information to the console or to a file, provides feedback about the state of a node. 
```
Logging

rospy.loginfo(msg, *args)

```
*on_shutdown*

Registers a callback function to be called when a node is shutdown, an can be used to perform cleanup operations or save state. 
```
on_shutdown

rospy.on_shutdown(string)

```
*publisher*

A node in ROS that broadcasts a message, often referred to as the talker node. 
```
Publisher

message_pub = rospy.Publisher("topic", type, queue_size=10)

```
*service*

Allows a node to provide a ROS service. 
```
service

rospy.Service(name, service_class, handler, buff_size=65536)

```
*set_param*

Provides a way to set parameter values, parameters can be used to store configuration data that can be accessed by a node. 
```
set_param

rospy.set_param(param_name, param_value)

```
*sleep*

Suspends the execution of a node for a specified amount of time. 
```
Sleep

rospy.sleep(int)

```
*spin*

Called in a loop to continuously check for messages associated with callback functions. 
```
Spin

rospy.spin()

```
*subscriber*

A node in ROS that receives a message, often referred to as the listener node. 
```
Subscriber

rospy.Subscriber("topic", Type, callback)

```
*rate*

Provides a simple way to ensure the node runs at a fixed frequency.
```
rate

rospy.Rate(hz)

```
*wait_for_service*

Used to have a node wait for a service to become available. 
```
wait_for_service

rospy.wait_for_service(service, timeout=None)

```

## R-IDE Code Templates


___
###     C++ Templates
___
```
Publisher Node Example

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

/**
 * Reference: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

```

```
Subscriber Node Example

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
/**
 * Reference: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

```

___
###   Python Templates
___
```
Publisher Node Example

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def talker():

    pub = rospy.Publisher('chatter', String, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("talker_listener", anonymous=True)
    rospy.Subscriber("listener", String, callback)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# Reference: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29


```

```
Subscriber Node Example

#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

# Reference: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

```