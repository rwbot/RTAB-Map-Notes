## What is RTAB-Map?[](https://i-0658391eb9671438e.robotigniteacademy.com/jupyter/notebooks/Rtabmap_0.ipynb#What-is-RTAB-Map?)

RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D SLAM approach based on a loop closure detector. The loop closure detector uses a bag-of-words approach in order to determinate if a new image detected by an RGB-D sensor it is from a new location or from a location that it has been already visited. Of course, this is a very summarized explanation, you will get more details on how this loop closure detector works inside this Course. For using this approach in ROS, there exists the following package:

### rtabmap_ros[](https://i-0658391eb9671438e.robotigniteacademy.com/jupyter/notebooks/Rtabmap_0.ipynb#rtabmap_ros)

The  **rtabmap_ros**  package is an implementation in ROS of the RTAB-Map approach. It basically allows us to work with the RTAB-Map approach in ROS. It has been developed by Mathieu Labbe. This is the package we will be working with during this Course.

## Do you want to have a taste?[](https://i-0658391eb9671438e.robotigniteacademy.com/jupyter/notebooks/Rtabmap_0.ipynb#Do-you-want-to-have-a-taste?)

With the proper introductions made, it is time to actually start. And... as we always do in the Robot Ignite academy, let's start with practice! In the following example you will be testing the rtabmap_ros package with a pre-built setup, so you can have a practical look at what you can achieve with this package, which actually is what you are going to learn by completing this Micro Course. So... let's go!
#### Demo
`roslaunch rtabmap_ros demo_robot_mapping.launch rviz:=true rtabmapviz:=false`
`rosservice call /gazebo/pause_physics "{}"`
`roscd turtlebot_rtab`
`rosbag play --clock demo_mapping.bag`


#### **NOTE: When you are done with the demo, don't forget to unpause the simulation physics by using the following command.**
`rosservice call /gazebo/unpause_physics "{}"`

#### **NOTE: Also, when you finish with the demo, execute the following commands in order to remove a couple of parameters that have been sent and could cause conflicts during the Course.**

`rosparam delete /rtabmap/rtabmap/depth/image_transport`
`rosparam delete /rtabmap/rtabmap/rgb/image_transport`











<!--stackedit_data:
eyJoaXN0b3J5IjpbNzM0NjY0MDQ0XX0=
-->