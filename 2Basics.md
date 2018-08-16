## Chapter 1: Basic Concepts

## System Requirements

In order to be able to use the rtabmap_ros package to perform RGB-D SLAM, you need to have, at least, a Kinect-like sensor.  
  
Anyways, the recommended robot configuration is the following:

-   A 2D laser which provides  **sensor_msgs/LaserScan messages**.
-   Odometry (IMU, wheel encoders, ...) which provides a  **nav_msgs/Odometry**  message.
-   A calibrated Kinect-like sensor compatible with openni_launch, openni2_launch or freenect_launch ros packages.

Two of the most common setups in order to perform RGB-D Slam are the following:

#### Kinect + Odometry + 2D laser

![enter image description here](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_config.png?raw=true)


#### Kinect + Odometry + Fake 2D laser from Kinect

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_config_fakelaser.jpg?raw=true)



## Data Visualization (RViz)

Add the necessary elements to RViz in order to visualize the following data. 

a) RGB Image  
b) Depth Cloud  
c) Laser Scans

##### You will have to set the Fixed Frame to "base_link" in order to be able to visualize the data.  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/sensors_rviz.png?raw=true)


## Launching RTAB-Map

The rtabmap_ros package, as many other ROS packages, has a set of parameters that you need to set up in order to properly launch it. So let's analyze some of the most important ones. For that, you can have a look at the launch file named  _demo_launch_file.launch_  that is contained in the  _turtlebot_rtab_  package. So, find the mentioned launch file, open it, and have a look at it. It should look like this:  
  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/demo_launch_file.png?raw=true)

Before explaining some of these parameters, you need to know this one thing: when we talk about rtabmap_ros related parameters, we need to differentiate two kind of parameters: ROS Parameters and RTAB-Map Parameters.

## ROS Parameters

The ROS parameters are for connecting the RTAB-Map library with ROS.

-   **frame_id (string, default: "base_link")**: The frame attached to the mobile base.
-   **odom_frame_id (string, default: "")**: The frame attached to odometry. If empty, rtabmap will subscribe to odom topic to get odometry. If set, odometry is got from tf (in this case, a covariance of 1 is used).
-   **subscribe_depth (bool, default: "true")**: Subscribe to depth image.
-   **subscribe_scan (bool, default: "false")**: Subscribe to laser scan.
-   **wait_for_transform (bool, default: "true")**: Wait (maximum wait_for_transform_duration sec) for transform when a tf transform is not still available.
-   **wait_for_transform_duration (double, default: 0.1)**: Wait duration for wait_for_transform. To avoid some possible errors, it is recommended to set this value to "0.2".
-   **database_path (string, default: "~/.ros/rtabmap.db")**: Path of the RTAB-Map's database.

## RTAB-Map Parameters

The RTAB-Map's parameters are those related to the RTAB-Map library.

-   **RGBD/NeighborLinkRefining**: Correct odometry using the input laser topic using ICP.
-   **RGBD/ProximityBySpace**: Find local loop closures based on the robot position in the map. It is useful when the robot, for example, is coming back in the opposite direction. With camera facing back, global loop closures cannot be found. So using the position and previously added laser scans to the map, we find the transform using ICP. Be aware that on large-scale mapping, this method should be disabled because when the odometry is very erroneous, local ICP could give wrong results (false loop closures).
-   **RGBD/AngularUpdate**: The robot should move to update the map (if not 0).
-   **RGBD/LinearUpdate**: The robot should move to update the map (if not 0).
-   **RGBD/OptimizeFromGraphEnd**: Here we optimized from the latest node added to the map instead of the first. By optimizing from the last, the last pose keeps it's value and all the previous poses are corrected according to it (so /odom and /map will always match together). By optimizing from the first, all the successive nodes are corrected according to the first one (so there will be a transformation between /odom and /map to correct the last pose added). However, by optimizing from the first: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).
-   **Optimizer/Slam2D**: Do 2D graph optimization (only optimize x, y and yaw values).
-   **Reg/Strategy**: We chose ICP to refine global loop closures found with ICP using the laser scans.
-   **Reg/Force3DoF**: Force 3DoF registration: roll, pitch and z won't be estimated.
-   **Vis/MinInliers**: Minimum visual words inliers (after RANSAC transformation between images of a loop closure) to accept the transformation.
-   **Vis/InlierDistance**: From the RANSAC transformation computed, the maximum distance of the visual word inliers.
-   **Rtabmap/TimeThr**: The maximum time allowed for map update. When this threshold is reached, some nodes are transferred in the Long-Term Memory to satisfy real-time constraints. The map will be published without these nodes (until retrieved from Long-Term Memory to Working Memory).
-   **Mem/RehearsalSimilarity**: In the papers, it is referred as the Weight Update threshold. If consecutive nodes have similar images (over this threshold), they are merged together, increasing the total weight of the merged node. The weighting mechanism is used for the memory management approach (less weighted nodes will be transferred first to Long-Term Memory).

### Complete look at all the RTAB-Map parameters here: [rtabmap_ros ](http://wiki.ros.org/rtabmap_ros)

---

## Subscribed Topics

In order to work properly, the  _rtabmap_ros_  package needs to subscribe to some topics to get the data it needs. This topics are the following:

-   **rgb/image (sensor_msgs/Image)**: RGB/Mono image. Should be rectified when subscribe_depth is true.
-   **rgb/camera_info (sensor_msgs/CameraInfo)**: RGB camera metadata.
-   **depth/image (sensor_msgs/Image)**: Registered depth image. Required if parameter subscribe_depth is true.
-   **scan (sensor_msgs/LaserScan)**: Laser scan stream. Required if parameter subscribe_scan is true.
-   **odom (nav_msgs/Odometry)**: Odometry stream. Required if parameters subscribe_depth or subscribe_stereo are true and odom_frame_id is not set.

Note that in each system, the name of these topics may vary. This makes (almost always) mandatory to do  **remappings**  from this topics to the ones that are actually providing this data in our system. You can see how this is done within the previously shown launch file.  
  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_remap.png?raw=true)


## Arguments

You can also specify the following arguments to the rtabmap_ros package:

-   **"--delete_db_on_start"**: Delete the database before starting, otherwise the previous mapping session is loaded.
-   **"--udebug"**: Show RTAB-Map's debug/info/warning/error logs.
-   **"--uinfo"**: Show RTAB-Map's info/warning/error logs.
-   **"--params"**: Show RTAB-Map's parameters related to this node and exit.
-   **"--params-all"**: Show all RTAB-Map's parameters and exit.


**`demo_launch_file.launch`**
```xml
<launch>
    <group ns="rtabmap">
        <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
            <param name="frame_id" type="string" value="base_link"/>
            
            <param name="subscribe_depth" type="bool" value="true"/>
            <param name="subscribe_scan" type="bool" value="true"/>
            
            <!--<remap from="odom" to="/base_controller/odom"/>-->
            <remap from="odom" to="/odom"/>
            <!--<remap from="scan" to="/base_scan"/>-->
            <remap from="scan" to="/kobuki/laser/scan"/>
            
            
            <!--<remap from="rgb/image" to="/camera/rgb/image_rect_color"/>-->
            <remap from="rgb/image" to="/camera/rgb/image_raw"/>
            <!--<remap from="depth/image" to="/camera/depth_registered/image_raw"/>-->
            <remap from="depth/image" to="/camera/depth/image_raw"/>
            <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
            
            <!-- RTAB-Map's parameters -->
            <param name="RGBD/NeighborLinkRefining" type="string" value="true"/>
            <param name="RGBD/ProximityBySpace"     type="string" value="true"/>
            <param name="RGBD/AngularUpdate"        type="string" value="0.01"/>
            <param name="RGBD/LinearUpdate"         type="string" value="0.01"/>
            <param name="RGBD/OptimizeFromGraphEnd" type="string" value="false"/>
            <param name="Optimizer/Slam2D"          type="string" value="true"/>
            <param name="Reg/Strategy"              type="string" value="1"/> 
            <param name="Reg/Force3DoF"             type="string" value="true"/>
            <param name="Vis/MinInliers"            type="string" value="5"/>
            <param name="Vis/InlierDistance"        type="string" value="0.1"/>
            <param name="Rtabmap/TimeThr"           type="string" value="700"/>
            <param name="Mem/RehearsalSimilarity"   type="string" value="0.45"/>
        </node>
    </group>
</launch>
```












#
<!--stackedit_data:
eyJoaXN0b3J5IjpbMTk4ODI5NzU1M119
-->