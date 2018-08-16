## Mapping Mode

So, now that you have already seen how to properly launch the rabmap_ros package, it is time to give it some use!! And the first thing we are going to do is to generate a Map of the environment. So... let's go! For that, you'll need to create a new launch file in order to launch both RTAB-Map and the Navigation system.  
  
For building this new launch file, you should take these 2 things into account:
1.  Regarding the Navigation System, you will just need to launch the  _move_base_  node, since the SLAM process will be handled by the  _rtabmap_ros_package itself.
3.  By default, the rtabmap_ros package publishes the grid map that it's created into a  _/grid_map_  topic. In the Navigation system, though, the grid map is read from the  _/map topic_. So, you'll need to do the propper remap here.

`rtab_mapping.launch`
```xml
<launch>
  
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="args"              default=""/>
  
  <arg name="wait_for_transform"  default="0.2"/> 
  
  <!-- Navigation stuff (move_base) -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base_rtab.launch.xml"/>
  
  <!-- Mapping -->
  <group ns="rtabmap">

    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)">
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="wait_for_transform_duration"  type="double"   value="$(arg wait_for_transform)"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
    
      <!-- inputs -->
      <remap from="scan"            to="/kobuki/laser/scan"/>
      <remap from="rgb/image"       to="/camera/rgb/image_raw"/>
      <remap from="depth/image"     to="/camera/depth/image_raw"/>
      <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
      
      <!-- output -->
      <remap from="grid_map" to="/map"/>
    
      <!-- RTAB-Map's parameters: do "rosrun rtabmap rtabmap (double-dash)params" to see the list of available parameters. -->
      <param name="RGBD/ProximityBySpace"        type="string" value="true"/>   
      <param name="RGBD/OptimizeFromGraphEnd"    type="string" value="false"/>  
      <param name="Kp/MaxDepth"                  type="string" value="4.0"/>
      <param name="Reg/Strategy"                 type="string" value="1"/>      
      <param name="Icp/CoprrespondenceRatio"     type="string" value="0.3"/>
      <param name="Vis/MinInliers"               type="string" value="5"/>      
      <param name="Vis/InlierDistance"           type="string" value="0.1"/>    
      <param name="RGBD/AngularUpdate"           type="string" value="0.1"/>    
      <param name="RGBD/LinearUpdate"            type="string" value="0.1"/>    
      <param name="Rtabmap/TimeThr"              type="string" value="700"/>
      <param name="Mem/RehearsalSimilarity"      type="string" value="0.30"/>
      <param name="Optimizer/Slam2D"             type="string" value="true"/>
      <param name="Reg/Force3DoF"                type="string" value="true"/>   
      
    </node>
   
  </group>
</launch>
```

### To Map:
- Launch the rtabmap and move_base nodes:
`roslaunch rtabrw rtab_mapping.launch`
- Launch rviz:
`roslaunch rtabmap_ros demo_turtlebot_rviz.launch`
- Launch teleop to control:
`roslaunch turtlebot_teleop keyboard_teleop.launch`

---
After a mapping session as above, a database is saved here ~/.ros/rtabmap.db. Into this database, the rtabmap_ros package stores, for instance, images from the mapping session that will be later used for  **detecting loop closures**.

In order to better explain how this works, let's access to this database. The rtabmap_ros package offers a tool that allows us to visualize the content of this database: the  **RTAB-Map's Database Viewer**. You can open this tool using the following command:

`rtabmap-databaseViewer ~/.ros/rtabmap.db`

 ![enter image description here](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_viewer.png?raw=true)

This images you are seeing right now are the different images that have been taken during the Mapping session. If you move the "Id" scroll button that appears at the bottom of the screen, you will move through all the different images that are stored in the database.

But, as you may have noticed, most of the images have some strange yellow marks on them (for now, forget about the pink marks). What are these strange yellow marks? Can you guess? Well, basically,  **this overlapping yellow disks are marking/highlightning the key features of each image**. And... how are the key features of each image selected? That's another great question!  
  
This visual features used by RTAB-Map are using some popular techniques from computer vision including like SIFT, SURF, BRIEF, FAST, BRISK, ORB or FREAK. Most of these algorithms look for large changes in intensity in different directions around a point in the image. If you check into the different images, you will notice that there are no yellow discs centered on the homogeneous parts of the image such as the walls or the floor. Instead, the discs are inserted into areas where there are changes in intensity such as the corners. Corner-like features tend to be stable properties of a given location and can be easily detected even under different lighting conditions or when the robot’s view is from a different angle or distance from an object.

The rtabmap_ros package records these collections of visual features in memory as the robot maps the area. At the same time, a machine learning technique known as the “bag of words model” looks for patterns in the features that can then be used to classify the various images as belonging to one location. For instance, there may be a hundred different video frames like the one shown above but from slightly different viewpoints that all contain visual features similar enough to assign to the same location.

That's awesome, right? But now... what are those pink discs that appear in some cases? Could you find out what they mean? Can you detect in which cases they appear? I'll give you some minutes...

So what? Did you discovered anything? Well... let's solve the mystery! The  **pink discs indicate visual features that two images have in common**. For instance, if you select in the Database Viewer 2 images with the same Id (so that are the same image), there should be lots of visual features in common between both images, right?  

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_common.png?raw=true)  

On the other hand, if we select 2 images that don't have key features in common, we won't get any pink disk.  

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_different.png?raw=true)  
Based on the number of shared features and their geometric relations to one another, we can determine if the two views should be assigned to the same location or not. In this way, only a subset of the visual features needs to be stored in long term memory while still being able to recognize a location from many different viewpoints. As a result, RTAB-Map can map out large areas such as an entire building or an outdoor campus without requiring an excessive amount of memory storage or processing power to create or use the map.  
  
So... what do you say? Amazing, right? Well, now you know a little bit better how the whole process works, let's move to the Localization section!

---
---

## Localization Mode

So, after we have already mapped the environment, we can then relaunch the rtabmap_ros package within the localization mode.

In order to launch the package in localization mode, you need to take into account the following:

-   The RTAB-Map Parameter  **_Mem/IncrementalMemory_**  has to be set to false, and  **_Mem/InitWMWithAllNodes_**  has to be set to true.

The recommended way of doing this is adding to the previous launch file the required parameters for the localization mode, and then adding a condition to them.
`<param     if="$(arg name_of_the_argument)" name="NameOfTheParam" type="string" value="false"/>`

This way, you can launch the localization mode by just adding a parameter to the launch file you've already created. Like this:
`roslaunch rtabmap_ros demo_turtlebot_mapping.launch localization:=true`

When the localization mode is launched, you can move the robot around the environment until it can relocalize in the previous map. Then, **the 2D map would re-appear again when a loop closure is found**.
* In order to be able to relocalie itself, the package must detect a loop closure. So, you should move the robot to a place where you know your database has recognizable images from.  
* When your robot localizes itself and the whole 2D map appears, if the 3D map doesn't appear, you can just click on the "Download map" option in Rtabmap cloud panel.

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/download_map.png?raw=true)


- Robot without localizing himself:

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/un_localized.png?raw=true)


- Robot localized:

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/localized.png?raw=true)

---
**`rtab_localization.launch`**

```xml
<launch>
  
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="localization"      default="false"/>
  <arg name="args"              default=""/>
  
  <arg name="wait_for_transform"  default="0.2"/> 
  
  <!-- Navigation stuff (move_base) -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base_rtab.launch.xml"/>
  
  <!-- Mapping -->
  <group ns="rtabmap">

    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)">
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="wait_for_transform_duration"  type="double"   value="$(arg wait_for_transform)"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
    
      <!-- inputs -->
      <remap from="scan"            to="/kobuki/laser/scan"/>
      <remap from="rgb/image"       to="/camera/rgb/image_raw"/>
      <remap from="depth/image"     to="/camera/depth/image_raw"/>
      <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
      
      <!-- output -->
      <remap from="grid_map" to="/map"/>
    
      <!-- RTAB-Map's parameters: do "rosrun rtabmap rtabmap (double-dash)params" to see the list of available parameters. -->
      <param name="RGBD/ProximityBySpace"        type="string" value="true"/>   
      <param name="RGBD/OptimizeFromGraphEnd"    type="string" value="false"/>  
      <param name="Kp/MaxDepth"                  type="string" value="4.0"/>
      <param name="Reg/Strategy"                 type="string" value="1"/>      
      <param name="Icp/CoprrespondenceRatio"     type="string" value="0.3"/>
      <param name="Vis/MinInliers"               type="string" value="5"/>      
      <param name="Vis/InlierDistance"           type="string" value="0.1"/>    
      <param name="RGBD/AngularUpdate"           type="string" value="0.1"/>    
      <param name="RGBD/LinearUpdate"            type="string" value="0.1"/>    
      <param name="Rtabmap/TimeThr"              type="string" value="700"/>
      <param name="Mem/RehearsalSimilarity"      type="string" value="0.30"/>
      <param name="Optimizer/Slam2D"             type="string" value="true"/>
      <param name="Reg/Force3DoF"                type="string" value="true"/>   
      
      <!-- localization mode -->
      <param     if="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="false"/>
      <param unless="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="true"/>
      <param name="Mem/InitWMWithAllNodes" type="string" value="$(arg localization)"/> 
    </node>
   
  </group>
</launch>

```

---
## Autonomous Navigation

So now... you're ready to autonomously navigate your robot around the map!  
  
In order to make the robot move, you just need to send goals to the  _/move_base/goal_  topic, as you would normally do when using the Navigation stack. If you want to send goals through Rviz, you just have to use the  _2D Nav Goal_  option as shown in the picture below:

- Note 1: Autonomous Navigation won't work until the robot localizes itself correctly. This means, until it finds a loop closure.  
- Note 2: Make sure that you are not teleoperating the robot. If you have the keyboard_teleop.launch launched, autonomous navigation may not work.

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/autonomous_navigation_final.png?raw=true)















<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE5NTE0NDc2OV19
-->