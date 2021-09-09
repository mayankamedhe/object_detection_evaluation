# object_detection_evaluation

To run:

At autoware.proj.gsm8-
-	source install/setup.bash
-	roslaunch perception_launch lidar_based_detection.launch
-	roslaunch sensing_launch sensing.launch sensor_model:=aip_x2
-	rosbag play --clock TC1-45mm-2_2021-05-17-15-01-18.bag
-	rosrun rviz rviz -d src/autoware/launcher/autoware_launch/rviz/autoware.rviz

At catkin_ws-
To make-
(In catkin_ws) - catkin_make

-	source devel/setup.bash 
- 	source /home/mayanka/autoware.proj.gsm8/install/setup.bash (for importing autoware_perception_msgs.msg)
-	rosrun detection_evaluation scripts/evaluation.py
-	rosrun using_markers basic_shapes
