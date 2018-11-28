# Dependencies:

1. sudo apt-get install ros-kinetic-yocs-cmd-vel-mux
2. sudo apt-get install ros-kinetic-gmapping
3. sudo apt-get install ros-kinetic-turtlebot-teleop
4. sudo apt-get install ros-kinetic-kobuki-gazebo
5. sudo apt-get install ros-kinetic-hector-gazebo-plugins
6. sudo apt-get install ros-kinetic-turtlebot-gazebo
7. sudo apt-get install ros-kinetic-turtlebot-navigation
8. git clone https://github.com/KumarRobotics/kr_utils.git PATH_TO_HERE/deepmotionprimitives/tf-docker-data/deepmotionprimitives/code/cpp_ws/src/kr_utils

# Proper Launch Order:

1. Load world: roslaunch nx_launch spawn_world.launch world:=fla_warehouse1
2. Spawn robot: roslaunch nx_launch spawn_scarab.launch ns:=scarab40
3. Start teleop: roslaunch nx_launch scarab_teleop.launch ns:=scarab40
4. Start SLAM:  roslaunch nx_launch scarab_slam.launch ns:=scarab40
5. Start rviz: rosrun rviz rviz -d $(rospack find nx_launch)/rviz_cfg/scarab40.cfg.rviz
6. Save map once done: rosrun map_server map_saver map:=/scarab40/map -f ./mymap

# HFN Demo:

1. Launch everything: roslaunch nx_launch drlp_demo.launch world:=fla_warehouse1 use_localization:=false use_slam:=true use_move_base:=false use_hfn:=true use_teleop:=false px:=0.0 py:=0.0 pY:=0.0
2. Publish hfn/move_base goal: rostopic pub /scarab40/move_base_simple/goal geometry_msgs/PoseStamped '{ header: {stamp: now, frame_id:  "/map"}, pose: { position: { x: 12.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }'

# Sikang Localization Demo:

1. roslaunch nx_launch drlp_demo.launch use_localization:=true use_scarab_loc:=false use_slam:=true use_move_base:=false use_hfn:=false use_teleop:=true px:=0.0 py:=0.0 pY:=0.0


# DRLP Demo:

1. Launch everything: roslaunch nx_launch drlp_demo.launch use_slam:=true use_hfn:=true px:=0.0 py:=0.0 pY:=0.0
2. Publish DRLP goal: python dl_continuous_action_2.py 0_to_7000000/7109780_keyboardinterrupt.h5 --gx=31 --gy=-9


# TODO:

1. Simplify the laser_pose_estimator




## How-to:

### NOTE: UNCOMMENT MAP_SERVICE IN BEARING_SENSOR.YAML FILE

### TELEOP SLAM
1. Load world: roslaunch nx_launch spawn_world.launch world:=doors_chairs
2. Spawn robots: roslaunch nx_launch spawn_robots.launch ns:=scarab03
3. Start teleop: roslaunch nx_launch scarab_teleop.launch ns:=scarab03
4. Start detection: roslaunch nx_launch scarab_object_detection.launch ns:=scarab03
5. Start SLAM:  roslaunch nx_launch scarab_slam.launch ns:=scarab03
6. Start PHD filter: roslaunch nx_launch scarab_phd_filter.launch ns:=scarab03
7. Start rviz: rosrun rviz rviz -d $(rospack find nx_launch)/rviz_cfg/turtlebot.cfg.rviz
8. Steer robot using the teleop terminal
9. Save map once done: rosrun map_server map_saver map:=/scarab03/phd_grid -f ./result/mymap
###9. Save map once done: rosrun map_server map_saver


### Dependencies
 1. sudo apt-get install ros-indigo-yocs-cmd-vel-mux
 2. sudo apt-get install ros-indigo-gmapping
 3. sudo apt-get install ros-indigo-turtlebot-teleop
 4. sudo apt-get install ros-indigo-kobuki-gazebo
 5. sudo apt-get install ros-indigo-hector-gazebo-plugins
 
 
### (OLD) RUN SIMULATION
1. cd ~/catkin_ws/src/scarab_sim
2. roslaunch run_scarab_sim.launch (keyboard commands to drive scarab)
3. (in new terminal) rosbag record -O simtest1.bag /scarab03/map /scarab03/odom /scarab03/object_detector/object_detections /scarab03/camera/rgb/image_raw /scarab03/measurements /scarab03/phd_grid /tf
4. rosbag play simtest1.bag
5. rqt_bag

