# Proper Launch Order:

(TELEOP SLAM)
1. Load world: roslaunch nx_launch spawn_world.launch world:=doors_chairs
2. Spawn robots: roslaunch nx_launch spawn_robots.launch ns:=scarab03
3. Start teleop: roslaunch nx_launch turtlebot_teleop.launch
4. Start SLAM:  roslaunch nx_launch start_slam.launch ns:=scarab03
5. Start rviz: rosrun rviz rviz -d $(rospack find nx_launch)/rviz_cfg/turtlebot.cfg.rviz
6. Save map once done: rosrun map_server map_saver

(Human Friendly Navigation)
1. Load world: roslaunch nx_launch spawn_world.launch world:=doors_chairs
2. Spawn robots: roslaunch nx_launch spawn_robots.launch ns:=scarab03
3. Load map: roslaunch nx_launch scarab_prior_map.launch world:=doors_chairs
4. Start hfn: roslaunch nx_launch scarab_hfn.launch
5. Start rviz: rosrun rviz rviz -d $(rospack find nx_launch)/rviz_cfg/turtlebot.cfg.rviz


