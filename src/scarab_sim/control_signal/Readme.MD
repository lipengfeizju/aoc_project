
sudo apt-get install ros-kinetic-husky-simulator
sudo apt-get install ros-kinetic-husky-navigation
sudo apt-get install ros-kinetic-husky-desktop
sudo apt-get install ros-kinetic-husky-teleop
roslaunch husky_teleop teleop.launch


sudo apt-get install ros-kinetic-ackermann-controller ros-kinetic-ackermann-msgs
sudo apt-get install ros-kinetic-serial*

roslaunch control_signal husky_test.launch 
roslaunch control_signal husky_con.launch 
rostopic echo -p /gazebo/model_states   > ~/gaz_car2.csv




1. start gazebo: roslaunch control_signal Spawn_race.launch
2. start rviz: roslaunch control_signal Simu_race.launch 
3. roscd control_signal/src/
4. start control: python move_racecar.py 


