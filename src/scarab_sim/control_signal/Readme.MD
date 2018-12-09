
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





