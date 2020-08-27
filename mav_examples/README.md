## MAV-EXAMPLES
This contains the basic mavros package implementations.
### OffBoard Control
Installing the PX4 firmware first
``` bash
cd && mkdir PX4 && cd PX4
git clone https://github.com/PX4/Firmware
cd Firmware
DONT_RUN=1 make px4_sitl_default gazebo 
echo "alias workon-px4-default=\"source ~/PX4/Firmware/Tools/setup_gazebo.bash ~/PX4/Firmware ~/PX4/Firmware/build/px4_sitl_default\"" >> ~/.bashrc
echo "alias reg-px4=\"export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4/Firmware:~/PX4/Firmware/Tools/sitl_gazebo\"" >> ~/.bashrc
```
To use it on any terminal run this first
``` bash
workon-px4-default
reg-px4
roslaunch px4 posix_sixtl.launch ## This will launch the simulator
roslaunch gazebo_ros empty_world.launch world_name:=~/PX4/Firmware/Tools/sitl_gazebo/worlds/iris.world 
##This would initial an empty world as declared and running the command above it you will spawn the quad

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" ## THis will launch a mavlink between the PX4 Firmware and the ROS Core server

rosrun mav_examples offb_node ## For a simple take off
rosrun mav_examples offb_path ## For simple traversal of a circular path
```
