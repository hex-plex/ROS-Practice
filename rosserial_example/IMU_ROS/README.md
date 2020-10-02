## IMU_ROS
This is code snippet that can transmit the data IMU data that is MPU9250 in my case to the roscore using rosserial_arduino either by wire or over wifi.

### Usage
''' bash
  roscore &
  rosrun rosserial_python serial_node /dev/ttyACM0 ## or the port you want
  rosrun rosserial_python serial_node.py tcp ## If your using a esp
'''
This should make it a publisher
''' bash
  rosrun rviz rviz
'''
Open rviz and add a IMU element using the plugin then you can visualize the accelerometer values.
