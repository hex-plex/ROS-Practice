## ROSSERIAL_ARDUINO

This is a simple example as shown in the tutorial, But certain steps to be followed to avoid errors that i faced

## Tutorial
Do follow the rosserial_arduino tutorial for understanding and coding the example

### ERRORS
- UNKNOWN LOOKUP ERROR <br/>
     or      
- unknown error handle name 'rosmsg'

they have the same solution. <br/>
``` bash
cd ~/catkin_ws/src
sudo rm -r genpy
git clone https://github.com/ros/genpy.git
cd genpy
## You can try see the git log output to track down the
## hash of the commit with tag 0.6.13
## Its constant so you can just copy and paste
git checkout -b fixes bdece79b95d11aaac542be98e5407233102f3bfc
cd ~/catkin_ws
catkin build
```
