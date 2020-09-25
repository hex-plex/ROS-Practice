#include <ros.h>
ros::NodeHandle nh;

void setup()
{
  nh.initNode();
}

void loop()
{
  while (!nh.connected())
  {
    nh.spinOnce();
  }
 
  nh.logdebug("Debug Statement");
  nh.loginfo("Program info");
  nh.logwarn("Warnings.");
  nh.logerror("Errors..");
  nh.logfatal("Fatalities!");
  delay(5000);
}
