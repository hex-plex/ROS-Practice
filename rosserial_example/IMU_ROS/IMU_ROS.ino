#include "MPU9250.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Wire.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  const char* ssid = "No";
  const char* pass = "abcdq1987654";
  IPAddress server(192,168,43,18);
  const uint16_t serverPort = 11411;
#endif 
MPU9250* IMU;
int status;

ros::NodeHandle nh;

sensor_msgs::Imu msg_dat;
ros::Publisher raw("imu_raw",&msg_dat);
void setup() {
#ifdef ESP8266
  Wire.begin(D1,D2);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid,pass);
  while(WiFi.status()!=WL_CONNECTED){
      delay(500);
      Serial.print(".");
    }
  nh.getHardware() -> setConnection(server,serverPort);
  nh.initNode();
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

#else
  Wire.begin();
  nh.initNode();
#endif
  IMU = new MPU9250(Wire,0x68);
  while(!nh.connected()){
      nh.spinOnce();
    }
  status = IMU->begin();
  char buff[7];
  if(status<0){
      nh.logfatal("IMU initialization unsuccessful");
      nh.logerror("Check IMU wiring or try cycling power");
      itoa(status,buff,10);
      nh.loginfo(strcat("Status: ",buff));
      while(1){nh.spinOnce();delay(1000);}  // Nodemcu doesnt like empty while loops
    }
  nh.advertise(raw);
  msg_dat.header.frame_id="/base_link";
  msg_dat.orientation.x=0;msg_dat.orientation.y=0;msg_dat.orientation.z=0;msg_dat.orientation.w=0;
  for(int i=0;i<9;i++)msg_dat.orientation_covariance[i] =-1;
  for(int i=0;i<9;i++)msg_dat.angular_velocity_covariance[i]=0;
  for(int i=0;i<9;i++)msg_dat.linear_acceleration_covariance[i] = 0;
}

void loop() {
  IMU->readSensor();
  msg_dat.angular_velocity.x = IMU->getGyroX_rads();
  msg_dat.angular_velocity.y = IMU->getGyroY_rads();
  msg_dat.angular_velocity.z = IMU->getGyroZ_rads();
  msg_dat.linear_acceleration.x = IMU->getAccelX_mss();
  msg_dat.linear_acceleration.y = IMU->getAccelY_mss();
  msg_dat.linear_acceleration.z = IMU->getAccelZ_mss();
  msg_dat.header.stamp = nh.now();
  raw.publish(&msg_dat);
  nh.spinOnce();
  delay(50);
}
