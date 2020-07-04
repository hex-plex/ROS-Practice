#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

Marker makeBox (InteractiveMarker &msg){
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale*0.45;
    marker.scale.y = msg.scale*0.45;
    marker.scale.z = msg.scale*0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg){
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

void frameCallback(const ros::TimeEvent& ){
    static uint32_T counter= 0;
    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0)*2.0 ));
    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br.sendTransform(tf::StampedTransform(t, time, "base_link","moving_frame"));

    t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
    br.sendTransform(tf::StampedTransform(t, time ,"base_link", "rotating_frame"));

    counter++;
}

void processFeedback()
