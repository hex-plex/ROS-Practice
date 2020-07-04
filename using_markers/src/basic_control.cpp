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

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    std::ostringstream s;
    s<<"Feedback from marker '"<<feedback->marker_name<<"' "<<" / control '"<<feedback->control_name <<"'";

    std::ostringstream mouse_point_ss;
    if(feedback->mouse_point_valid){
        mouse_point_ss<<" at "<<feedback->mouse_point.x<<", " <<feedback->mouse_point.y<<", " <<feedback->mouse_point.z<<" in frame "<<feedback->header.frame_id;
    }

    switch(feedback->event_type){
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM(s.str()<<": button click "<<mouse_point_ss.str()<<".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str()<<": menu item "<<feedback->menu_entry_id<<" clicked "<<mouse_point_ss.str()<<".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM( s.str()<<": pose changed"
                <<"\nposition = "
                << feedback->pose.position.x
                <<", "<<feedback->pose.position.y
                <<", "<<feedback->pose.position.z
                <<"\norientation = "
                <<feedback->pose.orientation.w
                <<", "<<feedback->pose.orientation.x
                <<", "<<feedback->pose.orientation.y
                <<", "<<feedback->pose.orientation.z
                <<"\nframe: "<<feedback->header.frame_id
                <<" time: "<<feedback->header.stamp.sec<<"sec, "
                <<feedback->header.stamp.sec<<" nsec");
            break;
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM(s.str()<<": mouse down"<<mouse_point_ss.str()<<".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM(s.str()<<": mouse up"<<mouse_point_ss.str()<<".");
            break;
    }

    server->applyChanges();
}

void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    geometry_msgs::Pose pose = feedback->pose;
    pose.position.x = round(pose.position.x - 0.5)+0.5;
    pose.position.y = round(pose.position.y - 0.5)+0.5;

    ROS_INFO_STREAM( feedback->marker_name<<":"
        <<" aligning position = "
        << feedback -> pose.position.x
        <<", "<<feedback->pose.position.y
        <<", "<<feedback->pose.position.z
        <<" to "
        << pose.position.x
        <<", "<<pose.position.y
        <<", "<<pose.position.z);

    server->setPose(feedback->marker_name, pose);
    server->applyChanges();
}

double rand(double min, double max){
    double t = (double)rand()/(double)RAND_MAX:
    return min + t*(max-min);
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof){
    interactiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;
    int_marker.name = "simple_6dof";
    int_marker.description = "Simple 6-DOF Control";

    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;
    InteractiveMarkerControl control;

    if(fixed)
    {
        int_marker.name+= "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InterativeMarkerControl::FIXED;
    }

    if(interaction_mode!=visualization_msgs::InteractiveMarkerControl::NONE){
        std::string mode_text;
        if(interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_3D) mode_text="MOVE_3D";
        if(interaction_mode==visualization_msgs::InteractiveMarkerControl::ROTATE_3D) mode_text = "ROTATE_3D";
        if(interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D) mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_"+ mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof? "+ 6-DOF controls" : "")+"\n" +mode_text;
    }

    if(show_6dof){

        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name="rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien,normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name="rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE){
        menu_handler.apply(*server,int_marker,name);
    }
}

void makeRandomDOFMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;
    int_marker.name = "6dof_random_axes";
    int_marker.description = "6-DOF\n(Arbitrary Axes)";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    for(int i=0; i<3 ;i++){
        tf::Quaternion orien(rand(-1,1), rand(-1,1), rand(-1,1), rand(-1,1));
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.interactive_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}
