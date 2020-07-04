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

void frameCallback(const ros::TimerEvent& ){
    static uint32_t counter= 0;
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
    double t = (double)rand()/(double)RAND_MAX;
    return min + t*(max-min);
}

void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof){
    InteractiveMarker int_marker;
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
        control.orientation_mode = InteractiveMarkerControl::FIXED;
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
        orien.normalize();
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
        menu_handler.apply(*server,int_marker.name);
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
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeViewFacingMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;
    int_marker.name = "view_facing";
    int_marker.description = "View Facing 6-DOF";

    InteractiveMarkerControl control;

    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation.w = 1;
    control.name = "rotate";

    int_marker.controls.push_back(control);

    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    control.orientation_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.independent_marker_orientation = true;
    control.name = "move";

    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;

    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeQuadcopterMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "quadcopter";
    int_marker.description = "Quadcoptor";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeChessPieceMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "chess_piece";
    int_marker.description = "Chess Piece\n(2D Move + Alignment)";

    InteractiveMarkerControl control;

    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);

    server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}

void makePanTiltMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "pan_tilt";
    int_marker.description = "Pan / Tilt";

    makeBoxControl(int_marker);

    InteractiveMarkerControl control;

    tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::FIXED;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = InteractiveMarkerControl::INHERIT;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeMenuMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "context_menu";
    int_marker.description = "Context Menu\n (Right Click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";

    Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true ;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply(*server, int_marker.name);
}

void makeButtonMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "button";
    int_marker.description = "Button\n(Left Click)";

    InteractiveMarkerControl control;

    control.interaction_mode = InteractiveMarkerControl::BUTTON;

    Marker marker = makeBox(int_marker);
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

void makeMovingMarker(const tf::Vector3& position){
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "moving_frame";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "moving";
    int_marker.description = "Marker Attached to a \nMoving Frame";

    InteractiveMarkerControl control;

    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;
    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}


int main(int argc, char** argv){

    ros::init(argc,argv, "basic_controls");
    ros::NodeHandle n;

    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_control","", false));

    ros::Duration(0.1).sleep();

    menu_handler.insert("First Entry", &processFeedback);
    menu_handler.insert("Second Entry", &processFeedback);
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Submenu");
    menu_handler.insert(sub_menu_handle,"First Entry",&processFeedback);
    menu_handler.insert(sub_menu_handle,"Second Entry", &processFeedback);

    tf::Vector3 position;
    position = tf::Vector3(-3, 3, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    position = tf::Vector3( 0, 3, 0);
    make6DofMarker( true, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    position = tf::Vector3( 3, 3, 0);
    makeRandomDOFMarker(  position );
    position = tf::Vector3(-3, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::ROTATE_3D, position, false );
    position = tf::Vector3( 0, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
    position = tf::Vector3( 3, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_3D, position, false );
    position = tf::Vector3(-3,-3, 0);
    makeViewFacingMarker( position );
    position = tf::Vector3( 0,-3, 0);
    makeQuadcopterMarker( position );
    position = tf::Vector3( 3,-3, 0);
    makeChessPieceMarker( position );
    position = tf::Vector3(-3,-6, 0);
    makePanTiltMarker( position );
    position = tf::Vector3( 0,-6, 0);
    makeMovingMarker( position );
    position = tf::Vector3( 3,-6, 0);
    makeMenuMarker( position );
    position = tf::Vector3( 0,-9, 0);
    makeButtonMarker( position );

    server->applyChanges();

    ros::spin();

    server.reset();
}
