#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

char* char_array;
double pi = 2*std::acos(0.0);
std::vector<mavros_msgs::State> current_states;
void state_cb(const mavros_msgs::State::ConstPtr& msg,int i){
    current_states[i] = *msg;
}
char* ch_it(std::string s){
    int n = s.length();
    char_array = new char [n+1];
    std::strcpy(char_array,s.c_str());
    return char_array;
}
int main(int argc,char **argv){
    ros::init(argc,argv,"offb_node");
    ros::NodeHandle nh;
    int module =3;
    if (argc>1){
        module = std::atoi(argv[1]);
    }
    current_states = std::vector<mavros_msgs::State>(module);
    std::vector<ros::Subscriber> state_subs;
    std::vector<ros::Publisher> local_pos_pubs;
    std::vector<ros::ServiceClient> arming_clients;
    std::vector<ros::ServiceClient> set_mode_clients;
    int i=0,j,t=0;
    for(i=0;i<module;i++){
        state_subs.push_back(nh.subscribe<mavros_msgs::State>(ch_it("/uav"+std::to_string(i)+"/mavros/state"),10,[i](const mavros_msgs::State::ConstPtr& msg){return state_cb(msg,i);}));
        local_pos_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>("uav"+std::to_string(i)+"/mavros/setpoint_position/local",10));
        arming_clients.push_back(nh.serviceClient<mavros_msgs::CommandBool>("uav"+std::to_string(i)+"/mavros/cmd/arming"));
        set_mode_clients.push_back(nh.serviceClient<mavros_msgs::SetMode>("uav"+std::to_string(i)+"/mavros/set_mode"));
    }
    ros::Rate rate(20.0);
    while(ros::ok()&& !current_states[module-1].connected){
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped dummy;
    double r=3,di=1.00;
    double dr = 0.1;
    for(i=0;i<module;i++){
        dummy = *(new geometry_msgs::PoseStamped());
        dummy.pose.position.x = r*sin((i-1)*2*pi/module);
        dummy.pose.position.y = r*cos((i-1)*2*pi/module);
        dummy.pose.position.z = 4;
        poses.push_back(dummy);
    }

    for(i = 100;ros::ok()&& i>0;--i){
        for(j=0;j<module;j++)
            local_pos_pubs[j].publish(poses[j]);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode ="OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    std::vector<ros::Time> last_requests;
    for(i=0;i<module;i++){
        last_requests.push_back(ros::Time::now());}
    i=0;
    while(ros::ok()){
        for(j=0;j<module;j++){
        if(current_states[j].mode != "OFFBOARD" &&((ros::Time::now() - last_requests[j]) > ros::Duration(5.0))){
            if(set_mode_clients[j].call(offb_set_mode)&&offb_set_mode.response.mode_sent){
                ROS_INFO("offBoard uav enabled");
            }
            last_requests[j] = ros::Time::now();
        }
        else{
            if(!current_states[j].armed && (ros::Time::now()-last_requests[j] > ros::Duration(5.0) )){
                if(arming_clients[j].call(arm_cmd)&&arm_cmd.response.success){
                    ROS_INFO("Vehicle uav armed");
                }
                last_requests[j] = ros::Time::now();
            }
        }
        poses[j].pose.position.x = r*sin(0.05*i + ((j-1)*2*pi/module));
        poses[j].pose.position.y = r*cos(0.05*i + ((j-1)*2*pi/module));
        poses[j].pose.position.z += 0.05*di;
        }
        t++;
         if(i<0||i>=300)
           { di*=-1;
            dr*=-1;}
        if(t<=1500)i+=di;
        r+=dr;
        for(int j = 0;j<module;j++)
            local_pos_pubs[j].publish(poses[j]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
