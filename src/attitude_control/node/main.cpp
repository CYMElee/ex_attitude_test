#include "ros/ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include "mavros_msgs/AttitudeTarget.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;

mavros_msgs::AttitudeTarget T;

std_msgs::Bool trigger;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Rd_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    T.orientation.w = msg->data[0];
    T.orientation.x = msg->data[1];
    T.orientation.y = msg->data[2];
    T.orientation.z = msg->data[3];
    T.thrust = 0.4;
}


int main(int argv,char** argc)
{
    ros::init(argv,argc,"main");
    ros::NodeHandle nh;
    int UAV_ID;
    trigger.data = 0;
    T.type_mask = T.IGNORE_PITCH_RATE | \
    T.IGNORE_ROLL_RATE |T.IGNORE_YAW_RATE ;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    ros::Subscriber Rd_sub = nh.subscribe<std_msgs::Float32MultiArray>
        ("Rd", 10, Rd_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    /*Attitude control*/
    ros::Publisher T_pub = nh.advertise<mavros_msgs::AttitudeTarget >
            ("mavros/setpoint_raw/attitude", 10);
    /*trigger*/
    ros::Publisher z_pub = nh.advertise<std_msgs::Bool>
            ("trigger", 10);
    /*Attitude start generate*/
    ros::Publisher a_pub = nh.advertise<std_msgs::Bool>
            ("Ag", 10);


    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ros::param::get("UAV_ID", UAV_ID);
    ROS_INFO("Wait for setting origin and home position...");
    std::string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic);
   // ROS_ERROR("Message received or timeout reached. Continuing execution.");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    //send a few setpoints before starting

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 0.5;
    if (takeoff_cl.call(srv_takeoff)) {
     
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }
    ros::Time time_out0 = ros::Time::now();
    while(ros::ok() && ros::Time::now() - time_out0 <ros::Duration(8)){
    trigger.data = 1;
    z_pub.publish(trigger);
    ROS_ERROR("takeoff");
    ros::spinOnce();
    rate.sleep();
    }
    trigger.data = 0;
    z_pub.publish(trigger);
    sleep(2);
    a_pub.publish(trigger);
    ros::Time time_out = ros::Time::now();
    while(ros::ok() && ros::Time::now() - time_out <ros::Duration(30) ){
        T_pub.publish(T);
       // ROS_ERROR("Thrust control!!");
        ros::spinOnce();
        rate.sleep();
    }
        ROS_WARN("kill!");
        offb_set_mode.request.custom_mode = "LAND";
        set_mode_client.call(offb_set_mode);
        arm_cmd.request.value = false;
        arming_client.call(arm_cmd);
        sleep(3);

    return 0;
}
