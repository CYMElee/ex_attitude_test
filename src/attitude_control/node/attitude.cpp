/*this node use to sub the current attitude and pub the desire attitude*/

#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include "Eigen/Dense"
#include "cmath"

float t = 0;
float dt = 0.01;
float theta = 0;

std_msgs::Float32MultiArray er;
std_msgs::Float32MultiArray rd;
Eigen::Matrix<float, 3, 3> E;
Eigen::Matrix<float, 3, 3> R; //measure attitude
Eigen::Matrix<float, 3, 3> Rr; // desire attitude
Eigen::Vector3f eR;

Eigen::Quaternionf quaternion;

void Attitude_Desire(void);
void Attitude_Error(void);
void R_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

int main(int argv,char** argc)
{
    ros::init(argv,argc,"Attitude");
    ros::NodeHandle nh;
    int UAV_ID;
    er.data.resize(3);
    rd.data.resize(4); //publish quaternion form
    ros::param::get("UAV_ID", UAV_ID);
    std::string Rd_Topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/trigger");
    ros::Subscriber R =nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,R_cb);
    ros::Publisher eR = nh.advertise<std_msgs::Float32MultiArray>
        ("eR",10);

    ros::Publisher Rd = nh.advertise<std_msgs::Float32MultiArray>
        ("Rd",10);  

    ros::Rate rate(100);  
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(Rd_Topic);
    ROS_ERROR("Start_Generate_Rd");
    while(ros::ok()){
        Attitude_Desire();
        Attitude_Error();
        t++;
        eR.publish(er);
        Rd.publish(rd);
       
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}


void Attitude_Desire(void)
{   
    theta = 0.5*sin(t*dt);
    float yaw = 0.0;
    float pitch = theta;
    float roll = theta;


    Eigen::Quaternionf quaternion;
    
    quaternion = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    rd.data[0] =quaternion.w();
    rd.data[1] =quaternion.x();
    rd.data[2] =quaternion.y();
    rd.data[3] =quaternion.z();
    Rr = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
               * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
               * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    
}

void Attitude_Error(void)
{
    E = 0.5*(Rr.transpose()*R-R.transpose()*Rr);
    eR(0) = E(2,1);
    eR(1) = E(0,2);
    eR(2) = E(1,0);
    er.data[0] = eR(0);
    er.data[1] = eR(1);
    er.data[2] = eR(2);
}

void R_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    quaternion = Eigen::AngleAxisf(msg->pose.orientation.w, \
    Eigen::Vector3f(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z));
    R = quaternion.toRotationMatrix();

}
