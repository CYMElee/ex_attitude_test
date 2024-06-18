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
    er.data.resize(3);
    rd.data.resize(4); //publish quaternion form
    ros::Subscriber R =nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,R_cb);
    ros::Publisher eR = nh.advertise<std_msgs::Float32MultiArray>
        ("eR",10);

    ros::Publisher Rd = nh.advertise<std_msgs::Float32MultiArray>
        ("Rd",10);  

    ros::Rate rate(100);  

    while(ros::ok()){

        Attitude_Desire();
        Attitude_Error();
       // ROS_INFO("PUBLISHING RD");
        eR.publish(er);
        Rd.publish(rd);
        t++;
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}


void Attitude_Desire(void)
{   
    
    
    
    theta = 0.5*sin(t*dt);

    Eigen::Quaternionf q1(cos(theta/2), 0, sin(theta/2), 0);
   // Eigen::Quaternionf q2(cos(theta/2), sin(theta/2), 0,0);

    Eigen::Quaternionf q_product = q1;

    rd.data[0] =q_product.w();
    rd.data[1] =q_product.x();
    rd.data[2] =q_product.y();
    rd.data[3] =q_product.z();
    Rr = q_product.toRotationMatrix();
    
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
