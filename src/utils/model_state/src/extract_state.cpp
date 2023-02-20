#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
/*-----------------PARAM_SAVED-----------------*/
Eigen::Vector3d odom_pos_, odom_vel_;
Eigen::Quaterniond odom_orient_;
geometry_msgs::PoseStamped  msg_pos;
geometry_msgs::TwistStamped msg_vel;
nav_msgs::Odometry init_state;

void orb_cb(const nav_msgs::Odometry::ConstPtr &msg_)
{
    nav_msgs::Odometry objstate;
    objstate = *(msg_);
    msg_vel.twist  = objstate.twist.twist;
    msg_vel.header = objstate.header;
    msg_pos.pose   = objstate.pose.pose;
    msg_pos.header = objstate.header;      
}

bool first_call = true;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"extract_state_client");
    ros:: NodeHandle nh;

    ros::Subscriber orb_sub = nh.subscribe<nav_msgs::Odometry>
                                ("/orb_odom", 10, orb_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                ("/orb_odom/pose",1);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                                ("/orb_odom/velosity",1);
    
    while (ros::ok())
    {
        pos_pub.publish(msg_pos);
        vel_pub.publish(msg_vel);
    }
    return 0;
}
