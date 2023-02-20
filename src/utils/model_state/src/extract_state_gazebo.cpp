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
nav_msgs::Odometry msg;
nav_msgs::Odometry init_state;

bool first_call = true;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"extract_state_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>
                                ("/gazebo/get_model_state");
    ros::Publisher fsm_pub    = nh.advertise<nav_msgs::Odometry>
                                ("/orb_odom",1);
    ros::Publisher pos_pub    = nh.advertise<geometry_msgs::PoseStamped>
                                ("/orb_odom/pose",1);
    ros::Publisher vel_pub    = nh.advertise<geometry_msgs::TwistStamped>
                                ("/orb_odom/velosity",1);
    
    gazebo_msgs::GetModelState objstate;
    objstate.request.model_name = "iris";
    while (ros::ok())
    {
        if(client.call(objstate) && !first_call)
        {
            msg.header                = objstate.response.header;
            msg.header.frame_id       = "world";
            msg.child_frame_id        = "base_link";
            msg.pose.pose.position.x  = objstate.response.pose.position.x - init_state.pose.pose.position.x;
            msg.pose.pose.position.y  = objstate.response.pose.position.y - init_state.pose.pose.position.y;
            msg.pose.pose.position.z  = objstate.response.pose.position.z - init_state.pose.pose.position.z;
            msg.pose.pose.orientation = objstate.response.pose.orientation;
            msg.twist.twist           = objstate.response.twist;
            msg_vel.twist             = objstate.response.twist;
            msg_vel.header            = objstate.response.header;
            msg_vel.header.frame_id   = "map";
            msg_pos.pose              = msg.pose.pose;
            msg_pos.header            = objstate.response.header;
            msg_pos.header.frame_id   = "map";
        }        
        else{
            init_state.pose.pose   = objstate.response.pose;
            init_state.twist.twist = objstate.response.twist;

            first_call = false;
        }
        fsm_pub.publish(msg);
        pos_pub.publish(msg_pos);
        vel_pub.publish(msg_vel);
    }
    return 0;
}
