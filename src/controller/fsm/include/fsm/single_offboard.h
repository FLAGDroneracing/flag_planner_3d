
#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/types.h>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
//#include <eigen2/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <std_msgs/Float64MultiArray.h>
#include <fsm/command_acc.h>
//#include <bspline_race/bspline_race.h>

#include<bspline_race/BsplineTraj.h>
#include<bspline_race/EdtTransform.hpp>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <vector>
#include <deque>

/* ROS头文件 */
// #include <mavros/mavros.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>

class Flash 
{
    // public: void CmdCallback(const robot_msgs::HostCmdArrayConstPtr &msg);
    // public: void GroupStateCallback(const robot_msgs::RobotStatesConstPtr &msg); 
	// public: void TagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg);
	// public:	void PubDecisionState();
	// public:	void PubMembers();
	// public: void PubTagPose();

    // public: std::vector<geometry_msgs::Pose> GetGoal();//每一个任务可能有多个点的goal
	// public: std::vector<robot_msgs::HostCmd> msgs;//一次发布多个任务
    // public: std::vector<robot_msgs::HostCmd> TaskList;
	// public: std::vector<geometry_msgs::Pose> goal;
	// public:  geometry_msgs::Pose tag_pose;

	public:	void Init();

	 private: ros::Subscriber state_sub;
	 private: ros::Subscriber position_sub;
	 private: ros::Subscriber ready;
	 private: ros::Subscriber voltage_sub;
	 private: ros::Subscriber vel_sub;
	 private: ros::Subscriber box_sub;
	 private: ros::Subscriber ring_sub;
	 private: ros::Subscriber wall_sub;
	 private: ros::Subscriber tunnel_sub;
	 private: ros::Subscriber tag_sub;
	 private: ros::Subscriber traj_sub;
	 private: ros::Subscriber msg_from_cloud_suber;

     public: void state_cb(const mavros_msgs::State::ConstPtr& msg);
     public: void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
     public: void ready_cb(const fsm::command_acc &msg);
     public: void vtg_cb(const sensor_msgs::BatteryState::ConstPtr &msg);
     public: void vel_cb(const geometry_msgs::TwistStamped::ConstPtr & msg);
     public: void box_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
     public: void ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
     public: void wall_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
     public: void tunnel_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
     public: void tag_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
     public: void traj_cb(const bspline_race::BsplineTraj::ConstPtr & msg);
     public: void msg_from_cloud_subCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private: ros::Publisher fsm_pub;
	private: ros::Publisher traj_goal_pub;
	private: ros::Publisher local_pos_pub;
    private: ros::Publisher local_raw_pub;
    private: ros::Publisher bsaim_pub;
};