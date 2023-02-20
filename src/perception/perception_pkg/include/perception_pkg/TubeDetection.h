#ifndef _TUBE_DETECTION_H_
#define _TUBE_DETECTION_H_

/* INCLUDE */
#include <iostream>
#include <deque>
#include <vector>   

/* ROS头文件 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int64.h>

/* PCL头文件 */
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* --- Kd树构建 --- */
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
/* --- 包围盒构建 --- */
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;

class TubeDetection
{
    private:
        ros::NodeHandle nh;

        bool in_process;    // 进入到本任务

        /* 包围盒处理 */
        double B_x_max, B_x_min, B_y_max, B_y_min, B_z_max, B_z_min;    // B区域范围
double max_width, min_width;
        double search_radius;
        int min_neighbors_num;
        std::vector<double> task_B_processing(pcl::PointCloud<pcl::PointXYZ> cloud, std::vector<double> area_contours);

        /* 消息订阅与发布 */
        ros::Subscriber cloud_map_suber;
        void cloud_map_subCallback(const sensor_msgs::PointCloud2ConstPtr &received_cloud);
        ros::Publisher debug_puber;
        ros::Publisher task_B_puber;
        ros::Subscriber flag_suber;
        void flag_subCallback(const std_msgs::Int64ConstPtr &msg);
    public:
        /* 流程函数 */
        void TubeDetectionLoop(ros::NodeHandle n);
};

#endif // !_TUBE_DETECTION_H_
