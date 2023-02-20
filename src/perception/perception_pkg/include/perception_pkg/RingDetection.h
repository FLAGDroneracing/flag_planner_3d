#ifndef _RING_DETECTION_H_
#define _RING_DETECTION_H_

/* INCLUDE */
#include <iostream>
#include <deque>
#include <vector>   

/* ROS头文件 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

/* PCL头文件 */
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* --- RANSAC滤波 --- */
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
/* --- Kd树构建 --- */
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>

/* Eigen头文件 */
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class RingDetection
{
    private:
        ros::NodeHandle nh;

        /* 处理函数 */
        void process_task_D();

        /* 圆环探测 */
        double D_x_max, D_x_min, D_y_max, D_y_min, D_z_max, D_z_min;
        double dist_threshold;  // 距离阈值
        int max_iteration_times;    // 迭代次数
        std::vector<Eigen::VectorXf> detect_ring(pcl::PointCloud<pcl::PointXYZ> env_cloud);

        /* 滤波处理 */
        std::vector<std::vector<double>> filtered_rings_information;    // [圆环序号[圆心+法向量]]
        std::vector<std::deque<Eigen::VectorXf>> filtering_rings_information;  // [圆环序号[检测次数号]]
        double radius_real;
        double acceptable_error;
        int update_times;

        /* 环境点云订阅 */
        ros::Subscriber env_cloud_suber;
        pcl::PointCloud<pcl::PointXYZ> environment_cloud;   // 全局环境点云
        void env_cloud_subCallback(const sensor_msgs::PointCloud2ConstPtr &received_cloud);

        /* 【测试】结果发布 */
        ros::Publisher ring_center_test_puber;
        ros::Publisher ring_msg_test_puber;
        ros::Publisher debug_cloud_puber;
    public:
        void ring_detection_loop(ros::NodeHandle n);
};

#endif // !_RING_DETECTION_H_


