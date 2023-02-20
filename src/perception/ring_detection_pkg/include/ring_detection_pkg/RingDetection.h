#ifndef _RING_DETECTION_H_
#define _RING_DETECTION_H_

/* INCLUDE */
#include <iostream>

/* ROS headers */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* PCL headers */
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* --- Kd树构建 --- */
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>

using namespace std;

class RingDetection
{
    private:

        double ring_radius_g;
        double max_inner_error_g;
        double min_inner_rate_g;
        int max_iter_time_g;
        double x_max, x_min, y_max, y_min, z_max, z_min;
        double DBSCAN_search_radius;
        int DBSCAN_min_nbrs_num;
        int DBSCAN_min_total_num;

        ros::NodeHandle nh;
        ros::Subscriber cloud_suber;
        void cloud_subCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
        ros::Publisher test_cluster_puber;
        ros::Publisher test_result_markers_puber;

        std::vector<double> recognize_ring(pcl::PointCloud<pcl::PointXYZ> cluster_cloud, double radius, double max_inner_error, double min_inner_rate, int max_iter_time);

        std::vector<std::vector<int>> DBSCAN_cluster(pcl::PointCloud<pcl::PointXYZ>& origin_cloud, double radius, int min_nbrs_num, int min_total_num, std::vector<double> area);
    public:
        pcl::PointCloud<pcl::PointXYZ> env_cloud;   // cloud for detection

        void RingDetectionLoop(ros::NodeHandle& n);
};

#endif // !_RING_DETECTION_H