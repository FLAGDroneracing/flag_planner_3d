#include "perception_pkg/TubeDetection.h"

/* 标志位接收 */
void TubeDetection::flag_subCallback(const std_msgs::Int64ConstPtr &msg)
{
    if (msg->data == 2)
    {
        in_process = true;
    }
    else
    {
        in_process = false;
    }
}

/* 包围盒处理函数 */
std::vector<double> TubeDetection::task_B_processing(pcl::PointCloud<pcl::PointXYZ> cloud, std::vector<double> area_contours)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_area_B(new pcl::PointCloud<pcl::PointXYZ>);   // A区内的点
    std::vector<double> task_B_result;

    /* 裁剪原始点云只保留B区部分 */
    for (int i = 0; i < cloud.points.size(); i++)
    {
        if (cloud.points[i].x>area_contours[0] && cloud.points[i].x<area_contours[1] && 
             cloud.points[i].y>area_contours[2] && cloud.points[i].y<area_contours[3] &&
             cloud.points[i].z>area_contours[4] && cloud.points[i].z<area_contours[5] 
        )
        {
            pcl::PointXYZ pt;
            pt.x = cloud.points[i].x;
            pt.y = cloud.points[i].y;
            pt.z = cloud.points[i].z;
            pts_in_area_B->points.push_back(pt);
        }        
    }

    /* DBSCAN聚类 */
    /* --- 构建Kd树 --- */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr B_area_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    B_area_kdtree->setInputCloud(pts_in_area_B);
    /* --- 聚类 --- */
    std::vector<int> neighbor_indexs;   // 邻居的序列号
    std::vector<float> neighbor_distances;  // 邻点距离
    std::vector<int> largest_cluster_indexs;  // 聚类后的序列号簇
    std::vector<int> point_types(pts_in_area_B->points.size(), 0);  // 点的状态：未处理0；处理中1；已处理2；
    for (int i = 0; i < pts_in_area_B->points.size(); i++)
    {
        if (point_types[i] == 2)   continue;   // 若点处理过则跳过
        int nbrs_num = B_area_kdtree->radiusSearch(i, search_radius, neighbor_indexs, neighbor_distances);  // 计算邻点数

        /* 将中心点与邻域的点放入队列 */
        std::vector<int> seed_queue;
        seed_queue.push_back(i);    // 中心点放入
        point_types[i] = 2;
        for (int j = 0; j < nbrs_num; j++)
        {
            if (neighbor_indexs[j] != i)    // 邻点入队处理
            {
                seed_queue.push_back(neighbor_indexs[j]);
                point_types[neighbor_indexs[j]] = 1;
            }
        }
         
         /* 遍历队列中的成员 */
         int sq_idx = 1;    // seed_queue中的序列号
         while (sq_idx < seed_queue.size())
         {
            int pt_idx = seed_queue[sq_idx];
            if (point_types[pt_idx] == 2)  // 处理过则跳过
            {
                sq_idx ++;
                continue;
            }
            neighbor_indexs.clear();
            neighbor_distances.clear();
            nbrs_num = B_area_kdtree->radiusSearch(pt_idx, search_radius, neighbor_indexs, neighbor_distances);    // 搜索seed_queue中点的邻域
            if (nbrs_num > min_neighbors_num)  // 如果是内殿
            {
                for (int j = 0; j < nbrs_num; j++) // 内点的邻居放入seed_queue
                {
                    if (point_types[neighbor_indexs[j]] == 0)  // 若未处理过
                    {
                        seed_queue.push_back(neighbor_indexs[j]);
                        point_types[neighbor_indexs[j]] = 1;
                    }
                }
            }
            point_types[pt_idx] = 2;   // seed_queue中该点处理过
            sq_idx++;
        }
         
        /* 找到最大簇的序号 */
        std::vector<int> cluster_idxs;
        if (seed_queue.size() >= 20)   // 一个簇最少10个点
        {
            for (int j = 0; j < seed_queue.size(); j++)
            {
                cluster_idxs.push_back(seed_queue[j]);
            }
            if (cluster_idxs.size()>largest_cluster_indexs.size())
            {
                largest_cluster_indexs = cluster_idxs;
            }
        }
    } 
    /* --- 最大的点云簇 --- */ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    for (int i = 0; i < largest_cluster_indexs.size(); i++)
    {
        box_cloud->points.push_back(pts_in_area_B->points[largest_cluster_indexs[i]]);
    }
    
    /* 计算AABB包围盒 */
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> box_extractor;
    box_extractor.setInputCloud(box_cloud);
    box_extractor.compute();
    std::vector <float> moment_of_inertia;  
    std::vector <float> eccentricity;  
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    box_extractor.getMomentOfInertia(moment_of_inertia);
    box_extractor.getEccentricity(eccentricity);
    box_extractor.getAABB(min_point_AABB,max_point_AABB);

    /* debug */
    visualization_msgs::Marker marker_result;
    marker_result.header.frame_id = "/world";
    marker_result.id = 100;
    marker_result.type = visualization_msgs::Marker::CUBE;
    marker_result.action = visualization_msgs::Marker::ADD;
    marker_result.pose.orientation.x = 0;
    marker_result.pose.orientation.y = 0;
    marker_result.pose.orientation.z = 0;
    marker_result.pose.orientation.w = 1;
    marker_result.pose.position.x = (min_point_AABB.x + max_point_AABB.x)/2.0;
    marker_result.pose.position.y = (min_point_AABB.y + max_point_AABB.y)/2.0;
    marker_result.pose.position.z = (min_point_AABB.z + max_point_AABB.z)/2.0;
    marker_result.scale.x = max_point_AABB.x - min_point_AABB.x;
    marker_result.scale.y = max_point_AABB.y - min_point_AABB.y;
    marker_result.scale.z = max_point_AABB.z - min_point_AABB.z;
    marker_result.color.r = 1.0;
    marker_result.color.g = 0.0;
    marker_result.color.b = 0.0;
    marker_result.color.a = 0.5;
    debug_puber.publish(marker_result);

    /* 输出结果 */
    double point2_x = min_point_AABB.x - 0.5;
    double point2_y = (min_point_AABB.y + max_point_AABB.y)/2.0;
    double point2_z = 1.75;
    double width = max_point_AABB.y - min_point_AABB.y; 
    task_B_result.push_back(point2_x);
    task_B_result.push_back(point2_y);
    task_B_result.push_back(point2_z);
    task_B_result.push_back(width);
    return task_B_result;
}

void TubeDetection::cloud_map_subCallback(const sensor_msgs::PointCloud2ConstPtr &received_cloud)
{
    if (in_process == true)
    {
        std::cout << "IN PROCESS!" << std::endl;
         pcl::PointCloud<pcl::PointXYZ>  environment_cloud;
        pcl::fromROSMsg(*received_cloud, environment_cloud);  
        std::vector<double> area_B_contours = {B_x_min, B_x_max, B_y_min, B_y_max, B_z_min, B_z_max};
        std::vector<double> task_B_result;
        task_B_result = task_B_processing(environment_cloud, area_B_contours);
        if (task_B_result[3] < max_width && task_B_result[3]>min_width)
        {
        geometry_msgs::PoseStamped task_B_msg;
            task_B_msg.header.frame_id = "/world";
            task_B_msg.pose.position.x = task_B_result[0];
            task_B_msg.pose.position.y = task_B_result[1];
            task_B_msg.pose.position.z = task_B_result[2];
            task_B_puber.publish(task_B_msg);
        }
    }
    else
    {
        std::cout << "EMPTY!" << std::endl;
    }
}

/* 流程函数 */
void TubeDetection::TubeDetectionLoop(ros::NodeHandle n)
{
    nh = n;

    /* 参数初始化 */
    nh.param<double>("B_x_max", B_x_max, 2.0);
    nh.param<double>("B_x_min", B_x_min, 0.5);
    nh.param<double>("B_y_max", B_y_max, 2.0);
    nh.param<double>("B_y_min", B_y_min, -2.0);
    nh.param<double>("B_z_max", B_z_max, 2.0);
    nh.param<double>("B_z_min", B_z_min, 0.2);
    nh.param<double>("search_radius", search_radius, 0.3);
    nh.param<int>("min_neighbors_num", min_neighbors_num, 5);
	nh.param<double>("max_width", max_width, 1.75);
    nh.param<double>("min_width", min_width, 1.25);

    /* 初始化订阅者与发布者 */
    cloud_map_suber = nh.subscribe<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10, &TubeDetection::cloud_map_subCallback, this);
    task_B_puber = nh.advertise<geometry_msgs::PoseStamped>("/task_B_msg", 1);
    debug_puber = nh.advertise<visualization_msgs::Marker>("/debug_msg", 1);
    flag_suber = nh.subscribe<std_msgs::Int64>("/flag_detect", 10, &TubeDetection::flag_subCallback, this);
}
