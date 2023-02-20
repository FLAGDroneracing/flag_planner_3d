#include "perception_pkg/RingDetection.h"

/*************************************************
 * 圆环检测函数
 *      输入：
 *          环境点云
 *      输出：
 *          圆心位置、半径、法向量
 **************************************************/
std::vector<Eigen::VectorXf> RingDetection::detect_ring(pcl::PointCloud<pcl::PointXYZ> env_cloud)
{    
    std::vector<Eigen::VectorXf> rings_result;
    
    /* 环境点云聚类分割 */
    /* --- 限定点云范围 --- */
    pcl::PointCloud<pcl::PointXYZ>::Ptr D_area_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < env_cloud.points.size(); i++)
    {
        if (
            env_cloud.points[i].x < D_x_max && env_cloud.points[i].x > D_x_min &&
            env_cloud.points[i].y < D_y_max && env_cloud.points[i].y > D_y_min &&
            env_cloud.points[i].z < D_z_max && env_cloud.points[i].z > D_z_min
        )
        {
            D_area_cloud->points.push_back(env_cloud.points[i]);
        }
    }
    /* --- 构建Kd树 --- */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr D_area_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    D_area_kdtree->setInputCloud(D_area_cloud);
    /* --- DBSCAN聚类 --- */
    double search_radius = 0.20;    // 搜索范围
    int min_neighbors_num = 5;  // 内点最少邻居数
    std::vector<int> neighbor_indexs;   // 邻居的序列号
    std::vector<float> neighbor_distances;  // 邻点距离
    std::vector<std::vector<int>> clusters_indexs;  // 聚类后的序列号簇
    std::vector<int> point_types(D_area_cloud->points.size(), 0);  // 点的状态：未处理0；处理中1；已处理2；
    for (int i = 0; i < D_area_cloud->points.size(); i++)
    {
        if (point_types[i] == 2)   continue;   // 若点处理过则跳过
        int nbrs_num = D_area_kdtree->radiusSearch(i, search_radius, neighbor_indexs, neighbor_distances);  // 计算邻点数

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
            nbrs_num = D_area_kdtree->radiusSearch(pt_idx, search_radius, neighbor_indexs, neighbor_distances);    // 搜索seed_queue中点的邻域
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
         
        /* 序列号组合成簇 */
        std::vector<int> cluster_idxs;
        if (seed_queue.size() >= 10)   // 一个簇最少10个点
        {
            for (int j = 0; j < seed_queue.size(); j++)
            {
                cluster_idxs.push_back(seed_queue[j]);
            }
            clusters_indexs.push_back(cluster_idxs);
        }
    }
    /* --- 【测试】点云输出 --- */
    sensor_msgs::PointCloud2 debug_cloud;
    pcl::PointCloud<pcl::PointXYZI> debug_clusters;
    // pcl::toROSMsg(*D_area_cloud, debug_cloud);
    // debug_cloud.header.frame_id = "/world";
    // debug_cloud_puber.publish(debug_cloud);

    /* 对每个簇进行RANSAC滤波 */
    for (int i = 0; i < clusters_indexs.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 聚类后的点云簇
        for (int j = 0; j < clusters_indexs[i].size(); j++)
        {
            cluster_cloud->points.push_back(D_area_cloud->points[clusters_indexs[i][j]]);
            pcl::PointXYZI pt_temp;
            pt_temp.x = D_area_cloud->points[clusters_indexs[i][j]].x;
            pt_temp.y = D_area_cloud->points[clusters_indexs[i][j]].y;
            pt_temp.z = D_area_cloud->points[clusters_indexs[i][j]].z;
            pt_temp.intensity = i%8;
            debug_clusters.points.push_back(pt_temp);
        }
        
        /* RANSAC滤波 */
        Eigen::VectorXf ring_param;
        pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr ring_model( new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cluster_cloud) );   // 选择拟合的模型与点云
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_solver(ring_model);
        ransac_solver.setDistanceThreshold(dist_threshold);
        ransac_solver.setMaxIterations(max_iteration_times);
        ransac_solver.computeModel();
        std::vector<int> inner_pt_idxs; // 内点索引集
        ransac_solver.getInliers(inner_pt_idxs);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ring_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointXYZ>(*cluster_cloud, inner_pt_idxs, *ring_cloud);
cout<<"inner_rate:"<<ring_cloud->points.size()/cluster_cloud->points.size()<<endl;
        //if (ring_cloud->points.size()/cluster_cloud->points.size()>0.5 && ring_cloud->points.size()>=5)
        //{
            ransac_solver.getModelCoefficients(ring_param);
            rings_result.push_back(ring_param);
        //}
    }

    pcl::toROSMsg(debug_clusters, debug_cloud);
    debug_cloud.header.frame_id = "/world";
    debug_cloud_puber.publish(debug_cloud);
    
    return rings_result;
}

/* 环境点云订阅回调函数 */
void RingDetection::env_cloud_subCallback(const sensor_msgs::PointCloud2ConstPtr &received_cloud)
{
    pcl::fromROSMsg(*received_cloud, environment_cloud);    

    /* 处理操作 */
    /* --- D区域处理操作 --- */
    std::vector<Eigen::VectorXf> rings_parameters;
    rings_parameters = detect_ring(environment_cloud);
    /* --- 滤波处理操作 --- */
    if (update_times<200)
    {
        update_times++;
    }
    else
    {
        update_times = 0;
        filtered_rings_information.clear();
        filtering_rings_information.clear();
    }
    for (int i = 0; i < rings_parameters.size(); i++)
    {
        /* 如果参数处于允许范围内 */
        if ( 
            pow(rings_parameters[i](3)-radius_real, 2)<acceptable_error*acceptable_error &&
            rings_parameters[i](0)<D_x_max && rings_parameters[i](0)>D_x_min &&
            rings_parameters[i](1)<D_y_max && rings_parameters[i](1)>D_y_min &&
            rings_parameters[i](2)<D_z_max && rings_parameters[i](2)>D_z_min
        )
        {
            /* 遍历确定是否检测到过 */
            bool already_exist = false;
            for (int j = 0; j < filtered_rings_information.size(); j++)
            {
                /* 如果与之前的位置重合 */
                if ( pow(rings_parameters[i](0)-filtered_rings_information[j][0], 2)+pow(rings_parameters[i](1)-filtered_rings_information[j][1], 2)+pow(rings_parameters[i](2)-filtered_rings_information[j][2], 2)<0.5*0.5 )
                {
                    /* 如果多于10次检测出队较早的检测 */
                    if (filtering_rings_information[j].size()>=10)
                    {
                        filtering_rings_information[j].pop_front();
                    }
		            filtering_rings_information[j].push_back(rings_parameters[i]);

                    /* 计算均值求和 */
                    double ring_cx=0, ring_cy=0, ring_cz=0, ring_vx=0, ring_vy=0, ring_vz=0;
                    for (int k = 0; k < filtering_rings_information[j].size(); k++)
                    {
                        ring_cx = ring_cx + filtering_rings_information[j][k](0);
                        ring_cy = ring_cy + filtering_rings_information[j][k](1);
                        ring_cz = ring_cz + filtering_rings_information[j][k](2);
                        ring_vx = ring_vx + filtering_rings_information[j][k](4);
                        ring_vy = ring_vy + filtering_rings_information[j][k](5);
                        ring_vz = ring_vz + filtering_rings_information[j][k](6);
                    }
                    filtered_rings_information[j][0] = ring_cx/filtering_rings_information[j].size();
if(filtered_rings_information[j][0]>D_x_max)
cout<<"fuck----------------"<<filtering_rings_information[j].size()<<endl;
                    filtered_rings_information[j][1] = ring_cy/filtering_rings_information[j].size();
                    filtered_rings_information[j][2] = ring_cz/filtering_rings_information[j].size();
                    filtered_rings_information[j][3] = ring_vx/filtering_rings_information[j].size();
                    filtered_rings_information[j][4] = ring_vy/filtering_rings_information[j].size();
                    filtered_rings_information[j][5] = ring_vz/filtering_rings_information[j].size();

                    already_exist = true;
                }  

if ( pow(rings_parameters[i](0)-filtered_rings_information[j][0], 2)+pow(rings_parameters[i](1)-filtered_rings_information[j][1], 2)+pow(rings_parameters[i](2)-filtered_rings_information[j][2], 2)>0.5*0.5 && pow(rings_parameters[i](0)-filtered_rings_information[j][0], 2)+pow(rings_parameters[i](1)-filtered_rings_information[j][1], 2)+pow(rings_parameters[i](2)-filtered_rings_information[j][2], 2)<1.5*1.5)
already_exist = true;
            }

            /* 如果未被检测过 */
            if (!already_exist)
            {
                std::deque<Eigen::VectorXf> new_ring_information;
                new_ring_information.push_back(rings_parameters[i]);
                filtering_rings_information.push_back(new_ring_information);
                std::vector<double> new_filtered_information;
                new_filtered_information.resize(6);
                new_filtered_information[0] = rings_parameters[i](0);
                new_filtered_information[1] = rings_parameters[i](1);
                new_filtered_information[2] = rings_parameters[i](2);
                new_filtered_information[3] = rings_parameters[i](4);
                new_filtered_information[4] = rings_parameters[i](5);
                new_filtered_information[5] = rings_parameters[i](6);
                filtered_rings_information.push_back(new_filtered_information);
            }
        }
    }
    /* --- 发布与可视化 --- */
    visualization_msgs::MarkerArray rings_markers;
    std::vector<geometry_msgs::PoseStamped> rings_msg_data;     // 存放目标点
     for (int i = 0; i < filtered_rings_information.size(); i++)
    {
        /* 如果多于 次观测到 */
        if (filtering_rings_information[i].size()>3)
        {
            visualization_msgs::Marker ring_center_marker;
            ring_center_marker.header.frame_id = "/world";
            ring_center_marker.ns = "ring_center";
            ring_center_marker.action = visualization_msgs::Marker::ADD;
            ring_center_marker.id = i;
            ring_center_marker.type = visualization_msgs::Marker::CUBE;
            ring_center_marker.scale.x = 0.15;
            ring_center_marker.scale.y = 0.15;
            ring_center_marker.scale.z = 0.15;
            ring_center_marker.pose.position.x = filtered_rings_information[i][0];
if(filtered_rings_information[i][0]>D_x_max)cout<<"fuck"<<endl;
            ring_center_marker.pose.position.y = filtered_rings_information[i][1];
            ring_center_marker.pose.position.z = filtered_rings_information[i][2];
            ring_center_marker.pose.orientation.w = 1.0;
            ring_center_marker.pose.orientation.x = 0.0;
            ring_center_marker.pose.orientation.y = 0.0;
            ring_center_marker.pose.orientation.z = 0.0;
            ring_center_marker.color.a = 1;
            ring_center_marker.color.r = 0;
            ring_center_marker.color.g = 255;
            ring_center_marker.color.b = 0;         
            rings_markers.markers.push_back(ring_center_marker);

            /* 目标点入队 */
            geometry_msgs::PoseStamped test_ring_msg;
            test_ring_msg.pose.position.x = filtered_rings_information[i][0];
            test_ring_msg.pose.position.y = filtered_rings_information[i][1];
            test_ring_msg.pose.position.z = filtered_rings_information[i][2];
            if (filtered_rings_information[i][3]>0)
            {
                test_ring_msg.pose.orientation.x = filtered_rings_information[i][3];
                test_ring_msg.pose.orientation.y = filtered_rings_information[i][4];
                test_ring_msg.pose.orientation.z = filtered_rings_information[i][5];
            }
            else
            {
                test_ring_msg.pose.orientation.x = -filtered_rings_information[i][3];
                test_ring_msg.pose.orientation.y = -filtered_rings_information[i][4];
                test_ring_msg.pose.orientation.z = -filtered_rings_information[i][5];
            }
            rings_msg_data.push_back(test_ring_msg);
        }
    }
cout<<"markers_size:"<<rings_markers.markers.size()<<endl;
    ring_center_test_puber.publish(rings_markers);

    /* 目标点按距离排序 */
    geometry_msgs::PoseStamped sorting_ring_data;
    for (int i = 0; i < rings_msg_data.size(); i++)
    {
        for (int j = i+1; j < rings_msg_data.size(); j++)
        {
            if (rings_msg_data[i].pose.position.x>rings_msg_data[j].pose.position.x)
            {
                sorting_ring_data = rings_msg_data[i];
                rings_msg_data[i] = rings_msg_data[j];
                rings_msg_data[j] = sorting_ring_data;
            }
        }
    }
    

    std_msgs::Float64MultiArray test_ring_msg;
    test_ring_msg.data.push_back(rings_msg_data.size());
    for (int i = 0; i < rings_msg_data.size(); i++)
    {
        test_ring_msg.data.push_back(rings_msg_data[i].pose.position.x);
        test_ring_msg.data.push_back(rings_msg_data[i].pose.position.y);
        test_ring_msg.data.push_back(rings_msg_data[i].pose.position.z);
        test_ring_msg.data.push_back(rings_msg_data[i].pose.orientation.x);
        test_ring_msg.data.push_back(rings_msg_data[i].pose.orientation.y);
        test_ring_msg.data.push_back(rings_msg_data[i].pose.orientation.z);
    }
    ring_msg_test_puber.publish(test_ring_msg);    
}

/* 流程函数 */
void RingDetection::ring_detection_loop(ros::NodeHandle n)
{
    nh = n;

    /* 参数初始化 */
    nh.param<double>("D_x_max", D_x_max, 2.0);
    nh.param<double>("D_x_min", D_x_min, 0.5);
    nh.param<double>("D_y_max", D_y_max, 2.0);
    nh.param<double>("D_y_min", D_y_min, -2.0);
    nh.param<double>("D_z_max", D_z_max, 2.0);
    nh.param<double>("D_z_min", D_z_min, 0.2);
    nh.param<double>("dist_threshold", dist_threshold, 0.3);
    nh.param<int>("max_iteration", max_iteration_times, 3000);
    nh.param<double>("radius_real", radius_real, 0.5);
    nh.param<double>("acceptable_error", acceptable_error, 0.3);
    update_times = 0;

    /* 初始化订阅者与发布者 */
    env_cloud_suber = nh.subscribe<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10, &RingDetection::env_cloud_subCallback, this);
    ring_center_test_puber = nh.advertise<visualization_msgs::MarkerArray>("/ring_center_test", 10);
    ring_msg_test_puber = nh.advertise<std_msgs::Float64MultiArray>("/test_ring_msg", 10);
    debug_cloud_puber = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 10);
}
