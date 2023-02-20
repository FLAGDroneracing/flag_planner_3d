#include "ring_detection_pkg/RingDetection.h"

/*************************************************
 * DBSCAN聚类
 *  输入：
 *      原始点云；
 *      邻居搜索半径；
 *      最小邻点数；
 *      最小簇要求；
 *      分割区域；
 *  输出：
 *      各个簇的点序号；
 *************************************************/
std::vector<std::vector<int>> RingDetection::DBSCAN_cluster(pcl::PointCloud<pcl::PointXYZ>& origin_cloud, double radius, int min_nbrs_num, int min_total_num, std::vector<double> area)
{
    std::vector<std::vector<int>> clusters_indexs;  // 聚类后的序列号簇

    /* 限定点云范围 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr area_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < origin_cloud.points.size(); i++)
    {
        if (
            origin_cloud.points[i].x < area[0] && origin_cloud.points[i].x > area[1] &&
            origin_cloud.points[i].y < area[2] && origin_cloud.points[i].y >area[3] &&
            origin_cloud.points[i].z < area[4] && origin_cloud.points[i].z > area[5]
        )
        {
            area_cloud->points.push_back(origin_cloud.points[i]);
        }
    }

    std::cout << "area_cloud_size:" << area_cloud->points.size() << std::endl;

    /* 构建Kd树 */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr area_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    area_kdtree->setInputCloud(area_cloud);

    /* DBSCAN聚类 */
    std::vector<int> neighbor_indexs;   // 邻居的序列号
    std::vector<float> neighbor_distances;  // 邻点距离
    std::vector<int> point_types(area_cloud->points.size(), 0);  // 点的状态：未处理0；处理中1；已处理2；
    for (int i = 0; i < area_cloud->points.size(); i++)
    {
        if (point_types[i] == 2)   continue;   // 若点处理过则跳过
        int nbrs_num = area_kdtree->radiusSearch(i, radius, neighbor_indexs, neighbor_distances);  // 计算邻点数

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
            nbrs_num = area_kdtree->radiusSearch(pt_idx, radius, neighbor_indexs, neighbor_distances);    // 搜索seed_queue中点的邻域
            if (nbrs_num > min_nbrs_num)  // 如果是内点
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
        if (seed_queue.size() >= min_total_num)   
        {
            for (int j = 0; j < seed_queue.size(); j++)
            {
                cluster_idxs.push_back(seed_queue[j]);
            }
            clusters_indexs.push_back(cluster_idxs);
        }
    }

    env_cloud.points.clear();
    for (int i = 0; i < area_cloud->points.size(); i++)
    {
        env_cloud.points.push_back(area_cloud->points[i]);
    }
    
    return clusters_indexs;
}

/*************************************************
 * RANSAC识别圆环
 *  输入：
 *      待识别的点云簇；
 *      已知圆环的半径；
 *      内点最大偏差；
 *      内点率阈值；
 *      最大迭代次数；
 *  输出：
 *      是否识别成功：0失败，1成功；
 *      圆心的x、y、z坐标；
 *      法向量的x、y、z坐标；
 *************************************************/
std::vector<double> RingDetection::recognize_ring(pcl::PointCloud<pcl::PointXYZ> cluster_cloud, double radius, double max_inner_error, double min_inner_rate, int max_iter_time)
{
    std::vector<double> recognize_result;
    recognize_result.resize(7, 0.0);

    /* RANSAC逐次迭代 */
    int pts_num = cluster_cloud.points.size();
    int update_time = 0;
    double best_inner_rate = 0.0;   // 最好的内点率
    std::vector<double> best_param; // 最好的参数
    best_param.resize(6, 0.0);
    std::cout << "max_iter_time:" << max_iter_time << std::endl;
    while (update_time < max_iter_time)
    {
        /* 随机选择三个点的索引 */
        srand(update_time);
        int pt_idx_0 = rand() % pts_num;
        int pt_idx_1 = rand() % pts_num;
        int pt_idx_2 = rand() % pts_num;
        // std::cout << "pt_idx" << pt_idx_0 << "," << pt_idx_1 << "," << pt_idx_2 << std::endl;
        update_time++;

        /* 计算本次迭代的圆心 */
        if (pt_idx_0 == pt_idx_1 || pt_idx_0 == pt_idx_2 || pt_idx_1 == pt_idx_2) continue;
        double vec_0_x, vec_0_y, vec_0_z, vec_1_x, vec_1_y, vec_1_z, vec_2_x, vec_2_y, vec_2_z;    // 两点连线的指向向量
        vec_0_x = cluster_cloud.points[pt_idx_0].x - cluster_cloud.points[pt_idx_1].x;
        vec_0_y = cluster_cloud.points[pt_idx_0].y - cluster_cloud.points[pt_idx_1].y;
        vec_0_z = cluster_cloud.points[pt_idx_0].z - cluster_cloud.points[pt_idx_1].z;
        vec_1_x = cluster_cloud.points[pt_idx_0].x - cluster_cloud.points[pt_idx_2].x;
        vec_1_y = cluster_cloud.points[pt_idx_0].y - cluster_cloud.points[pt_idx_2].y;
        vec_1_z = cluster_cloud.points[pt_idx_0].z - cluster_cloud.points[pt_idx_2].z;
        vec_2_x = cluster_cloud.points[pt_idx_1].x - cluster_cloud.points[pt_idx_2].x;
        vec_2_y = cluster_cloud.points[pt_idx_1].y - cluster_cloud.points[pt_idx_2].y;
        vec_2_z = cluster_cloud.points[pt_idx_1].z - cluster_cloud.points[pt_idx_2].z;
        /* --- 找到点1和点2的垂直平分线 --- */
        double lamda;   // vec_0与vec_1的组合系数
        double vec_mp_x, vec_mp_y, vec_mp_z;    // 垂直平分线向量
        if ( abs(vec_1_x * vec_2_x + vec_1_y * vec_2_y + vec_1_z * vec_2_z) > 0.0 )
        {
            lamda = -(vec_0_x * vec_1_x + vec_0_y * vec_1_y + vec_0_z * vec_1_z) / (vec_1_x * vec_2_x + vec_1_y * vec_2_y + vec_1_z * vec_2_z);
            vec_mp_x = vec_0_x + lamda * vec_1_x;
            vec_mp_y = vec_0_y + lamda * vec_1_y;
            vec_mp_z = vec_0_z + lamda * vec_1_z;
        }
        else
        {
            vec_mp_x = vec_1_x;
            vec_mp_y = vec_1_y;
            vec_mp_z = vec_1_z;
        }
        double middle_pt_x, middle_pt_y, middle_pt_z;   // 中点坐标
        middle_pt_x = (cluster_cloud.points[pt_idx_1].x + cluster_cloud.points[pt_idx_2].x) / 2.0;
        middle_pt_y = (cluster_cloud.points[pt_idx_1].y + cluster_cloud.points[pt_idx_2].y) / 2.0;
        middle_pt_z = (cluster_cloud.points[pt_idx_1].z + cluster_cloud.points[pt_idx_2].z) / 2.0;
        /* --- 找到圆心及法向量 --- */
        double pt1_mid_dist = sqrt( pow(cluster_cloud.points[pt_idx_1].x - middle_pt_x, 2) + pow(cluster_cloud.points[pt_idx_1].y - middle_pt_y, 2) + pow(cluster_cloud.points[pt_idx_1].z - middle_pt_z, 2) );
        double angle = acos(pt1_mid_dist / radius); // 1点处的圆周角
        double p_length = sin(angle) * radius;
        double center_x, center_y, center_z;    // 圆心坐标
        double center_x_0, center_y_0, center_z_0, center_x_1, center_y_1, center_z_1;
        center_x_0 = middle_pt_x + p_length * vec_mp_x / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        center_y_0 = middle_pt_y + p_length * vec_mp_y / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        center_z_0 = middle_pt_z + p_length * vec_mp_z / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        center_x_1 = middle_pt_x + p_length * vec_mp_x / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        center_y_1 = middle_pt_y + p_length * vec_mp_y / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        center_z_1 = middle_pt_z + p_length * vec_mp_z / sqrt(vec_mp_x * vec_mp_x + vec_mp_y * vec_mp_y + vec_mp_z * vec_mp_z);
        double form_vec_x, form_vec_y, form_vec_z;  // 法向量
        form_vec_x = vec_0_y * vec_1_z - vec_1_y * vec_0_z;
        form_vec_y = vec_1_x * vec_0_z - vec_0_x * vec_1_z;
        form_vec_z = vec_0_x * vec_1_y - vec_1_x * vec_0_y;
        /* --- 根据0点进行判优 --- */
        double dist2_0 = pow(center_x_0 - cluster_cloud.points[pt_idx_0].x, 2) + pow(center_y_0 - cluster_cloud.points[pt_idx_0].y, 2) + pow(center_z_0 - cluster_cloud.points[pt_idx_0].z, 2);
        double dist2_1 = pow(center_x_1 - cluster_cloud.points[pt_idx_0].x, 2) + pow(center_y_1 - cluster_cloud.points[pt_idx_0].y, 2) + pow(center_z_1 - cluster_cloud.points[pt_idx_0].z, 2);
        if ( abs(dist2_0 - radius) < abs(dist2_1 - radius) )
        {
            center_x = center_x_0;
            center_y = center_y_0;
            center_z = center_z_0;
        }
        else
        {
            center_x = center_x_1;
            center_y = center_y_1;
            center_z = center_z_1;
        }
        
        /* 计算内点率 */
        int inner_pts_num = 0;
        for (int i = 0; i < pts_num; i++)
        {
            /* 计算该点到圆环平面的投影 */
            double proj_pt_x, proj_pt_y, proj_pt_z; // 投影点的坐标
            double t = ( form_vec_x * (cluster_cloud.points[pt_idx_0].x - cluster_cloud.points[i].x) + form_vec_y * (cluster_cloud.points[pt_idx_0].y - cluster_cloud.points[i].y) + form_vec_z * (cluster_cloud.points[pt_idx_0].z - cluster_cloud.points[i].z) ) / (form_vec_x * form_vec_x + form_vec_y * form_vec_y + form_vec_z * form_vec_z);
            proj_pt_x = cluster_cloud.points[i].x + form_vec_x * t;
            proj_pt_y = cluster_cloud.points[i].y + form_vec_y * t;
            proj_pt_z = cluster_cloud.points[i].z + form_vec_z * t;

            /* 计算投影点的偏差 */
            double center_error = abs( sqrt( pow(proj_pt_x - center_x, 2) + pow(proj_pt_y - center_y, 2) + pow(proj_pt_z - center_z, 2) ) - radius );
            double plane_error2 = pow(proj_pt_x - cluster_cloud.points[i].x, 2) + pow(proj_pt_y - cluster_cloud.points[i].y, 2) + pow(proj_pt_z - cluster_cloud.points[i].z, 2);
            double total_error2 = center_error * center_error + plane_error2;

            /* 判断是否为内点 */
            if (total_error2 < max_inner_error * max_inner_error)
            {
                inner_pts_num = inner_pts_num + 1;
            }
        }
        
        /* 本次是否为最优结果 */
        double inner_rate = (double)inner_pts_num / (double)pts_num;
        if (inner_rate > best_inner_rate)
        {
            best_inner_rate = inner_rate;
            best_param[0] = center_x;
            best_param[1] = center_y;
            best_param[2] = center_z;
            best_param[3] = form_vec_x;
            best_param[4] = form_vec_y;
            best_param[5] = form_vec_z;
        } 

        if (best_inner_rate>min_inner_rate+0.10)
        {
            break;
        }
    }
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:" << std::endl;

    std::cout << "best_params:" << std::endl;
    std::cout << "pos:" << best_param[0] << "," << best_param[1] << best_param[2] << std::endl;
    std::cout << "form_vec" << best_param[3] <<  "," << best_param[4] << "," << best_param[5] << std::endl;
    std::cout << "best_inner_rate:" << best_inner_rate << std::endl;

    if (best_inner_rate > min_inner_rate)
    {
        recognize_result[0] = 1.0;
        for (int i = 1; i < recognize_result.size(); i++)
        {
            recognize_result[i] = best_param[i-1];
        }  
    }
    else
    {
        recognize_result[0] = 0.0;
    }
    
    return recognize_result;
}

void  RingDetection::cloud_subCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, env_cloud);

    std::vector<double> test_area;
    test_area.push_back(x_max);
    test_area.push_back(x_min);
    test_area.push_back(y_max);
    test_area.push_back(y_min);
    test_area.push_back(z_max);
    test_area.push_back(z_min);
    std::cout << "area:" << test_area[0] << ","<< test_area[1] << "," << test_area[2] << "," << test_area[3] << "," << test_area[4] << "," << test_area[5]  << std::endl;
    std::vector<std::vector<int>> cluster_result = DBSCAN_cluster(env_cloud, DBSCAN_search_radius, DBSCAN_min_nbrs_num, DBSCAN_min_total_num, test_area);
    
    pcl::PointCloud<pcl::PointXYZI> debug_cloud;
    pcl::PointCloud<pcl::PointXYZ> result_pts;

    for (int i = 0; i < cluster_result.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> clusted_cloud;
        for (int j = 0; j < cluster_result[i].size(); j++)
        {
            clusted_cloud.points.push_back(env_cloud.points[cluster_result[i][j]]);
            pcl::PointXYZI debug_pt;
            debug_pt.x = env_cloud.points[cluster_result[i][j]].x;
            debug_pt.y = env_cloud.points[cluster_result[i][j]].y;
            debug_pt.z = env_cloud.points[cluster_result[i][j]].z;
            debug_pt.intensity = i % 8;
            debug_cloud.points.push_back(debug_pt);
        }

        std::vector<double> detect_result = recognize_ring(clusted_cloud, ring_radius_g, max_inner_error_g, min_inner_rate_g, max_iter_time_g);

        std::cout << "detect_result:"  << std::endl;
        std::cout << "success:" << detect_result[0] << std::endl;
        std::cout << "pos:" << detect_result[1] << "," << detect_result[2] << "," << detect_result[3] << std::endl;
        std::cout << "norm:" << detect_result[4] << "," << detect_result[5] << "," << detect_result[6] << std::endl;        

        if (detect_result[0] > 0.5)
        {
            pcl::PointXYZ pt_temp;
            pt_temp.x = detect_result[1];
            pt_temp.y = detect_result[2];
            pt_temp.z = detect_result[3];
            result_pts.points.push_back(pt_temp);
        } 
    }

    debug_cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(debug_cloud, debug_cloud_msg);
    test_cluster_puber.publish(debug_cloud_msg);

    result_pts.header.frame_id = "world";
    sensor_msgs::PointCloud2 result_msg;
    pcl::toROSMsg(result_pts, result_msg);
    test_result_markers_puber.publish(result_msg);
}

void RingDetection::RingDetectionLoop(ros::NodeHandle& n)
{
    nh = n;

    nh.param<double>("/ring_detection_pkg/ring_radius", ring_radius_g, 1.25);
    nh.param<double>("/ring_detection_pkg/max_inner_error", max_inner_error_g, 0.30);
    nh.param<double>("/ring_detection_pkg/min_inner_rate", min_inner_rate_g, 0.80);
    nh.param<int>("/ring_detection_pkg/max_iter_time", max_iter_time_g, 3000);

    nh.param<double>("/ring_detection_pkg/x_max", x_max, 10.0);
    nh.param<double>("/ring_detection_pkg/x_min", x_min, -10.0);
    nh.param<double>("/ring_detection_pkg/y_max", y_max, 10.0);
    nh.param<double>("/ring_detection_pkg/y_min", y_min, -10.0);
    nh.param<double>("/ring_detection_pkg/z_max", z_max, 1.0);
    nh.param<double>("/ring_detection_pkg/z_min", z_min, -1.0);

    nh.param<double>("/ring_detection_pkg/search_radius", DBSCAN_search_radius, 0.20);
    nh.param<int>("/ring_detection_pkg/min_nbrs_num", DBSCAN_min_nbrs_num, 4);
    nh.param<int>("/ring_detection_pkg/min_total_num", DBSCAN_min_total_num, 10);
    
    cloud_suber = nh.subscribe<sensor_msgs::PointCloud2>("/grid_map/occupancy", 1, &RingDetection::cloud_subCallback, this);

    test_cluster_puber = nh.advertise<sensor_msgs::PointCloud2>("/test_clusters", 1);
    test_result_markers_puber = nh.advertise<sensor_msgs::PointCloud2>("/test_centroid", 1);
}