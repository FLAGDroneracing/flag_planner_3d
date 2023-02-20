#include "RingDetection.cpp"

int main(int argc, char  **argv)
{
    /* 初始化ROS节点 */
    ros::init(argc, argv, "Main_N");
    ros::NodeHandle main_n("~");

    RingDetection ring_detection;

    ring_detection.RingDetectionLoop(main_n);

    ros::spin();

    return 0;
}