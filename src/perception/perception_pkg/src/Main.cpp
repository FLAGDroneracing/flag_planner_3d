/* INCLUDE */
#include "RingDetection.cpp"
#include "TubeDetection.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main");
    ros::NodeHandle main_nh("~");

    // RingDetection ringDetection;
    // ringDetection.ring_detection_loop(main_nh);

    TubeDetection tubedetection;
    tubedetection.TubeDetectionLoop(main_nh);    

    ros::spin();

    return 0;
}
