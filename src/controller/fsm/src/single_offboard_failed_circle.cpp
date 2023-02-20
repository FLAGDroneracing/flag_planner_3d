/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

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
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <fsm/command_acc.h>
#include <bspline_race/bspline_race.h>
using namespace std;

static mavros_msgs::State current_state;
static int flight_mode=1;
static int waypoint_num=0;
static int j=1;
static int cmdd=0;
static int flag1=0;
static int flag2=0;
static int ready_fly=0;
static float voltage_now;
static double waypoint[50][3];
static geometry_msgs::PoseStamped aim;
static geometry_msgs::PoseStamped goal;
Eigen::Vector3d pos_drone_fcu;



/*----------ASCUP----------*/

static int flag_A = 0;                              // 0: disable mission A debug       1: enable mission A debug
static int flag_B = 0;                              // 0: disable mission B debug       1: enable mission B debug
static int flag_C = 0;                              // 0: disable mission C debug       1: enable mission C debug
static int flag_D = 0;                              // 0: disable mission D debug       1: enable mission D debug
static int flag_E = 1;                              // 0: disable mission E debug       1: enable mission E debug
static int flag_LAND = 0;                           // 0: disable mission LAND debug    1: enable mission LAND debug  

int COUNT_WALL = 5;
int COUNT_TUNNEL = 5;
int COUNT_CIRCLE = 5;
int COUNT_RING = 5;
double LAMDA_WALL = 3.0;
double LAMDA_TUNNEL = 3.0;
double LAMDA_CIRCLR = 1.0;
double LAMDA_RING = 0.5;

Eigen::Vector3d pos_drone_now;
Eigen::Vector3d pos_drone_temp;
int search_direct = 1;                           // 0: left    1: right

int flag_wall = 0;                           /*----------------------------------------------- 
                                               0: start state                                 
                                               1: arrive point 1 (find bounding box)          
                                               2: arrive point 2 (bounding box right midpoint) 
                                               3: arrive point 3 (bounding box left midpoint)  
                                               4: arrive point 4 (transition before mission B)     
                                               -----------------------------------------------*/
int count_box = 0;
Eigen::Vector3d box_midpoint;
double box_width;

int flag_land = 0;                           /* 0: start state: determine wait coordinate
                                               1: wait recognition 
                                               2: determine tag coordinate
                                               3: fly towards tag         
                                               4: land                     */ 
int flag_tag_recog = 0;
int count_tag = 0;
Eigen::Vector3d pos_tag;
Eigen::Vector3d pos_land;                

static int flag_tunnel = 0;

int flag_ring = 0;                           /*-------------------------------------
                                               0-5-10: search ring                  
                                               1-6-11: check ring recognition       
                                               2-7-12: determine ring position      
                                               3-8-13: fly towards point before ring
                                               4-9-14: fly towards point after ring 
                                               -   15: transition to mission LAND   
                                               -------------------------------------*/


int flag_ring1 = 0;
int count_ring1 = 0;
int flag_ring_recog = 0;
int flag_tunnel_recog = 0;
int flag_wall_recog = 0;

int count_ring = 0;
int count_tunnel = 0;
int count_wall = 0;

int flag_circle = 0;                           /*
                                                 0: 
                                                 1: 
                                                 2:
                                                               */

Eigen::Vector3d circle_center;                                //center point from ring recognition
Eigen::Vector3d circle_direct;                                //normal vector from ring recognition
Eigen::Vector3d circle_before;                                //point before going through ring      
Eigen::Vector3d circle_after;                                 //point after going through ring    

Eigen::Vector3d ring_point1;
Eigen::Vector3d ring_point2;
Eigen::Vector3d ring_point1_temp;
Eigen::Vector3d ring_point2_temp;
Eigen::Vector3d ring_center_temp;
Eigen::Vector3d ring_direct_temp;
Eigen::Vector3d ring_delta;
Eigen::Vector3d ring_center;                                //center point from ring recognition
Eigen::Vector3d ring_direct;                                //normal vector from ring recognition
Eigen::Vector3d ring_before;                                //point before going through ring      
Eigen::Vector3d ring_after;                                 //point after going through ring    
Eigen::Vector2d ring_orientation;                           //yaw orientation to go through ring

Eigen::Vector3d tunnel_entry;                                
Eigen::Vector3d tunnel_before;                                
Eigen::Vector3d tunnel_after;                                                              

Eigen::Vector3d wall_point0;
Eigen::Vector3d wall_point1;
Eigen::Vector3d wall_point2;
Eigen::Vector3d wall_point0_temp;
Eigen::Vector3d wall_point1_temp;
Eigen::Vector3d wall_point2_temp;
Eigen::Vector3d wall_center_temp;
Eigen::Vector3d wall_direct_vertical;
Eigen::Vector3d wall_center;                                
Eigen::Vector3d wall_direct;
Eigen::Vector3d wall_0;                                
Eigen::Vector3d wall_before;                               
Eigen::Vector3d wall_after;                                   
Eigen::Vector2d wall_orientation;

bspline_race::BsplineTraj traj_fsm;
// geometry_msgs::PoseStamped[] traj_pos;
// geometry_msgs::PoseStamped[] traj_vel;
// geometry_msgs::PoseStamped[] traj_acc;
int traj_len;
int traj_p = 0;
int flag_new_traj = 1;

/*-------------------------*/

/*----------ASCUP----------*/
void box_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if(count_box < 10)
    {
        box_midpoint = Eigen::Vector3d(msg->data[0], msg->data[1], 0.6);
        box_width = msg->data[3];
        
        /*----------check bounding box width: error < 0.3 as correct----------*/
        if(abs(box_width - 2.5) < 0.3) {count_box = count_box + 1;} 
    }
}

void ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ring_delta = Eigen::Vector3d(msg->pose.position.z, -msg->pose.position.x, -msg->pose.position.y);
    ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);
    
    flag_ring_recog = 1;
}

void pcl_ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//     ring_center = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);

    ring_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ring_direct_temp = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    flag_ring_recog = 1;
}

void wall_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//     ring_center = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);

    wall_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    wall_direct_vertical = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    flag_wall_recog = 1;
    ROS_ERROR("fuckkkkkkkkkkkkkkk!");
}

void tunnel_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tunnel_entry = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    
    flag_tunnel_recog = 1;
    ROS_ERROR("fuck tunnel!!!");
}

void tag_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_tag = Eigen::Vector3d(msg->pose.position.z, -msg->pose.position.x, -msg->pose.position.y);
    flag_tag_recog = 1;
}

void traj_cb(const bspline_race::BsplineTraj::ConstPtr& msg)
{
    if(flag_new_traj == 1) 
    {
        traj_fsm.position = msg->position;
        //traj_fsm.position = msg->velocity;
        //traj_acc = msg->acceleration;
        traj_len = msg->position.size();
        traj_p = traj_len - 1;                                                //???
    }

    flag_new_traj = 0;
}

/*-------------------------*/

/*----------ASCUP----------*/

void Set_PosAim(double ptx, double pty, double ptz)
{
    aim.pose.position.x = ptx;
    aim.pose.position.y = pty;      
    aim.pose.position.z = ptz;
}

void Set_SearchTemp(double x_max, double y_max)
{
    if(search_direct == 1) {pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0], pos_drone_temp[1] - 0.01, pos_drone_temp[2]);}
    else                   {pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0], pos_drone_temp[1] + 0.01, pos_drone_temp[2]);}

    if( (pos_drone_temp[1] + y_max) < 0) 
    {
        pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0] + 0.1, -y_max, pos_drone_temp[2]);
        search_direct = 0;
    }
    if( (pos_drone_temp[1] - y_max) > 0)
    {
        pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0] + 0.1, y_max, pos_drone_temp[2]);
        search_direct = 1;
    }

    if(pos_drone_temp[0] > x_max)
    {
        pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0], pos_drone_temp[1], pos_drone_temp[2]);
    }
}

/*-------------------------*/


void vtg_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage_now = msg->voltage;
}

void ready_cb(const fsm::command_acc &msg)
{ 
    ready_fly=msg.ready;
    //ROS_ERROR("11111111");voltage_now
    //cout<<"1111";
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void UdpListen(const uint16_t cport)
{
    ros::NodeHandle nh;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_lis;
    int len;
    memset(&addr_lis, 0, sizeof(struct sockaddr_in));
    addr_lis.sin_family = AF_INET;
    addr_lis.sin_port = htons(cport);
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_lis.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址
    len = sizeof(addr_lis);

    /* 绑定socket */
    if(bind(sock_fd, (struct sockaddr *)&addr_lis, sizeof(addr_lis)) < 0)
    {
      perror("bind error:");
      exit(1);
    }


    int recv_num;
    char recv_buf[100];
    const char dot[2] = ",";
    struct sockaddr_in addr_client;

    while(ros::ok()){
        char *p;
        int ent=0;
        int cmd;
        double px,py,pz;

        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

        if(recv_num < 0||abs(recv_num-19)>3)
        {
            ROS_ERROR("Recv Fail!");
            continue;
        }
        recv_buf[recv_num] = '\0';
//      ROS_INFO("Rec: %s, len=%d",recv_buf,recv_num);

        p = strtok(recv_buf,dot);
        sscanf(p,"%d",&cmd);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&px);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&py);
        p=strtok(NULL,dot);
        sscanf(p,"%lf",&pz);
        if(cmd!=6)
        {
          aim.pose.position.x=px;
          aim.pose.position.y=py;
          aim.pose.position.z=pz;
        }
//        ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
        if(flag2==0)
        cmdd=cmd;
         switch (cmdd)
        {
        case 0:
            if(ready_fly==1)
            cout<<"Ready Fly"<<endl;
            else
            ROS_INFO("Waiting");
            break;
/*            
        case 1:
            if(pos_drone_fcu[2]>1.5||voltage_now<13.5)
            {
            cmdd=4;
            flag2=1;
            }
            break;
        case 9:
            if(pos_drone_fcu[2]>1.5||voltage_now<13.5)
            {
            cmdd=4;
            flag2=1;
            }
            break;
*/            
        }
        //   cout<<voltage_now<<endl;
        //    cout<<pos_drone_fcu[2]<<endl;
    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>   
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose_orb", 100, pos_cb);
    ros::Subscriber ready= nh.subscribe("/px4/ready",10,ready_cb);
    ros::Subscriber voltage_sub = nh.subscribe<sensor_msgs::BatteryState>
            ("/mavros/battery", 300, vtg_cb);

    /*----------AISHENG CUP----------*/
    ros::Subscriber box_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("/task_A_msgs", 100, box_cb);
    ros::Subscriber ring_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/circle_pose", 100, ring_cb);
    ros::Subscriber pcl_ring_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/test_ring_msg", 100, pcl_ring_cb);
    ros::Subscriber wall_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/walls_pose", 100, wall_cb);
    ros::Subscriber tunnel_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/task_B_msg", 100, tunnel_cb);
    ros::Subscriber tag_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/apriltag_ros_continuous_node/apriltag_pose", 100, tag_cb);
    ros::Subscriber traj_sub = nh.subscribe<bspline_race::BsplineTraj>       //路径规划，接收B样条的点
            ("/bspline_traj", 10 ,traj_cb);

    ros::Publisher traj_goal_pub = nh.advertise<geometry_msgs::PoseStamped>  //路径规划，发送目标点
            ("/move_base_simple/goal", 100);
    
    /*-------------------------------*/

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>  
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>   
            ("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>    
            ("mavros/setpoint_position/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //parameters
    nh.param("/single_offboard/way_num",waypoint_num,0);
    aim.pose.position.x = 0;
    aim.pose.position.y = 0;
    aim.pose.position.z = 1.0;
    // double arg_ = atan2(-0.0,-1) + (3.1415926/2.0f);
    // geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(arg_);
    // aim.pose.orientation = geo_q;

    //point
    for (int i = 1; i <= waypoint_num; i++)
    {
      nh.param("/single_offboard/waypoint" + to_string(i) + "_x", waypoint[i][0], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_y", waypoint[i][1], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_z", waypoint[i][2], -1.0);
    }
//    nh.param("/single_offboard/waypoint1_x",aim.pose.position.x,1.0);
//    nh.param("/single_offboard/waypoint1_y",aim.pose.position.y,2.0);
//    nh.param("/single_offboard/waypoint1_z",aim.pose.position.z,3.0);
    new std::thread(&UdpListen,12001);
    // wait for FCU connection
/*
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
*/
    sensor_msgs::BatteryState voltage_now;
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode="OFFBOARD";

    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "LAND";

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd,disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.value = false;

    ros::Time last_request = ros::Time::now();

    /*----------ASCUP----------*/
    
    ros::Time last_recog = ros::Time::now();
    pos_drone_now = Eigen::Vector3d(0.0, 0.0, 1.0);
    pos_drone_temp = Eigen::Vector3d(0.0, 0.0, 0.0);

    /*-------------------------*/

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(aim);
        ros::spinOnce();
        rate.sleep();
    }




    while(ros::ok())
    {
        //waypoint
        if(cmdd==6)
        {   
//          if(fabs(pos_drone_fcu[0] - waypoint[j][0]) < (0.05) && fabs(pos_drone_fcu[1] - waypoint[j][1]) < (0.05) && fabs(pos_drone_fcu[2] - waypoint[j][2]) < (0.05))
          
        if(sqrt(pow(pos_drone_fcu[0] - waypoint[j][0],2)+pow(pos_drone_fcu[1] - waypoint[j][1],2)+pow(pos_drone_fcu[2] - waypoint[j][2],2))<(0.1) && j<waypoint_num)
          j=j+1;
          aim.pose.position.x = waypoint[j][0];
          aim.pose.position.y = waypoint[j][1];
          aim.pose.position.z = waypoint[j][2];
        }
        //fsm
//        if(cmdd!=4) local_pos_pub.publish(aim);
        if(cmdd==1)
        {
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                
                last_request = ros::Time::now();
            }
            local_pos_pub.publish(aim);
        }
        if(cmdd==2)
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            
        }
        
        if(cmdd==9)
        {
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                  ROS_ERROR("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            else if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
               if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_ERROR("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
                  ROS_ERROR("Vehicle armed");
            local_pos_pub.publish(aim);
        }
        
        if(cmdd == 4)
        {   
            if(abs(pos_drone_fcu[2] - 0.1) < 0.05) {flag1 = 1;}
            if(flag1 == 0)
            {
                aim.pose.position.x = pos_drone_fcu[0];
                aim.pose.position.y = pos_drone_fcu[1];      
                aim.pose.position.z = 0.1;
                local_pos_pub.publish(aim);
            }
            else
            {
                if(current_state.mode == "OFFBOARD")
                {
                    mode_cmd.request.custom_mode = "Hold";
                    set_mode_client.call(mode_cmd);
                    ROS_ERROR("1111");
                }
                if(current_state.armed)
                {
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                    ROS_ERROR("2222");
                }
                if (arm_cmd.response.success)
                {
                    ROS_ERROR("Disarm successfully!");
                }
            }
        }

        if(cmdd==5)
        {
            if(current_state.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }

            if(current_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);
            }

            if (arm_cmd.response.success)
            {
                ROS_ERROR("Disarm successfully!");
            }
        }

        /*----------ASCUP----------*/
        if(cmdd == 7)
        {
            /*----------mission A----------*/
            if(flag_A == 1)
            {    
                if(flag_wall == 0)
                {
                    /*----------check wall recognition----------*/
                    if(count_wall < 5)
                    {
                        if(flag_wall_recog == 1)
                        {
                            count_wall = count_wall + 1;
                            flag_wall_recog = 0;
                        }
                        aim.pose.position.x = pos_drone_fcu[0];
                        aim.pose.position.y = pos_drone_fcu[1];      
                        aim.pose.position.z = 0.7;
                        local_pos_pub.publish(aim);
                    }
                    /*----------determine point before and after ring----------*/
                    else if(count_wall == 5)
                    {
                        count_wall = count_wall + 1;    
                       
                        wall_direct = Eigen::Vector3d(-wall_direct_vertical[2], wall_direct_vertical[1], wall_direct_vertical[0]);//parallel wall
                        
                        wall_point0_temp = wall_center_temp ;
                        wall_point1_temp = wall_center_temp + LAMDA_WALL * wall_direct;
                        wall_point2_temp = wall_point1_temp + 2.0 * wall_direct_vertical;
 
                        // position in body                    

                        wall_point0 = Eigen::Vector3d(wall_point0_temp[2], -wall_point0_temp[0], -wall_point0_temp[1]);                                                                    
                        wall_point1 = Eigen::Vector3d(wall_point1_temp[2], -wall_point1_temp[0], -wall_point1_temp[1]);
                        wall_point2 = Eigen::Vector3d(wall_point2_temp[2], -wall_point2_temp[0], -wall_point2_temp[1]);

                        //ring_point1 equal 0
                        wall_0 = Eigen::Vector3d(pos_drone_fcu[0] + wall_point0[0], 
                                                    pos_drone_fcu[1] + wall_point0[1], 
                                                    pos_drone_fcu[2] + wall_point0[2]);
                        wall_before = Eigen::Vector3d(pos_drone_fcu[0] + wall_point1[0], 
                                                    pos_drone_fcu[1] + wall_point1[1], 
                                                    pos_drone_fcu[2] + wall_point1[2]);
                        wall_after = Eigen::Vector3d(pos_drone_fcu[0] + wall_point2[0], 
                                                    pos_drone_fcu[1] + wall_point2[1], 
                                                    pos_drone_fcu[2] + wall_point2[2]);                                          
                        aim.pose.position.x = wall_0[0];
                        aim.pose.position.y = wall_0[1];      
                        aim.pose.position.z = 0.7;
                        local_pos_pub.publish(aim);
ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmdd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
                    }
                    /*----------fly towards point before ring----------*/
                    else if(count_wall == 6)
                    {
                        aim.pose.position.x = wall_0[0];
                        aim.pose.position.y = wall_0[1];      
                        aim.pose.position.z = 0.7;
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - wall_0[0]) < 0.1) && (abs(pos_drone_fcu[1] - wall_0[1]) < 0.1) && (abs(pos_drone_fcu[2] - 0.7) < 0.1) )
                        {
                            count_wall = count_wall + 1;
                        }
                    }
                    /*----------fly towards point after ring----------*/
                    else if(count_wall == 7)
                    {
                        aim.pose.position.x = wall_before[0];
                        aim.pose.position.y = wall_before[1];      
                        aim.pose.position.z = 0.7;
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - wall_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - wall_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - 0.7) < 0.1) )
                        {
                            count_wall = count_wall + 1;
                        }
                    }
                    else
                    {
                        ROS_ERROR("99999999999999999999999999");
                        aim.pose.position.x = wall_after[0];
                        aim.pose.position.y = wall_after[1];      
                        aim.pose.position.z = 0.7;
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - wall_after[0]) < 0.1) && (abs(pos_drone_fcu[1] - wall_after[1]) < 0.1) && (abs(pos_drone_fcu[2] - 0.7) < 0.1) )
                        {
                            flag_wall = 1;
                        }
                    }
                }

                if(flag_wall == 1)
                {
                    aim.pose.position.x = pos_drone_fcu[0];
                    aim.pose.position.y = pos_drone_fcu[1];      
                    aim.pose.position.z = 0.1;
                    local_pos_pub.publish(aim);
                }
            }


            
            /*----------mission B----------*/
            if(flag_B == 1)
            {    
                /*----------determine wait point----------*/
                if(flag_tunnel == 0)
                {
                    pos_drone_now = Eigen::Vector3d(pos_drone_now[0], pos_drone_now[1] - 0.01, 1.0);
                    Set_PosAim(pos_drone_now[0], pos_drone_now[1], 1.0);
		            local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, flag: %d, recog: %d, count: %d, %.3lf, %.3lf", cmdd, flag_tunnel, flag_tunnel_recog, count_tunnel, aim.pose.position.x, aim.pose.position.y);
                    
                    /*----------wait every 20cm----------*/
                    count_tunnel = count_tunnel + 1;
                    if(count_tunnel == 30)
                    {
                        count_tunnel = 0;
                        flag_tunnel = 1; 
                        last_recog = ros::Time::now();   
                    }
                }
                
                /*----------wait and check tunnel recognition----------*/
                if(flag_tunnel == 1)
                {    
                    Set_PosAim(pos_drone_now[0], pos_drone_now[1], 1.0);
		            local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, flag: %d, recog: %d, count: %d, %.3lf, %.3lf", cmdd, flag_tunnel, flag_tunnel_recog, count_tunnel, aim.pose.position.x, aim.pose.position.y);
                    
                    /*----------check tunnel recognition----------*/
                    if(count_tunnel < COUNT_TUNNEL)
                    {
                        if(flag_tunnel_recog == 1)
                        {
                            count_tunnel = count_tunnel + 1;
                            last_recog = ros::Time::now();
                            flag_tunnel_recog = 0;
                        }

                        /*----------no recognition during 10s as wrong----------*/
                        if( ros::Time::now() - last_recog > ros::Duration(5.0) ) 
                        {
                            count_tunnel = 0;
                            flag_tunnel = 0;
                        }
                    }
                    
                    /*----------COUNT_TUNNEL times recognition as successful----------*/
                    else {flag_tunnel = 2;}
                }

                /*----------determine point before and after tunnel----------*/
                if(flag_tunnel == 2)
                {       
                    tunnel_before = Eigen::Vector3d(tunnel_entry[0], tunnel_entry[1], 1.0);
                    tunnel_after = Eigen::Vector3d(tunnel_before[0] + LAMDA_TUNNEL, tunnel_before[1], 1.0);

                    flag_tunnel = 3;
                }

                /*----------fly towards point before tunnel----------*/
                if(flag_tunnel == 3)
                {    
                    Set_PosAim(tunnel_before[0], tunnel_before[1], 1.0);
                    local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf", cmdd, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);

                    if( (abs(pos_drone_fcu[0] - tunnel_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - tunnel_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - 1.0) < 0.1) )
                    {
                        flag_tunnel = 4;
                    }
                }

                /*----------fly towards point after tunnel----------*/
                if(flag_tunnel == 4)
                {
                    Set_PosAim(tunnel_after[0], tunnel_after[1], 1.0);
                    local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf", cmdd, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);
                    
                    if( (abs(pos_drone_fcu[0] - tunnel_after[0]) < 0.1) && (abs(pos_drone_fcu[1] - tunnel_after[1]) < 0.1) && (abs(pos_drone_fcu[2] - 1.0) < 0.1) )
                    {
                        pos_drone_temp = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], 1.0);
                        flag_tunnel = 5;
                    }
                }

                if(flag_tunnel == 5)
                {
                    Set_PosAim(pos_drone_temp[0], pos_drone_temp[1], 0.1);
                    local_pos_pub.publish(aim);
                }
                        
            }
            
            /*----------mission D----------*/
/*            if(flag_D == 1)
            {    
                if(flag_ring1 == 0)
                {

                    if(count_ring1 <5)
                    {
                        if(flag_ring_recog == 1)
                        {
                            count_ring1 = count_ring1 + 1;
                            flag_ring_recog = 0;
                        }
                        aim.pose.position.x = pos_drone_fcu[0];
                        aim.pose.position.y = pos_drone_fcu[1];      
                        aim.pose.position.z = pos_drone_fcu[2];
                        local_pos_pub.publish(aim);
                    }

                    else if(count_ring1 == 5)
                    {
                        count_ring1 = count_ring1 + 1;

                        ring_point1_temp = ring_center_temp - lamda_ring * ring_direct_temp;
                        ring_point2_temp = ring_center_temp + lamda_ring * ring_direct_temp;
 
                        // position in body                                                                                                                
                        ring_point1 = Eigen::Vector3d(ring_point1_temp[2], -ring_point1_temp[0], -ring_point1_temp[1]);
                        ring_point2 = Eigen::Vector3d(ring_point2_temp[2], -ring_point2_temp[0], -ring_point2_temp[1]);

                        //ring_point1 equal 0
                        ring_before = Eigen::Vector3d(pos_drone_fcu[0] + ring_point1[0], 
                                                    pos_drone_fcu[1] + ring_point1[1], 
                                                    pos_drone_fcu[2] + ring_point1[2]);
                        ring_after = Eigen::Vector3d(pos_drone_fcu[0] + ring_point2[0], 
                                                    pos_drone_fcu[1] + ring_point2[1], 
                                                    pos_drone_fcu[2] + ring_point2[2]);                                          
                        aim.pose.position.x = ring_before[0];
                        aim.pose.position.y = ring_before[1];      
                        aim.pose.position.z = ring_before[2];
                        local_pos_pub.publish(aim);
ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmdd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
                    }

                    else if(count_ring1 == 6)
                    {
                        aim.pose.position.x = ring_before[0];
                        aim.pose.position.y = ring_before[1];      
                        aim.pose.position.z = ring_before[2];
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - ring_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - ring_before[2]) < 0.1) )
                        {
                            count_ring1 = count_ring1 + 1;
                        }
                    }

                    else
                    {
                        aim.pose.position.x = ring_after[0];
                        aim.pose.position.y = ring_after[1];      
                        aim.pose.position.z = ring_after[2];
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - ring_after[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_after[1]) < 0.1) && (abs(pos_drone_fcu[2] - ring_after[2]) < 0.1) )
                        {
                            flag_ring1 = 1;
                        }
                    }
                }
*/
            if(flag_D == 1)
            {    
                if(flag_ring1 == 0)
                {
                    /*----------check ring detection----------*/
                    if(count_ring1 <5)
                    {
                        if(flag_ring_recog == 1)
                        {
                            count_ring1 = count_ring1 + 1;
                            flag_ring_recog = 0;
                        }
                        aim.pose.position.x = pos_drone_fcu[0];
                        aim.pose.position.y = pos_drone_fcu[1];      
                        aim.pose.position.z = pos_drone_fcu[2];
                        local_pos_pub.publish(aim);
                    }
                    /*----------determine point before and after ring----------*/
                    else if(count_ring1 == 5)
                    {
                        count_ring1 = count_ring1 + 1;

                        ring_point1_temp = ring_center_temp - LAMDA_RING * ring_direct_temp;
                        ring_point2_temp = ring_center_temp + LAMDA_RING * ring_direct_temp;
 
                        // position in body                                                                                                                
                        ring_point1 = Eigen::Vector3d(ring_point1_temp[2], -ring_point1_temp[0], -ring_point1_temp[1]);
                        ring_point2 = Eigen::Vector3d(ring_point2_temp[2], -ring_point2_temp[0], -ring_point2_temp[1]);

                        //ring_point1 equal 0
                        ring_before = Eigen::Vector3d(ring_point1_temp[0], 
                                                    ring_point1_temp[1], 
                                                    ring_point1_temp[2]);
                        ring_after = Eigen::Vector3d(ring_point2_temp[0], 
                                                    ring_point2_temp[1], 
                                                    ring_point2_temp[2]);                                          
                        aim.pose.position.x = ring_before[0];
                        aim.pose.position.y = ring_before[1];      
                        aim.pose.position.z = ring_before[2];
                        local_pos_pub.publish(aim);
ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmdd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
                    }
                    /*----------fly towards point before ring----------*/
                    else if(count_ring1 == 6)
                    {
                        aim.pose.position.x = ring_before[0];
                        aim.pose.position.y = ring_before[1];      
                        aim.pose.position.z = ring_before[2];
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - ring_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - ring_before[2]) < 0.1) )
                        {
                            count_ring1 = count_ring1 + 1;
                        }
                    }
                    /*----------fly towards point after ring----------*/
                    else
                    {
                        aim.pose.position.x = ring_after[0];
                        aim.pose.position.y = ring_after[1];      
                        aim.pose.position.z = ring_after[2];
                        local_pos_pub.publish(aim);

                        if( (abs(pos_drone_fcu[0] - ring_after[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_after[1]) < 0.1) && (abs(pos_drone_fcu[2] - ring_after[2]) < 0.1) )
                        {
                            flag_ring1 = 1;
                        }
                    }
                }

                if(flag_ring1 == 1)
                {
                    aim.pose.position.x = pos_drone_fcu[0];
                    aim.pose.position.y = pos_drone_fcu[1];      
                    aim.pose.position.z = 0.1;
                    local_pos_pub.publish(aim);
                }
            }



            /*----------mission E----------*/
            if(flag_E == 1)
            {   
               /*----------search ring----------*/
                if( (flag_ring == 0) || (flag_ring == 5) || (flag_ring == 10) )
                {
                    Set_SearchTemp(2.0, 2.0);
                    Set_PosAim(pos_drone_now[0] + pos_drone_temp[0], pos_drone_now[1] + pos_drone_temp[1], 1.0);
                    local_pos_pub.publish(aim);
                    ROS_INFO("fffffffffuck!Rec: %d, flag: %d, recog: %d, count: %d, %.3lf, %.3lf", cmdd, flag_ring, flag_ring_recog, count_ring, aim.pose.position.x, aim.pose.position.y);
                    
                    /*----------wait every 20 counts----------*/
                    count_ring = count_ring + 1;
                    if(count_ring == 20)
                    {
                        count_ring = 0;
                        flag_ring = flag_ring + 1; 
                        last_recog = ros::Time::now();   
                    }
                }

                if( (flag_ring == 1) || (flag_ring == 6) || (flag_ring == 11) )
                {    
                    Set_PosAim(pos_drone_now[0] + pos_drone_temp[0], pos_drone_now[1] + pos_drone_temp[1], 1.0);
		            local_pos_pub.publish(aim);
                    ROS_INFO("fuckkkkkkkkkkk!Rec: %d, flag: %d, recog: %d, count: %d, %.3lf, %.3lf", cmdd, flag_ring, flag_ring_recog, count_ring, aim.pose.position.x, aim.pose.position.y);
                    
                    /*----------check tunnel recognition----------*/
                    if(count_ring < COUNT_RING)
                    {
                        if(flag_ring_recog == 1)
                        {
                            count_ring = count_ring + 1;
                            last_recog = ros::Time::now();
                            flag_ring_recog = 0;
                        }

                        /*----------no recognition during 5s as wrong----------*/
                        if( ros::Time::now() - last_recog > ros::Duration(5.0) ) 
                        {
                            count_ring = 0;
                            flag_ring = flag_ring - 1;
                        }
                    }
                    
                    /*----------COUNT_TUNNEL times recognition as successful----------*/
                    else {flag_ring = flag_ring + 1;}
                }

                /*----------determine point before and after tunnel----------*/
                if( (flag_ring == 2) || (flag_ring == 7) || (flag_ring == 12) )
                {       
                    ring_center = pos_drone_fcu + ring_delta;
                    ring_before = ring_center - ring_direct*LAMDA_RING;
                    ring_after = ring_center + ring_direct*LAMDA_RING;

                    flag_ring = flag_ring + 1;
                }

                /*----------fly towards point before tunnel----------*/
                if( (flag_ring == 3) || (flag_ring == 8) || (flag_ring == 13) )
                {    
                    Set_PosAim(ring_before[0], ring_before[1], ring_before[2]);
                    local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf", cmdd, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);

                    if( (abs(pos_drone_fcu[0] - ring_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - ring_before[2]) < 0.1) )
                    {
                        flag_ring = flag_ring + 1;
                    }
                }

                /*----------fly towards point after tunnel----------*/
                if( (flag_ring == 4) || (flag_ring == 9) || (flag_ring == 14) )
                {
                    Set_PosAim(ring_after[0], ring_after[1], ring_after[2]);
                    local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf", cmdd, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);
                    
                    if( (abs(pos_drone_fcu[0] - ring_after[0]) < 0.1) && (abs(pos_drone_fcu[1] - ring_after[1]) < 0.1) && (abs(pos_drone_fcu[2] -ring_after[2]) < 0.1) )
                    {
                        pos_drone_temp = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], 1.0);
                        flag_ring = flag_ring+ 1;
                    }
                }

                if(flag_ring == 15)
                {
                    Set_PosAim(ring_after[0], ring_after[1], 0.1);
                    local_pos_pub.publish(aim);
                }
            }


            /*----------mission LAND----------*/
            if(flag_LAND == 1)
            {
                /*----------determine wait coordinate----------*/
                if(flag_land == 0)
                {
                    pos_drone_now = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], pos_drone_fcu[2]);
                    aim.pose.position.x = pos_drone_now[0];
                    aim.pose.position.y = pos_drone_now[1];      
                    aim.pose.position.z = pos_drone_now[2];
                    local_pos_pub.publish(aim);
                    
                    flag_land = 1;
                }
                
                /*----------wait and check tag recognition: 10 recognition as successful----------*/
                if(flag_land == 1)
                {
                    aim.pose.position.x = pos_drone_now[0];
                    aim.pose.position.y = pos_drone_now[1];      
                    aim.pose.position.z = pos_drone_now[2];
                    local_pos_pub.publish(aim);

                    if(flag_tag_recog == 1)
                    {
                        count_tag = count_tag + 1;
                        flag_tag_recog = 0;
                    }
                    if(count_tag == 10) {flag_land = 2;}
                }

                /*----------determine tag coordinate----------*/
                if(flag_land == 2)
                {
                    pos_land = Eigen::Vector3d(pos_drone_now[0] + pos_tag[0], pos_drone_now[1] + pos_tag[1], 0.1);
                    aim.pose.position.x = pos_land[0];
                    aim.pose.position.y = pos_land[1];      
                    aim.pose.position.z = pos_land[2];
                    local_pos_pub.publish(aim);

                    flag_land = 3;
                }

                /*----------fly towards tag----------*/
                if(flag_land == 3)
                {
                    aim.pose.position.x = pos_land[0];
                    aim.pose.position.y = pos_land[1];      
                    aim.pose.position.z = pos_land[2];
                    local_pos_pub.publish(aim);

                    if( (abs(pos_drone_fcu[0] - pos_land[0]) < 0.1) 
                     && (abs(pos_drone_fcu[1] - pos_land[1]) < 0.1) 
                     && (abs(pos_drone_fcu[2] - 0.1) < 0.05) )      {flag_land = 4;}
                }
                
                /*----------land and lock----------*/
                if(flag_land == 4)
                {
                    if(current_state.mode == "OFFBOARD")
                    {
                        mode_cmd.request.custom_mode = "Hold";
                        set_mode_client.call(mode_cmd);
                        ROS_ERROR("1111");
                    }
                    if(current_state.armed)
                    {
                        arm_cmd.request.value = false;
                        arming_client.call(arm_cmd);
                        ROS_ERROR("2222");
                    }
                    if (arm_cmd.response.success)
                    {
                        ROS_ERROR("Disarm successfully!");
                    }
                }
            }





        }


        if(cmdd == 8)
        {
            
//            if(flag_new_traj == 1)
//            {
//                traj_p = 0;
//                flag_new_traj = 0;
//            }
            /*----------给规划器发送目标点----------*/
            goal.pose.position.x = 3.0;
            goal.pose.position.y = 0.0;
            goal.pose.position.z = 0.3;
               cout<<"fuck"<<endl;
            traj_goal_pub.publish(goal);
            
            cout<<"flag_new_traj="<<flag_new_traj<<endl;

            /*----------读取点并发送----------*/
            if(flag_new_traj == 0)
            {
                
                if(traj_p > 0)
                {
                    aim.pose.position.x = traj_fsm.position[traj_p].pose.position.x;
                    aim.pose.position.y = traj_fsm.position[traj_p].pose.position.y;
                    aim.pose.position.z = 0.3;
                    local_pos_pub.publish(aim);

                    traj_p = traj_p - 1;
                }
                else
                {
                    aim.pose.position.x = traj_fsm.position[0].pose.position.x;
                    aim.pose.position.y = traj_fsm.position[0].pose.position.y;
                    aim.pose.position.z = 0.3;
                    local_pos_pub.publish(aim);
                }
                cout<<aim.pose.position.x<<"    "<<aim.pose.position.y<<"    "<<aim.pose.position.z<<endl;
            }    

        }



        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}



/* yaw:  0 deg      1      0      0      0
        90 deg    0.707    0      0    0.707
       180 deg      0      0      0      1
       -90 deg    0.707    0      0   -0.707    */
