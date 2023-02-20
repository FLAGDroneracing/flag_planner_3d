#include"fsm/single_offboard.h"

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
Eigen::Vector3d pos_drone_now;
Eigen::Vector2d yaw_drone_fcu;
Eigen::Vector4d Euler_drone_fcu;
mavros_msgs::PositionTarget movement;

// bspline
#define PI acos(-1)
#define DEV 0.05
bool traj_unlocked = true;
bool arrived = false;
double get_traj_time, curr_traj_time;
double task_3_height;
double roll, pitch, yaw;//定义存储r\p\y的容器
double last_yaw;
double last_yaw_dot;
Eigen::Vector3d vel_drone_fcu;
vector<Eigen::Vector2d> traj;
vector<Eigen::Vector2d> vel_;
vector<Eigen::Vector2d> acc_;
  double delta_T = 0.05;
  double D_YAW_MAX = PI/3;
  double output_yaw;
  double output_d_yaw;
  double YAW_MAX = D_YAW_MAX * delta_T;
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw);
std::pair<double, double> cal_yaw( double current_yaw,double aim_yaw);

/*----------ASCUP----------*/
static int flag_A = 0;                              // 0: disable mission A debug       1: enable mission A debug
static int flag_B = 0;                              // 0: disable mission B debug       1: enable mission B debug
static int flag_C = 0;                              // 0: disable mission C debug       1: enable mission C debug
static int flag_D = 0;                              // 0: disable mission D debug       1: enable mission D debug
static int flag_E = 1;                              // 0: disable mission E debug       1: enable mission E debug
static int flag_LAND = 0;                           // 0: disable mission land debug    1: enable mission land debug  

int flag_a = 0;                           /* 0: start state
                                             1: arrive point 1 (find bounding box)
                                             2: arrive point 2 (bounding box right midpoint) 
                                             3: arrive point 3 (bounding box left midpoint)  
                                             4: arrive point 4 (transition before mission B)     */
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

static int flag_wall = 0;
static int flag_tunnel = 0;

static int flag_ring1 = 0;                //修改，原来为2
static int flag_ring2 = 0;
static int flag_ring3 = 0;
static int flag_ring4 = 0;

float lamda_ring = 1.0;
float lamda_wall = 3.0;
float lamda_tunnel = 3.0;

int flag_ring_recog = 0;
int flag_tunnel_recog = 0;
int flag_wall_recog = 0;

int count_ring1 = 0;
int count_tunnel = 0;
int count_wall = 0; 

Eigen::Vector3d ring_point1;
Eigen::Vector3d ring_point2;
Eigen::Vector3d ring_point1_temp;
Eigen::Vector3d ring_point2_temp;
Eigen::Vector3d ring_center_temp;
Eigen::Vector3d ring_direct_temp;
Eigen::Vector3d ring_center;                                //center point from ring recognition
Eigen::Vector3d ring_direct;                                //normal vector from ring recognition
Eigen::Vector3d ring_before;                                //point before going through ring      
Eigen::Vector3d ring_after;                                 //point after going through ring    
Eigen::Vector2d ring_orientation;                           //yaw orientation to go through ring

Eigen::Vector3d tunnel_point0;
Eigen::Vector3d tunnel_point1;
Eigen::Vector3d tunnel_point2;
Eigen::Vector3d tunnel_point0_temp;
Eigen::Vector3d tunnel_point1_temp;
Eigen::Vector3d tunnel_point2_temp;
Eigen::Vector3d tunnel_center_temp;
Eigen::Vector3d tunnel_direct_vertical;
Eigen::Vector3d tunnel_center;                                
Eigen::Vector3d tunnel_direct;
Eigen::Vector3d tunnel_0;                                
Eigen::Vector3d tunnel_before;                               
Eigen::Vector3d tunnel_after;                                   
Eigen::Vector2d tunnel_orientation;                          

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

/* ---point cloud --- */
std::vector<geometry_msgs::PoseStamped> visited_target;
geometry_msgs::PoseStamped target_pt_pos;   // 目标点位姿

bool accurate_ring_received;
bool OK_to_process=false;
/* ---point cloud --- */

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
//     ring_center = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);

    ring_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ring_direct_temp = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    flag_ring_recog = 1;
}

// void pcl_ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     ring_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct_temp = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
//     flag_ring_recog = 1;
// }

void wall_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//     ring_center = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);

    wall_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    wall_direct_vertical = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    flag_wall_recog = 1;
ROS_ERROR("fuckkkkkkkkkkkkkkk!");
}

void tunnel_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
//     ring_center = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     ring_direct = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);

    tunnel_center_temp = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tunnel_direct_vertical = Eigen::Vector3d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    flag_tunnel_recog = 1;
ROS_ERROR("fuck!!!!!!!!!!!!!!!!!!!!!!");
}

void tag_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_tag = Eigen::Vector3d(msg->pose.position.z, -msg->pose.position.x, -msg->pose.position.y);
    flag_tag_recog = 1;
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
    yaw_drone_fcu = Eigen::Vector2d(msg->pose.orientation.x, msg->pose.orientation.w);
    Euler_drone_fcu = Eigen::Vector4d(msg->pose.orientation.w, msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
    vel_drone_fcu = Eigen::Vector3d(msg->twist.linear.x , msg->twist.linear.y, msg->twist.linear.z);
}

void msg_from_cloud_subCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    int target_nums = msg->data[0];
    for (int i = 0; i < target_nums; i++)
    {
        /* 判断是否钻过 */
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = msg->data[6*i+1];
        temp.pose.position.y = msg->data[6*i+2];
        temp.pose.position.z = msg->data[6*i+3];
        temp.pose.orientation.x = msg->data[6*i+4];
        temp.pose.orientation.y = msg->data[6*i+5];
        temp.pose.orientation.z = msg->data[6*i+6];
        
        for (int j = 0; j < visited_target.size(); j++)
        {
            if (
                pow(temp.pose.position.x - visited_target[j].pose.position.x, 2)<0.5*0.5 &&
                pow(temp.pose.position.y - visited_target[j].pose.position.y, 2)<0.5*0.5 &&
                pow(temp.pose.position.z - visited_target[j].pose.position.z, 2)<0.5*0.5
            )
            {
                continue;
            }
            else    // new appear target
            {
                if (
                    pow(temp.pose.position.x - target_pt_pos.pose.position.x, 2)>0.5*0.5 ||
                    pow(temp.pose.position.y - target_pt_pos.pose.position.y, 2)>0.5*0.5 ||
                    pow(temp.pose.position.z - target_pt_pos.pose.position.z, 2)>0.5*0.5
                )   // not close to target_now
                {
                    target_pt_pos.pose.position.x = temp.pose.position.x - 1.0*temp.pose.orientation.x;
                    target_pt_pos.pose.position.y = temp.pose.position.y - 1.0*temp.pose.orientation.y;
                    target_pt_pos.pose.position.z = temp.pose.position.z;
                }
                
                //(need pub)
            }
        }
        if(visited_target.size()==0)
        {
                    target_pt_pos.pose.position.x = temp.pose.position.x - 1.0*temp.pose.orientation.x;
                    target_pt_pos.pose.position.y = temp.pose.position.y - 1.0*temp.pose.orientation.y;
                    target_pt_pos.pose.position.z = temp.pose.position.z;
        }
        
    }
}

void traj_cb(const bspline_race::BsplineTraj::ConstPtr & msg)
{
    if(traj_unlocked){
        get_traj_time = ros::Time::now().toSec();
        // traj_unlocked = false;
        traj.clear();
        vel_.clear();
        acc_.clear();
        Eigen::Vector2d ptr;
        Eigen::Vector2d ptr_v;
        Eigen::Vector2d ptr_a;
        for(int i = msg->position.size()-1;i>=0;i--)
        {
            ptr << msg->position[i].pose.position.x,msg->position[i].pose.position.y;
            traj.push_back(ptr);
            ptr_v << msg->velocity[i].pose.position.x,msg->velocity[i].pose.position.y;
            vel_.push_back(ptr_v);
            ptr_a << msg->acceleration[i].pose.position.x,msg->acceleration[i].pose.position.y;
            acc_.push_back(ptr_a);
        }
        if(!traj.empty())
        {        
            traj.erase(traj.begin());
            vel_.erase(vel_.begin());
            acc_.erase(acc_.begin());
            traj.erase(traj.begin());
            vel_.erase(vel_.begin());
            acc_.erase(acc_.begin());
        }
    }
}
void arrived_cb(const std_msgs::Bool::ConstPtr & msg)
{
    arrived = msg->data;
    cout<<"[Astar] arrived?"<<arrived<<arrived<<arrived<<arrived<<arrived<<endl;
}


void adjust_yaw(geometry_msgs::PoseStamped target_pos)
{
    double target_yaw;    
    target_yaw = atan2(target_pos.pose.orientation.y, target_pos.pose.orientation.x);
    double uav_row, uav_pitch, uav_yaw;
    geometry_msgs::PoseStamped pos_temp;
    pos_temp.pose.orientation.x = Euler_drone_fcu[1];
    pos_temp.pose.orientation.y = Euler_drone_fcu[2];
    pos_temp.pose.orientation.z = Euler_drone_fcu[3];
    pos_temp.pose.orientation.w = Euler_drone_fcu[0];
    tf::Quaternion tfQ;
    tf::quaternionMsgToTF(pos_temp.pose.orientation, tfQ);
    tf::Matrix3x3(tfQ).getRPY(uav_row, uav_pitch, uav_yaw);
    if ( pow(target_yaw-uav_yaw, 2)>pow(2.0*3.1415926/36.0, 2) )
    {
        movement.position = target_pt_pos.pose.position;
        movement.velocity.x = 0.0;
        movement.velocity.y = 0.0;
        movement.velocity.z = 0.0;
        movement.yaw = uav_yaw + (target_yaw-uav_yaw)/sqrt( pow(target_yaw-uav_yaw, 2)/200.0 );
        movement.yaw_rate = 0.1/20.0;
       // yaw_adjust_puber.publish(movement);
       OK_to_process=false;
    }
    else
    OK_to_process=true;
}

/* 识别及执行函数 */
void detect_and_process()
{
    visited_target.push_back(target_pt_pos);
   flag_D=1;   
   flag_E=1;   
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
        if(cmd!=6 && cmd !=8 && cmd!=3)
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

    std_msgs::Int64 flag_msg;
    flag_msg.data = 0;
    ros::Publisher fsm_pub = nh.advertise<std_msgs::Int64>
            ("flag_detect",1);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>   
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose_orb", 100, pos_cb);
    ros::Subscriber ready= nh.subscribe("/px4/ready",10,ready_cb);
    ros::Subscriber voltage_sub = nh.subscribe<sensor_msgs::BatteryState>
            ("/mavros/battery", 300, vtg_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_local_orb",100,vel_cb);
    /*----------AISHENG CUP----------*/
    ros::Subscriber box_sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("/task_A_msgs", 100, box_cb);
    ros::Subscriber ring_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/circle_pose", 100, ring_cb);
    // ros::Subscriber pcl_ring_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("/test_ring_msg", 100, pcl_ring_cb);
    ros::Subscriber wall_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/walls_pose", 100, wall_cb);
    ros::Subscriber tunnel_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/tunnel_pose", 100, tunnel_cb);
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
/* --- pva --- */
    ros::Publisher local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

/* --- 点云识别的疑似点 --- */
    ros::Subscriber msg_from_cloud_suber = nh.subscribe<std_msgs::Float64MultiArray>("/test_ring_msg", 10, msg_from_cloud_subCallback);
    ros::Publisher bsaim_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/bs_aim",1);
/* --- 点云识别的疑似点 --- */
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //parameters
    nh.param("/single_offboard/way_num",waypoint_num,0);
    aim.pose.position.x = 0;
    aim.pose.position.y = 0;
    aim.pose.position.z = 1.0;
    nh.param("yaw_rate_max", D_YAW_MAX, 0.3);
    nh.param("task_3_height" , task_3_height, 0.8);
    double YAW_MAX = D_YAW_MAX * delta_T;
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
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(aim);
        ros::spinOnce();
        rate.sleep();
    }




    while(ros::ok())
    {
        //if(cmdd!=4) local_pos_pub.publish(aim);
        //waypoint
        flag_msg.data = cmdd;
        fsm_pub.publish(flag_msg);

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
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                
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

            if(flag_A == 1)
            {    
                if(flag_wall == 0)
                {
                    if(count_wall <5)
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

                    else if(count_wall == 5)
                    {
                        count_wall = count_wall + 1;    
                       
                        wall_direct = Eigen::Vector3d(-wall_direct_vertical[2], wall_direct_vertical[1], wall_direct_vertical[0]);//parallel wall
                        
                        wall_point0_temp = wall_center_temp ;
                        wall_point1_temp = wall_center_temp + lamda_wall * wall_direct;
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


            if(flag_B == 1)
            {    
                if(flag_tunnel == 0)
                {
                    /*----------check ring detection----------*/
                    if(count_tunnel <5)
                    {
                        if(flag_tunnel_recog == 1)
                        {
                            count_tunnel = count_tunnel + 1;
                            flag_tunnel_recog = 0;
                        }
                        aim.pose.position.x = pos_drone_fcu[0];
                        aim.pose.position.y = pos_drone_fcu[1];      
                        aim.pose.position.z = 1.0;
                        local_pos_pub.publish(aim);
                    }
                    /*----------determine point before and after ring----------*/
                    else if(count_tunnel == 5)
                    {
                        count_tunnel = count_tunnel + 1;    
                       
                        tunnel_direct = Eigen::Vector3d(-tunnel_direct_vertical[2], tunnel_direct_vertical[1], tunnel_direct_vertical[0]);//parallel tunnel
                        
                        tunnel_point0_temp = tunnel_center_temp ;
                        tunnel_point1_temp = tunnel_center_temp + lamda_tunnel * tunnel_direct;
                        tunnel_point2_temp = tunnel_point1_temp + 2.0 * tunnel_direct_vertical;
 
                        // position in body                    

                        tunnel_point0 = Eigen::Vector3d(tunnel_point0_temp[2], -tunnel_point0_temp[0], -tunnel_point0_temp[1]);                                                                    
                        tunnel_point1 = Eigen::Vector3d(tunnel_point1_temp[2], -tunnel_point1_temp[0], -tunnel_point1_temp[1]);
                        tunnel_point2 = Eigen::Vector3d(tunnel_point2_temp[2], -tunnel_point2_temp[0], -tunnel_point2_temp[1]);

                        //ring_point1 equal 0
                        tunnel_0 = Eigen::Vector3d(pos_drone_fcu[0] + tunnel_point0[0], 
                                                    pos_drone_fcu[1] + tunnel_point0[1], 
                                                    pos_drone_fcu[2] + tunnel_point0[2]);
                        tunnel_before = Eigen::Vector3d(pos_drone_fcu[0] + tunnel_point1[0], 
                                                    pos_drone_fcu[1] + tunnel_point1[1], 
                                                    pos_drone_fcu[2] + tunnel_point1[2]);
                        tunnel_after = Eigen::Vector3d(pos_drone_fcu[0] + tunnel_point2[0], 
                                                    pos_drone_fcu[1] + tunnel_point2[1], 
                                                    pos_drone_fcu[2] + tunnel_point2[2]);                                          
                        aim.pose.position.x = tunnel_0[0];
                        aim.pose.position.y = tunnel_0[1];      
                        aim.pose.position.z = 1.0;
                        local_pos_pub.publish(aim);
                    }
                    /*----------fly towards point before ring----------*/
                    else if(count_tunnel == 6)
                    {
                        aim.pose.position.x = tunnel_0[0];
                        aim.pose.position.y = tunnel_0[1];      
                        aim.pose.position.z = 1.0;
                        local_pos_pub.publish(aim);
                        if( (abs(pos_drone_fcu[0] - tunnel_0[0]) < 0.1) && (abs(pos_drone_fcu[1] - tunnel_0[1]) < 0.1) && (abs(pos_drone_fcu[2] - 1.0) < 0.1) )
                        {
                            count_tunnel = count_tunnel + 1;
                        }
                    }
                    /*----------fly towards point after ring----------*/
                    else 
                    {
                        aim.pose.position.x = tunnel_before[0];
                        aim.pose.position.y = tunnel_before[1];      
                        aim.pose.position.z = 1.0;
                        local_pos_pub.publish(aim);
                        if( (abs(pos_drone_fcu[0] - tunnel_before[0]) < 0.1) && (abs(pos_drone_fcu[1] - tunnel_before[1]) < 0.1) && (abs(pos_drone_fcu[2] - 1.0) < 0.1) )
                        {
                            //count_tunnel = count_tunnel + 1;
                            flag_tunnel = 1;
                        }
                    }
                 }

                if(flag_tunnel == 1)
                {
                    aim.pose.position.x = pos_drone_fcu[0];
                    aim.pose.position.y = pos_drone_fcu[1];      
                    aim.pose.position.z = 0.1;
                    local_pos_pub.publish(aim);
                }
            }
            
            if(flag_D == 1)
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
                if(flag_ring1 == 1)
                {
                    aim.pose.position.x = pos_drone_fcu[0];
                    aim.pose.position.y = pos_drone_fcu[1];      
                    aim.pose.position.z = 0.1;
                    local_pos_pub.publish(aim);
                }
            }



            if(flag_E== 1)
            {    
       // go_to_target(target_pt_pos);
             if( (abs(pos_drone_fcu[0] - target_pt_pos.pose.position.x) > 0.1) || (abs(pos_drone_fcu[1] - target_pt_pos.pose.position.y) > 0.1) || (abs(pos_drone_fcu[2] - target_pt_pos.pose.position.z) > 0.1) )
             {
                aim.pose.position.x = target_pt_pos.pose.position.x;
                aim.pose.position.y =  target_pt_pos.pose.position.y;      
                aim.pose.position.z =  target_pt_pos.pose.position.z;
                // pub p
                local_pos_pub.publish(aim);
             }
             else if ( !OK_to_process )             
             {
                adjust_yaw(target_pt_pos);
                // pub pva
                local_raw_pub.publish(movement);
             }
             else
             {
                /* 识别并执行动作 */
                detect_and_process();  
             }
            }

            if(flag_LAND == 1)
            {

                if(flag_land == 0)
                {
                    pos_drone_now = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], pos_drone_fcu[2]);
                    aim.pose.position.x = pos_drone_now[0];
                    aim.pose.position.y = pos_drone_now[1];      
                    aim.pose.position.z = pos_drone_now[2];
                    local_pos_pub.publish(aim);
                    
                    flag_land = 1;
                }
                
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
            goal.pose.position.x = 3.0;
            goal.pose.position.y = 0.0;
            goal.pose.position.z = 0.3;
               cout<<"fuck"<<endl;
            traj_goal_pub.publish(goal);
            
            cout<<"flag_new_traj="<<flag_new_traj<<endl;

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

        if(cmdd == 3)
        {
        if(traj.size() != 0)
        {
            Eigen::Vector2d ptr;
            mavros_msgs::PositionTarget pva_msg;


            // CAL_YAW
            ptr = *(vel_.begin());
            double arg_ = atan2(ptr[1], ptr[0]);
            double vel_len = sqrt(pow(ptr[0],2)+pow(ptr[1],2));
            if(vel_len<=0.15)
            {
                // auto aim_pose = *(traj.end()-1);
                // double arg_aim = atan2(-( aim_pose(0) -(pos_drone_fcu(0)) ),( aim_pose(1) -(pos_drone_fcu(1)) )) + (PI/2.0f);
                arg_ = last_yaw;
            }
            std::pair<double, double> yaw_all = calculate_yaw(last_yaw,arg_);
            arg_ = yaw_all.first;
            geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(arg_);

        /*   AIM   */
            // POSE
            geometry_msgs::PoseStamped  pos_ptr;
            ptr = *(traj.begin());
            traj.erase(traj.begin());
            pos_ptr.pose.position.x = ptr(0);
            pos_ptr.pose.position.y = ptr(1);
            pos_ptr.pose.position.z = task_3_height;
            aim.pose.position       = pos_ptr.pose.position;
            // YAW
            aim.pose.orientation = geo_q;
            

        /*   PVA   */
        /*
        Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
        Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
        Bit 10 should set to 0, means is not force sp
        */
            // MASK
            pva_msg.type_mask = 0b000111111000;  // 100 111 111 000  xyz + yaw
            // FRAME
            pva_msg.coordinate_frame = 1;
            // TIME
            pva_msg.header.stamp = ros::Time::now();
            // POSE
            pva_msg.position.x = ptr(0);
            pva_msg.position.y = ptr(1);
            pva_msg.position.z = task_3_height;
            // VEL
            ptr = *(vel_.begin());
            vel_.erase(vel_.begin());
            pva_msg.velocity.x = ptr(0);
            pva_msg.velocity.y = ptr(1);
            pva_msg.velocity.z = 0;
            // ACC
            ptr = *(acc_.begin());
            acc_.erase(acc_.begin());
            pva_msg.acceleration_or_force.x = ptr(0);
            pva_msg.acceleration_or_force.y = ptr(1);
            pva_msg.acceleration_or_force.z = 0;
            // YAW
            pva_msg.yaw = arg_;
            pva_msg.yaw_rate = yaw_all.second;

        /*   PUB   */
            local_raw_pub.publish(pva_msg);
        }
        else
        {
            last_yaw     = yaw;
            last_yaw_dot = 0;
            local_pos_pub.publish(aim);
        }
        bsaim_pub.publish(aim);
        // ros::Duration(0.1).sleep();
    }

        fsm_pub.publish(flag_msg);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

double min_max_yaw(int mode, double yaw_)
{
    double yaw_mm = yaw_;
    switch (mode)
    {
    case 1:
    {
        while (yaw_mm >  PI)
        {
            yaw_mm -= 2*PI;
        }
        while (yaw_mm<= -PI)
        {
            yaw_mm += 2*PI;
        }
        break;
    }
        
    case 2:
    {
        while (yaw_mm > 2*PI)
        {
            yaw_mm -= 2*PI;
        }
        while (yaw_mm<= 0)
        {
            yaw_mm += 2*PI;
        }
        break;
    }
    default:
    {
        /* code */
        break;
    }
    }
    return yaw_mm;
}

std::pair<double, double> cal_yaw(double current_yaw,double aim_yaw)
{
    std::pair<double, double> yaw_yawdot(0, 0);
    double next_yaw_ = 0;
    double yawdot = 0;
    double delta_yaw = aim_yaw - current_yaw;
    if( delta_yaw > (PI + DEV ))
    {
        delta_yaw = delta_yaw - 2*PI;
    }
    else if ( delta_yaw < -(PI + DEV) )
    {
        delta_yaw = delta_yaw + 2*PI;
    }
    else
    {

    }
    // split
    if( delta_yaw>YAW_MAX )
    {
        // next_yaw_ = 
    }
    else if( delta_yaw<YAW_MAX )
    {

    }


    return yaw_yawdot;
}

std::pair<double, double> calculate_yaw( double aim_yaw,double current_yaw)
{
    // cout<<current_yaw<<",,,,,,"<<aim_yaw<<endl;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw_ = 0;
  double yawdot = 0;
  if (aim_yaw - current_yaw > PI)
  {
    // cout<<"case1"<<endl;
    if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
    {
        // cout<<"case11"<<endl;
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else
    {
        // cout<<"case12"<<endl;
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else if (aim_yaw - current_yaw < -PI)
  {
    // cout<<"case2"<<endl;
    if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
    {
        // cout<<"case21"<<endl;
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
        // cout<<"case22"<<endl;
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else
  {
    // cout<<"case3"<<endl;
    if (aim_yaw - current_yaw < -YAW_MAX)
    {
        // cout<<"case31"<<endl;
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else if (aim_yaw - current_yaw > YAW_MAX)
    {
        // cout<<"case32"<<endl;
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
        // cout<<"case33"<<endl;
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  if (fabs(yaw_ - last_yaw) <= YAW_MAX)
  yaw_ = 0.5 * last_yaw + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
  last_yaw = yaw_;  
  last_yaw_dot = yawdot;
  yaw_yawdot.first = yaw_;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}



/* yaw:  0 deg      1      0      0      0
        90 deg    0.707    0      0    0.707
       180 deg      0      0      0      1
       -90 deg    0.707    0      0   -0.707    */
