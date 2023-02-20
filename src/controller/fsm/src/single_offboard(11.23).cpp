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
#include "bspline_race/BsplineTraj.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/Int64.h"

using namespace std;

static mavros_msgs::State current_state;
static int flight_mode=1;
static int waypoint_num=0;
static int cmdd=0;
static int bt=1;
static int btt=0;
static int flag1=0;
static int flag2=0;
static int ready_fly=0;
static double waypoint[50][3];
static double ringpoint[50][3];
static double ringdirect[50][3];
static double zhongzhi[50];


static geometry_msgs::PoseStamped aim;
static mavros_msgs::PositionTarget target;

static geometry_msgs::PoseStamped goal;
Eigen::Vector3d pos_drone_fcu;
Eigen::Vector3d vel_drone_fcu;
Eigen::Vector4d orient_drone_fcu;

fsm::command_acc bt_wp;
fsm::command_acc bt_fsm_wp;

/*----------ROS DEFINE----------*/
ros::Subscriber state_sub,
                position_sub,
                vel_sub,
                odom_sub,
                ready,
                ring_sub,
                pcl_ring_sub,
                traj_sub;
ros::Publisher traj_goal_pub,
               local_pos_pub,
               local_target_pub,
               pub_ascup_flag,
               bs_pub,
               debug_pub,
               vis_path_pub;

ros::ServiceClient arming_client,
                   set_mode_client;
/*----------ASCUP----------*/

std_msgs::Int64 ascup_flag;      //比赛任务标志位

int flag_A = 0;                              // 0: disable mission A debug       1: enable mission A debug
int flag_B = 0;                              // 0: disable mission B debug       1: enable mission B debug
int flag_C = 0;                              // 0: disable mission C debug       1: enable mission C debug
int flag_D = 1;                              // 0: disable mission D debug       1: enable mission D debug
int flag_E = 0;                              // 0: disable mission E debug       1: enable mission E debug
int flag_LAND = 0;                           // 0: disable mission LAND debug    1: enable mission LAND debug  

int flag_land = 0;  
int circle_flag=1;

/*----------ASCUP----------*/

Eigen::Vector3d pos_drone_now;
Eigen::Vector3d pos_drone_temp;
int search_direct = 1;                           // 0: left    1: right
double yaw_search = 0;                             
int rotate_direct = 1;                           // 0: left    1: right

bspline_race::BsplineTraj traj_fsm;
int traj_len;
int traj_p = 0;
int flag_new_traj = 1;

/*------------------------------------------------------ring----------------------------------------------------------------------*/
int COUNT_RING = 3;
double LAMDA_RING = 1.0;
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
int count_ring = 0;

//Eigen::Vector3d ring_point1;
//Eigen::Vector3d ring_point2;
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
/*------------------------------------------------------ring----------------------------------------------------------------------*/

/*-------PLANNING--------*/
//BUG
#define PI acos(-1)

int connect_seq = 0;            // 当前时间序列
bool first_bs = true;           // 第一次轨迹不做衔接
bool arrived = false;           // 到达目标点
bool finish_plan = 0;
bool first_reach_C = true;      // ASCUP 留下的状态
double roll, pitch,yaw;         // 无人机真实欧拉角（咋被删了。。。
double last_yaw;                // yaw 平滑                
double last_yaw_dot;            // yaw_rate 平滑
double output_yaw;              // 输出yaw角
double output_d_yaw;            // 输出yaw速度 
// param 
double set_height;       // 二维规划设置的无人机飞行高度
double delta_T;          // 运行频率       
int seq_interval;        // 重规划半径(点数)
double seq_interval_time;// 重规划半径(时间)
double D_YAW_MAX;        // 最大yaw转速
double YAW_MAX;          // 单次最大yaw角  
mavros_msgs::PositionTarget pva_msg;  // pva控制信息
mavros_msgs::PositionTarget bs_msg;     // 规划信息
geometry_msgs::PoseStamped debug_msg; // debug信息
nav_msgs::Path vis_path;              // 发布过的控制信息可视化
class bs_traj
{
    public:
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d acc_;
        int seq_;
};
std::vector<bs_traj> BTraj;     // 存储轨迹
void run_planning(); // 规划主体函数
void circle();
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw);
/*-------PLANNING--------*/

/*----------UDP----------*/
static geometry_msgs::PoseStamped aimm;
static bool isAir;
/*----------UDP----------*/

void UdpServer(const char* ip,const uint16_t cport,const int UAVID)
{
    ROS_WARN("UDP %d",UAVID);
    std::string ss;
    geometry_msgs::PoseStamped offset;
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        ROS_ERROR("Network Error");
        return;
    }
    struct sockaddr_in addr_client;
    int len;
    memset(&addr_client, 0, sizeof(struct sockaddr_in));
    addr_client.sin_family = AF_INET;
    addr_client.sin_addr.s_addr = inet_addr(ip);
    addr_client.sin_port = htons(cport);
    len = sizeof(addr_client);

    int sdlen;
    int send_num;
    char send_buf[100];

    ros::Rate rate_udp(10.0);

    while(ros::ok())
    {
        memset(&send_buf, 0, sizeof(send_buf));
        sdlen=0;

        // if(btt==1)
        // {
        //     bt=1;
        // }

        //sprintf(send_buf+sdlen,"%d",cmdd>=9?9:cmdd);
        sprintf(send_buf+sdlen,"%d",cmdd);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.x+offset.pose.position.x);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.y+offset.pose.position.y);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%.3lf",aim.pose.position.z+offset.pose.position.z);
        sdlen=int(strlen(send_buf));
        send_buf[sdlen]   = ',';
        send_buf[sdlen+1] = '\0';
        sdlen=int(strlen(send_buf));
        sprintf(send_buf+sdlen,"%d",bt);

        send_num = sendto(sock_fd, send_buf, ssize_t(strlen(send_buf)), 0, (struct sockaddr *)&addr_client, len);


        if(send_num < 0)
        {
            ROS_ERROR("Send Fail!, UAV = %d",UAVID);
        }
        rate_udp.sleep();
    }

}

void ring_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
     ring_center_temp = Eigen::Vector3d(msg->pose.position.z, -msg->pose.position.x, -msg->pose.position.y);
     ring_direct_temp = Eigen::Vector3d(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y);
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

void circle()
{
                    if(count_ring1 <6)
                    {
                        if(flag_ring_recog == 1)
                        {

                        ringpoint[count_ring1][0]=ring_center_temp[0];
                        ringpoint[count_ring1][1]=ring_center_temp[1];
                        ringpoint[count_ring1][2]=ring_center_temp[2];

                        ringdirect[count_ring1][0]=ring_direct_temp[0];
                        ringdirect[count_ring1][1]=ring_direct_temp[1];
                        ringdirect[count_ring1][2]=ring_direct_temp[2];

                        zhongzhi[count_ring1]=pow(ringpoint[count_ring1][0],2)+pow(ringpoint[count_ring1][1],2)+pow(ringpoint[count_ring1][2],2);
                        
                        count_ring1 = count_ring1 + 1;
                        flag_ring_recog = 0;
                        ROS_INFO("Recognize!");
                        }
                        local_pos_pub.publish(aim);
                    }
                    /*----------determine point before and after ring----------*/
                    else if(count_ring1 == 6)
                    {
                        int z=0,y[5],x;
                        for(z=0;z<=6;z++)
                        {
                            y[z]=z;
                        }
                        for(z=0;z<=6;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=5;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=4;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=3;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }
                        count_ring1 = count_ring1 + 1;
                        int zhong=y[z];

                        ring_point1_temp[0]=ringpoint[y[3]][0] -LAMDA_RING * ringdirect[y[3]][0];
                        ring_point1_temp[1]=ringpoint[y[3]][1] -LAMDA_RING * ringdirect[y[3]][1];
                        ring_point1_temp[2]=ringpoint[y[3]][2] -LAMDA_RING * ringdirect[y[3]][2];

                        ring_point2_temp[0]=ringpoint[y[3]][0] +LAMDA_RING * ringdirect[y[3]][0];
                        ring_point2_temp[1]=ringpoint[y[3]][1] +LAMDA_RING * ringdirect[y[3]][1];
                        ring_point2_temp[2]=ringpoint[y[3]][2] +LAMDA_RING * ringdirect[y[3]][2];

                        //ring_point1 equal 0
                        ring_before = Eigen::Vector3d(pos_drone_fcu[0]+ring_point1_temp[0], 
                                                    pos_drone_fcu[1]+ring_point1_temp[1], 
                                                    pos_drone_fcu[2]+ring_point1_temp[2]);
                        ring_after = Eigen::Vector3d(pos_drone_fcu[0]+ring_point2_temp[0], 
                                                    pos_drone_fcu[1]+ring_point2_temp[1], 
                                                    pos_drone_fcu[2]+ring_point2_temp[2]);      
                                                                                       
                        goal.pose.position.x = ring_before[0];
                        goal.pose.position.y = ring_before[1];      
                        goal.pose.position.z = ring_before[2];
                        traj_goal_pub.publish(goal);

                        goal.pose.position.x = ring_after[0];
                        goal.pose.position.y = ring_after[1];      
                        goal.pose.position.z = ring_after[2];
                          traj_goal_pub.publish(goal);

                        ROS_INFO("Ring: %d, %.3lf, %.3lf, %.3lf",cmdd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
                        ROS_INFO("Direct: %d, %.3lf, %.3lf, %.3lf",cmdd,ring_after[0],ring_after[1],ring_after[2]);

                    }
                    /*----------fly towards point before ring----------*/
                    else
                    {
                          if(finish_plan==1)
                        {
                            
                            finish_plan=0;
                            circle_flag=0;
                            count_ring1=0;
                        }
                    }
}

// PLANNING
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
    vel_drone_fcu = Eigen::Vector3d(msg->twist.linear.x , msg->twist.linear.y, msg->twist.linear.z);
}
void traj_cb(const bspline_race::BsplineTrajConstPtr &msg)
{
  // 收到新轨迹
  if(first_bs || BTraj.empty())
  {
    // 直接放到BTraj里
    for (size_t i = 0; i < msg->position.size(); i++)
    {    
        bs_traj BT_ptr;
        BT_ptr.pos_ << msg->position[i].pose.position.x, 
                       msg->position[i].pose.position.y,
                       msg->position[i].pose.position.z;
        BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
                       msg->velocity[i].pose.position.y,
                       msg->velocity[i].pose.position.z;
        BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
                       msg->acceleration[i].pose.position.y,
                       msg->acceleration[i].pose.position.z;
        BT_ptr.seq_ = i;
        BTraj.push_back(BT_ptr);
    }
    first_bs = false;
  }
  else
  {
    // 轨迹拼接
    int new_traj_start_seq = msg->current_seq ;
    int delta_seq = new_traj_start_seq - BTraj[0].seq_;
    if(delta_seq<0)// 如果当前发过来的轨迹太旧
    {
      ROS_WARN("The delay is too large < %i > points away, < %f > ms ago, check the parameter Settings."
                ,-delta_seq, -delta_seq*delta_T*1000);
    //   // FIXME 需要从新轨迹上的对应时刻取点
    //   if( BTraj[0].seq_ < (msg->current_seq + msg->position.size()))
    //   {
    //     auto bbegin = *(BTraj.begin());
    //     BTraj.clear();
    //     BTraj.push_back(bbegin);
    //     for (size_t i = -delta_seq+1; i < msg->position.size(); i++)
    //     {    
    //         bs_traj BT_ptr;
    //         BT_ptr.pos_ << msg->position[i].pose.position.x, 
    //                        msg->position[i].pose.position.y,
    //                        msg->position[i].pose.position.z;
    //         BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
    //                        msg->velocity[i].pose.position.y,
    //                        msg->velocity[i].pose.position.z;
    //         BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
    //                        msg->acceleration[i].pose.position.y,
    //                        msg->acceleration[i].pose.position.z;
    //         BT_ptr.seq_ = i + new_traj_start_seq;
    //         BTraj.push_back(BT_ptr);
    //     }
    //   }
    //   else
    //   {
        ROS_ERROR("USELESS.");
    //   }
    }
    else if(delta_seq>BTraj.size())// 如果这个新轨迹的起点超过了当前剩余曲线的终点
    {
      ROS_WARN("The scope of replanning is too large, check the parameter Settings.");
      ROS_ERROR("Wait, something really big has happened...");
    }
    else// 终于没事了
    {
      std::vector<bs_traj> BTraj_remain;
      auto start_ptr_ = BTraj.begin();
      // auto end_ptr_   = BTraj.begin() + delta_seq;// BUG ?悬垂指针？
      int add_seq_;
      if(delta_seq < (BTraj.size()-1))  
      {         
          add_seq_  = delta_seq-1;
        //   cout << "delta_seq\n"<<delta_seq<<endl;
      }
      else
      {
        cout << "悬垂指针？"<<endl;
        add_seq_ = (BTraj.size()-1);
      }
      auto end_ptr_   = BTraj.begin() + add_seq_;
      if(add_seq_>0)
      BTraj_remain.assign(start_ptr_,end_ptr_);// 存储剩余的可用路径
      int old_traj_end_seq = end_ptr_->seq_;
      (*(BTraj_remain.end()-1)).seq_;
      for (size_t i = 0; i < msg->position.size(); i++)
      {    
          bs_traj BT_ptr;
          BT_ptr.pos_ << msg->position[i].pose.position.x, 
                         msg->position[i].pose.position.y,
                         msg->position[i].pose.position.z;
          BT_ptr.vel_ << msg->velocity[i].pose.position.x, 
                         msg->velocity[i].pose.position.y,
                         msg->velocity[i].pose.position.z;
          BT_ptr.acc_ << msg->acceleration[i].pose.position.x, 
                         msg->acceleration[i].pose.position.y,
                         msg->acceleration[i].pose.position.z;
          BT_ptr.seq_ = i + new_traj_start_seq;
          BTraj_remain.push_back(BT_ptr);
      }
      BTraj = BTraj_remain;
      cout << "Successfully connect the trajectory at < "<<old_traj_end_seq
                                           <<" > ++++ < "<<new_traj_start_seq<<" >."<<endl;
                
    }
  }
  ROS_INFO("New seq begin at: < %i >, end at: < %i >.",(*(BTraj.begin())).seq_,(*(BTraj.end()-1)).seq_);
}

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

void Set_StaticSearch(double y_max, double dy)
{
    if(search_direct == 1) {pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0], pos_drone_temp[1] - dy, pos_drone_temp[2]);}
    else                   {pos_drone_temp = Eigen::Vector3d(pos_drone_temp[0], pos_drone_temp[1] + dy, pos_drone_temp[2]);}

    if( (pos_drone_temp[1] + y_max) < 0) {search_direct = 0;}
    if(pos_drone_temp[1] > 0) {search_direct = 1;}
}

void Set_TargetYaw(double yaw_)
{
    target.type_mask = 0b100111111000;
    target.coordinate_frame = 1;

    target.position.x = aim.pose.position.x;
    target.position.y = aim.pose.position.y;
    target.position.z = aim.pose.position.z;
    
    if(abs(target.yaw*180.0/PI - yaw_) >= 3.0)
    {
        if(yaw_ > target.yaw) {target.yaw = target.yaw + PI/120;}
        else                 {target.yaw = target.yaw - PI/120;}
    }
    else {target.yaw = yaw_*PI/180.0;}
    
}

void Set_YawSearch()
{
    if(rotate_direct == 1) {yaw_search = yaw_search - 1.5;}
    else                   {yaw_search = yaw_search + 1.5;}

    if( (yaw_search + 60.0) < 0) {rotate_direct = 0;}
    if( (yaw_search - 60.0) > 0) {rotate_direct = 1;}
}

double QuaterToYaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0*(w*z + x*y);
    double cosy_cosp = 1.0 - 2.0*(y*y + z*z);
    double yaw_ = atan2(siny_cosp, cosy_cosp);
    double yaw_angle = yaw_*180.0/PI;
    return yaw_angle;
}


/*-------------------------*/

void ready_cb(const fsm::command_acc &msg)
{ 
    ready_fly=msg.ready;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_fcu = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    orient_drone_fcu = Eigen::Vector4d(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
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

        //if(recv_num < 0||abs(recv_num-19)>3)
        if(recv_num < 0)
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
        // p=strtok(NULL,dot);
        // sscanf(p,"%d",&bt);
        if(cmd!=6  &&  cmd!=7 && cmd!=3 && cmd!=31 )
        {
          aim.pose.position.x=px;
          aim.pose.position.y=py;
          aim.pose.position.z=pz;
        }
       //ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf, %d",cmd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z, bt);
       ROS_INFO("Rec: %d, %.3lf, %.3lf, %.3lf",cmd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);

        if(cmdd!=cmd) 
        {
            btt=0;
        }
        cmdd=cmd;

        //  switch (cmdd)
        // {
        // case 0:
        //     if(ready_fly==1)
        //     cout<<"Ready Fly"<<endl;
        //     else
        //     ROS_INFO("Waiting");
        //     break;
        // }

    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;
    // PARAMS
    nh.param("single_offboard/delta_T" , delta_T, 0.02);
    nh.param("single_offboard/set_height" , set_height, 1.2);
    nh.param("single_offboard/yaw_rate_max", D_YAW_MAX, PI/4);
    nh.param("single_offboard/seq_interval_time", seq_interval_time, 0.4);
    
    YAW_MAX = D_YAW_MAX * delta_T;
    seq_interval = seq_interval_time/delta_T;

    state_sub = nh.subscribe<mavros_msgs::State>   
            ("mavros/state", 10, state_cb);
    position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 100, pos_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_local",100,vel_cb);
        ready= nh.subscribe("/px4/ready",10,ready_cb);
    /*----------圆环检测----------*/
    ring_sub = nh.subscribe<geometry_msgs::PoseStamped>          //AB圆环检测
            ("/circle_pose", 100, ring_cb);
    pcl_ring_sub = nh.subscribe<geometry_msgs::PoseStamped>  //点云圆环识别
            ("/test_ring_msg", 100, pcl_ring_cb);
    /*-------------------------------*/
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>  
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>   
            ("mavros/set_mode");
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>    
            ("mavros/setpoint_position/local", 10);
    local_target_pub = nh.advertise< mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    pub_ascup_flag = nh.advertise<std_msgs::Int64>  //发布比赛任务标志位
            ("flag_detect", 10);
    /*---------规划--------*/
    traj_sub = nh.subscribe<bspline_race::BsplineTraj>       //路径规划，接收B样条的点
            ("/bspline_traj", 10 ,traj_cb);
    traj_goal_pub = nh.advertise<geometry_msgs::PoseStamped>  //路径规划，发送目标点
            ("/move_base_simple/goal", 100);
    bs_pub    = nh.advertise<mavros_msgs::PositionTarget>
            ("/fsm/planning_start" , 1);
    debug_pub    = nh.advertise<geometry_msgs::PoseStamped>
            ("/debug",1);
    vis_path_pub = nh.advertise<nav_msgs::Path>
            ("/pubed_path",1);
    /*---------PLANNING_END--------*/
    ros::Rate rate(1/delta_T);
    /*-------------------------------------------------launch文件中读取导航点-----------------------------------------------------------------------*/
    nh.param("/single_offboard/way_num",waypoint_num,0);
    aim.pose.position.x = 0;
    aim.pose.position.y = 0;
    aim.pose.position.z = 0.7;
    //point
    for (int i = 1; i <= waypoint_num; i++)
    {
      nh.param("/single_offboard/waypoint" + to_string(i) + "_x", waypoint[i][0], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_y", waypoint[i][1], -1.0);
      nh.param("/single_offboard/waypoint" + to_string(i) + "_z", waypoint[i][2], -1.0);
    }
    /*-------------------------------------------------launch文件中读取导航点-----------------------------------------------------------------------*/

    new std::thread(&UdpListen,12001);
    new std::thread(&UdpServer,"127.0.0.1",12002,1);

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

    ascup_flag.data = 0;

    /*-------------------------*/
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(aim);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        pub_ascup_flag.publish(ascup_flag);
        //run_planning();
//        if(cmdd!=4) local_pos_pub.publish(aim);
        if(cmdd==1)
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
            local_pos_pub.publish(aim);
            // if( (abs(pos_drone_fcu[0] - 0) < 0.05) && (abs(pos_drone_fcu[1] - 0) < 0.05) && (abs(pos_drone_fcu[2] - 0.8) < 0.05) )
            //         {
            //              btt=1;
            //         }
            static int j=50;
            j--;
            if(j<2&btt==0) 
            {
                btt=1;
                bt=-bt;
            }
 
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
        if(cmdd == 31)
        {
            run_planning();
        }
        if(cmdd==3)
        {   
            run_planning();
            // TEST
            //run_planning();// FIXME
            if(circle_flag==1)    circle();
            
            else
            {
                 //circle_flag=1;
                static int ccc=0;
                if(ccc==0)
                {
                        goal.pose.position.x = 11.0;
                        goal.pose.position.y = 0.0;      
                        goal.pose.position.z = 0.8;
                        traj_goal_pub.publish(goal);
                        ccc=1;
                }
                // btt=1;
                // bt=-bt;
                // cmdd=4;

            // if(abs(pos_drone_fcu[2] - 0.1) < 0.05) {flag1 = 1;}
            // if(flag1 == 0)
            // {
            //     aim.pose.position.x = pos_drone_fcu[0];
            //     aim.pose.position.y = pos_drone_fcu[1];      
            //     aim.pose.position.z = 0.1;
            //     local_pos_pub.publish(aim);
            // }
            // else
            // {
            //     if(current_state.mode == "OFFBOARD")
            //     {
            //         mode_cmd.request.custom_mode = "Hold";
            //         set_mode_client.call(mode_cmd);
            //         ROS_ERROR("1111");
            //     }
            //     if(current_state.armed)
            //     {
            //         arm_cmd.request.value = false;
            //         arming_client.call(arm_cmd);
            //         ROS_ERROR("2222");
            //     }
            //     if (arm_cmd.response.success)
            //     {
            //         ROS_ERROR("Disarm successfully!");
            //     }
            // }
                
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
                ROS_INFO("%f,%f,%f",pos_drone_fcu[0],pos_drone_fcu[1],pos_drone_fcu[2]);

            if( (abs(pos_drone_fcu[0] - aim.pose.position.x) < 0.1) && (abs(pos_drone_fcu[1] - aim.pose.position.y) < 0.1) && (abs(pos_drone_fcu[2] - aim.pose.position.z) < 0.1) && btt==0)
                        {
                          btt=1;
                          bt=-bt;
                        }            
        }
    
                    if(cmdd==71)
        {
            aim.pose.position.x =  waypoint[1][0];
            aim.pose.position.y =  waypoint[1][1];      
            aim.pose.position.z =  waypoint[1][2];
            local_pos_pub.publish(aim);

            if( (abs(pos_drone_fcu[0] - aim.pose.position.x) < 0.1) && (abs(pos_drone_fcu[1] - aim.pose.position.y) < 0.1) && (abs(pos_drone_fcu[2] - aim.pose.position.z) < 0.1) && btt==0)
                        {
                          btt=1;
                          bt=-bt;
                        }            
        }

                    if(cmdd==72)
        {
            aim.pose.position.x =  waypoint[2][0];
            aim.pose.position.y =  waypoint[2][1];      
            aim.pose.position.z =  waypoint[2][2];
            local_pos_pub.publish(aim);

            if( (abs(pos_drone_fcu[0] - aim.pose.position.x) < 0.1) && (abs(pos_drone_fcu[1] - aim.pose.position.y) < 0.1) && (abs(pos_drone_fcu[2] - aim.pose.position.z) < 0.1) && btt==0)
                        {
                          btt=1;
                          bt=-bt;
                        }            
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
                // if(flag_wall == 7)
                // {
                //     flag_B = 1;
                //     flag_A = 0;
                // }
            }

            /*----------mission B----------*/
            if(flag_B == 1)
            {    
      
            }
            /*----------mission C---------*/
            if(flag_C == 1)
            {

            }   
            /*----------mission D---------*/
            if(flag_D == 1)
            {    
                if(flag_ring1 == 0)
                {
                    
                    
                    /*----------check ring detection----------*/
                    if(count_ring1 <6)
                    {
                        if(flag_ring_recog == 1)
                        {

                        ringpoint[count_ring1][0]=ring_center_temp[0];
                        ringpoint[count_ring1][1]=ring_center_temp[1];
                        ringpoint[count_ring1][2]=ring_center_temp[2];

                        ringdirect[count_ring1][0]=ring_direct_temp[0];
                        ringdirect[count_ring1][1]=ring_direct_temp[1];
                        ringdirect[count_ring1][2]=ring_direct_temp[2];

                        zhongzhi[count_ring1]=pow(ringpoint[count_ring1][0],2)+pow(ringpoint[count_ring1][1],2)+pow(ringpoint[count_ring1][2],2);
                        
                        count_ring1 = count_ring1 + 1;
                        flag_ring_recog = 0;
                        ROS_INFO("Recognize!");
                        }
                        local_pos_pub.publish(aim);
                    }
                    /*----------determine point before and after ring----------*/
                    else if(count_ring1 == 6)
                    {
                        int z=0,y[5],x;
                        for(z=0;z<=6;z++)
                        {
                            y[z]=z;
                        }
                        for(z=0;z<=6;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=5;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=4;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }

                        for(z=0;z<=3;z++)
                        {
                            if(zhongzhi[z+1]<zhongzhi[z])
                                {
                                    x=zhongzhi[z];
                                    zhongzhi[z+1]=x;
                                    zhongzhi[z]=zhongzhi[z+1];

                                    x=y[z];
                                    y[z+1]=x;
                                    y[z]=y[z+1];
                                }
                        }
                        count_ring1 = count_ring1 + 1;
                        int zhong=y[z];

                        ring_point1_temp[0]=ringpoint[y[3]][0] -LAMDA_RING * ringdirect[y[3]][0];
                        ring_point1_temp[1]=ringpoint[y[3]][1] -LAMDA_RING * ringdirect[y[3]][1];
                        ring_point1_temp[2]=ringpoint[y[3]][2] -LAMDA_RING * ringdirect[y[3]][2];

                        ring_point2_temp[0]=ringpoint[y[3]][0] +LAMDA_RING * ringdirect[y[3]][0];
                        ring_point2_temp[1]=ringpoint[y[3]][1] +LAMDA_RING * ringdirect[y[3]][1];
                        ring_point2_temp[2]=ringpoint[y[3]][2] +LAMDA_RING * ringdirect[y[3]][2];

                        //ring_point1 equal 0
                        ring_before = Eigen::Vector3d(pos_drone_fcu[0]+ring_point1_temp[0], 
                                                    pos_drone_fcu[1]+ring_point1_temp[1], 
                                                    pos_drone_fcu[2]+ring_point1_temp[2]);
                        ring_after = Eigen::Vector3d(pos_drone_fcu[0]+ring_point2_temp[0], 
                                                    pos_drone_fcu[1]+ring_point2_temp[1], 
                                                    pos_drone_fcu[2]+ring_point2_temp[2]);      
                                                                                       
                        aim.pose.position.x = ring_before[0];
                        aim.pose.position.y = ring_before[1];      
                        aim.pose.position.z = ring_before[2];
                        local_pos_pub.publish(aim);
                        ROS_INFO("Ring: %d, %.3lf, %.3lf, %.3lf",cmdd,aim.pose.position.x,aim.pose.position.y,aim.pose.position.z);
                        ROS_INFO("Direct: %d, %.3lf, %.3lf, %.3lf",cmdd,ring_after[0],ring_after[1],ring_after[2]);

                    }
                    /*----------fly towards point before ring----------*/
                    else if(count_ring1 == 7)
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
                    aim.pose.position.z = 0.8;
                    local_pos_pub.publish(aim);
                }
            }

            /*----------mission E----------*/
            if(flag_E == 1)
            {       
                /*----------init state----------*/
                if(flag_ring == 0)
                {
                    pos_drone_now = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], 1.0);
                    pos_drone_temp = Eigen::Vector3d(0.0, 0.0, 0.0);
                    yaw_search = 0.0;
                    search_direct = 1;
                    rotate_direct = 1;
                    count_ring = 0;
                    last_recog = ros::Time::now();
                    ascup_flag.data = 5;

                    flag_ring = 1;
                    
                }
               
                /*----------wait and check ring recognition----------*/
                if( (flag_ring == 1) || (flag_ring == 6) || (flag_ring == 11) )
                {
                    Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                    Set_TargetYaw(yaw_search);
		            local_target_pub.publish(target);
                    ROS_INFO("Rec: %d, flag: %d, aim: %.3lf, %.3lf, %.3lf, %.3lf", cmdd, flag_ring, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z, yaw_search);
                    ROS_INFO("Rec: %d, recog: %d, count: %d, yaw: %.3lf", cmdd, flag_ring_recog, count_ring, yaw_search);
                    
                    /*----------check ring recognition----------*/
                    if(count_ring < COUNT_RING)
                    {
                        if(flag_ring_recog == 1)
                        {
                            count_ring = count_ring + 1;
                            last_recog = ros::Time::now();
                            flag_ring_recog = 0;
                        }

                        /*----------no recognition during 3s as wrong----------*/
                        if( ros::Time::now() - last_recog > ros::Duration(5.0) ) 
                        {
                            count_ring = 0;
                            flag_ring = flag_ring + 1;
                        }
                    }
                    
                    /*----------COUNT_RING times recognition as successful----------*/
                    else {flag_ring = flag_ring + 2;}
                }

                /*----------search ring----------*/
                if( (flag_ring == 2) || (flag_ring == 7) || (flag_ring == 12) )
                {    
                    Set_YawSearch();
                    Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                    Set_TargetYaw(yaw_search);
                    local_target_pub.publish(target);
                    ROS_INFO("Rec: %d, flag: %d, %.3lf, %.3lf, %.3lf, %.3lf", cmdd, flag_ring, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z, yaw_search);
                    ROS_INFO("Rec: %d, recog: %d, count: %d, yaw: %.3lf", cmdd, flag_ring_recog, count_ring, yaw_search);

                    /*----------wait every 15 deg----------*/
                    count_ring = count_ring + 1;
                    if(count_ring == 10)
                    {
                        count_ring = 0;
                        flag_ring = flag_ring - 1; 
                        last_recog = ros::Time::now();   
                    }
                }

                /*----------determine point before and after tunnel----------*/
                if( (flag_ring == 3) || (flag_ring == 8) || (flag_ring == 13) )
                {       
                    ring_center = pos_drone_fcu + ring_delta;
                    ring_before = ring_center - ring_direct*LAMDA_RING;
                    ring_after = ring_center + ring_direct*LAMDA_RING;

                    flag_ring = flag_ring + 1;
                    pos_drone_now = Eigen::Vector3d(pos_drone_fcu[0], pos_drone_fcu[1], pos_drone_fcu[2]);
                }

                /*----------fly towards point before tunnel----------*/
                if( (flag_ring == 4) || (flag_ring == 9) || (flag_ring == 14) )
                {    
                    if(abs(ring_before[2] - pos_drone_now[2]) > 0.05)
                    {
                        if(ring_before[2] > pos_drone_now[2]) {pos_drone_now = Eigen::Vector3d(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2] + 0.02);}
                        else                                  {pos_drone_now = Eigen::Vector3d(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2] - 0.02);}

                        Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                        local_pos_pub.publish(aim);
                    }
                    else
                    {
                        if(abs(ring_before[1] - pos_drone_now[1]) > 0.05)
                        {
                            if(ring_before[1] > pos_drone_now[1]) {pos_drone_now = Eigen::Vector3d(pos_drone_now[0], pos_drone_now[1] + 0.02, pos_drone_now[2]);}
                            else                                  {pos_drone_now = Eigen::Vector3d(pos_drone_now[0], pos_drone_now[1] - 0.02, pos_drone_now[2]);}
                            
                            Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                            local_pos_pub.publish(aim);
                        }
                        else
                        {
                            if(abs(ring_before[0] - pos_drone_now[0]) > 0.05)
                            {
                                if(ring_before[0] > pos_drone_now[0]) {pos_drone_now = Eigen::Vector3d(pos_drone_now[0] + 0.02, pos_drone_now[1], pos_drone_now[2]);}
                                else                                  {pos_drone_now = Eigen::Vector3d(pos_drone_now[0] - 0.02, pos_drone_now[1], pos_drone_now[2]);}
                                
                                Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                                local_pos_pub.publish(aim);
                            }
                            else
                            {
                                Set_PosAim(pos_drone_now[0], pos_drone_now[1], pos_drone_now[2]);
                                local_pos_pub.publish(aim);
                            }
                        }
                    }
                    
                    ROS_INFO("Rec: %d, flag: %d, %.3lf, %.3lf, %.3lf", cmdd, flag_ring, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);
                    ROS_INFO("Rec: %d, before: %.3lf, %.3lf, %.3lf, after: %.3lf, %.3lf, %.3lf", cmdd, ring_before[0], ring_before[1], ring_before[2], ring_after[0], ring_after[1], ring_after[2]);

                    if( (abs(pos_drone_fcu[0] - ring_before[0]) < 0.05) && (abs(pos_drone_fcu[1] - ring_before[1]) < 0.05) && (abs(pos_drone_fcu[2] - ring_before[2]) < 0.05) )
                    {
                        flag_ring = flag_ring + 1;
                    }
                }

                /*----------fly towards point after tunnel----------*/
                if( (flag_ring == 5) || (flag_ring == 10) || (flag_ring == 15) )
                {
                    Set_PosAim(ring_after[0], ring_after[1], ring_after[2]);
                    local_pos_pub.publish(aim);
                    ROS_INFO("Rec: %d, flag: %d, %.3lf, %.3lf, %.3lf", cmdd, flag_ring, aim.pose.position.x, aim.pose.position.y, aim.pose.position.z);
                    ROS_INFO("Rec: %d, before: %.3lf, %.3lf, %.3lf, after: %.3lf, %.3lf, %.3lf", cmdd, ring_before[0], ring_before[1], ring_before[2], ring_after[0], ring_after[1], ring_after[2]);


                    if( (abs(pos_drone_fcu[0] - ring_after[0]) < 0.05) && (abs(pos_drone_fcu[1] - ring_after[1]) < 0.05) && (abs(pos_drone_fcu[2] -ring_after[2]) < 0.05) )
                    {
                        Set_TargetYaw(0.0);
                        local_target_pub.publish(target);

                        if(abs(QuaterToYaw(orient_drone_fcu[0], orient_drone_fcu[1], orient_drone_fcu[2], orient_drone_fcu[3]) - 0.0) < 3.0)
                        {
                            pos_drone_now = Eigen::Vector3d(ring_after[0], ring_after[1], ring_after[2]);
                            pos_drone_temp = Eigen::Vector3d(0.0, 0.0, 0.0);
                            yaw_search = 0.0;

                            count_ring = 0;
                            flag_ring = flag_ring + 1;
                        }
                    }
                }

                if(flag_ring == 16)
                {
                    Set_PosAim(ring_after[0], ring_after[1], 0.1);
                    local_pos_pub.publish(aim);
                }
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
std::pair<double, double> calculate_yaw( double current_yaw,double aim_yaw)
{
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw_ = 0;
  double yawdot = 0;
  if (aim_yaw - current_yaw > PI)
  {
    if (aim_yaw - current_yaw - 2 * PI < -YAW_MAX)
    {
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else if (aim_yaw - current_yaw < -PI)
  {
    if (aim_yaw - current_yaw + 2 * PI > YAW_MAX)
    {
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
  else
  {
    if (aim_yaw - current_yaw < -YAW_MAX)
    {
      yaw_ = current_yaw - YAW_MAX;
      if (yaw_ < -PI)
        yaw_ += 2 * PI;

      yawdot = -D_YAW_MAX;
    }
    else if (aim_yaw - current_yaw > YAW_MAX)
    {
      yaw_ = current_yaw + YAW_MAX;
      if (yaw_ > PI)
        yaw_ -= 2 * PI;

      yawdot = D_YAW_MAX;
    }
    else
    {
      yaw_ = aim_yaw;
      if (yaw_ - current_yaw > PI)
        yawdot = -D_YAW_MAX;
      else if (yaw_ - current_yaw < -PI)
        yawdot = D_YAW_MAX;
      else
        yawdot = (aim_yaw - current_yaw) /delta_T;
    }
  }
//     if (fabs(yaw_ - last_yaw) <= YAW_MAX)
//     yaw_ = 0.5 * last_yaw + 0.5 * yaw_; // nieve LPF
//   yawdot = 0.5 * last_yaw_dot + 0.5 * yawdot;
  last_yaw = yaw_;  
  last_yaw_dot = yawdot;
  yaw_yawdot.first = yaw_;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void run_planning()
{
  if(BTraj.size() != 0)
        {
            bs_traj BT_ptr = *(BTraj.begin());
            BTraj.erase(BTraj.begin());
            if(BTraj.size() == 0) finish_plan = 1;

            // CAL_YAW
            double arg_    = atan2(-BT_ptr.vel_[0],BT_ptr.vel_[1]) + (PI/2.0f);
            double vel_len = sqrt(pow(BT_ptr.vel_[0],2)+pow(BT_ptr.vel_[1],2));
            if(vel_len<=0.1) arg_ = last_yaw; // 维持原yaw角
            std::pair<double, double> yaw_all = calculate_yaw(last_yaw,arg_);
            geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(yaw_all.first);

        /*   AIM   */
            geometry_msgs::PoseStamped pos_ptr;
            // POSE
            pos_ptr.pose.position.x = BT_ptr.pos_[0];
            pos_ptr.pose.position.y = BT_ptr.pos_[1];
            pos_ptr.pose.position.z = BT_ptr.pos_[2];
            aim.pose.position = pos_ptr.pose.position;
            // YAW
            // aim.pose.orientation = geo_q;// BUG
            

        /*   PVA   */
        /*
        Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
        Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
        Bit 10 should set to 0, means is not force sp
        */
            // MASK
            pva_msg.type_mask = 0b110000000000;  // 100 111 111 000  xyz + yaw
            // FRAME
            pva_msg.coordinate_frame = 1;
            // POSE
            pva_msg.position.x = BT_ptr.pos_[0];
            pva_msg.position.y = BT_ptr.pos_[1];
            pva_msg.position.z = BT_ptr.pos_[2];              // 
            // VEL
            pva_msg.velocity.x = BT_ptr.vel_[0];
            pva_msg.velocity.y = BT_ptr.vel_[1];
            pva_msg.velocity.z = BT_ptr.vel_[2];
            // ACC
            pva_msg.acceleration_or_force.x = BT_ptr.acc_[0];
            pva_msg.acceleration_or_force.y = BT_ptr.acc_[1];
            pva_msg.acceleration_or_force.z = BT_ptr.acc_[2];
            // YAW
            pva_msg.yaw      = yaw_all.first;
            pva_msg.yaw_rate = yaw_all.second;
            // TIME
            pva_msg.header.stamp = ros::Time::now();
            /*   PUB_CONTROL   */
            local_target_pub.publish(pva_msg);               // 无人机pva控制

        /*   BS   */ 
            int last_traj_seq      = (*(BTraj.end()-1)).seq_;
            int first_traj_seq     = (*(BTraj.begin())).seq_;
            int remain_traj_length = last_traj_seq- first_traj_seq;
            
            int to_bs_seq = (remain_traj_length>seq_interval) ? first_traj_seq + seq_interval : last_traj_seq;
            int to_bs_seq_index = to_bs_seq - first_traj_seq;
            if(BTraj.size()<=to_bs_seq_index) return;
            BT_ptr = BTraj[to_bs_seq_index];
            // POSE
            bs_msg.position.x = BT_ptr.pos_[0];
            bs_msg.position.y = BT_ptr.pos_[1];
            bs_msg.position.z = BT_ptr.pos_[2];// FIXME 修改成对应曲线高度
            // VEL
            bs_msg.velocity.x = BT_ptr.vel_[0];
            bs_msg.velocity.y = BT_ptr.vel_[1];
            bs_msg.velocity.z = BT_ptr.vel_[2];// FIXME 修改成对应曲线速度
            // ACC
            bs_msg.acceleration_or_force.x = BT_ptr.acc_[0];
            bs_msg.acceleration_or_force.y = BT_ptr.acc_[1];
            bs_msg.acceleration_or_force.z = BT_ptr.acc_[2];// FIXME 修改成对应曲线加速度
            bs_msg.yaw = (float)(to_bs_seq);
           
        }
    else
        {
            // cout <<"\033[32m[cmd] Arrived!\033[0m"<< endl;
            local_pos_pub.publish(aim);

            // double error_finish = sqrt(
            //                                                        pow(aim.pose.position.x - pos_drone_fcu[0],2)+
            //                                                        pow(aim.pose.position.y - pos_drone_fcu[1],2)+
            //                                                        pow(aim.pose.position.z - pos_drone_fcu[2],2));
            // if(error_finish < 0.1)
            //     finish_plan = 1;
            // POSE
            bs_msg.position = aim.pose.position;
            // VEL
            bs_msg.velocity.x = 0.0;
            bs_msg.velocity.y = 0.0;
            bs_msg.velocity.z = 0.0;// FIXME 修改成对应曲线速度
            // ACC
            bs_msg.acceleration_or_force.x = 0.0;
            bs_msg.acceleration_or_force.y = 0.0;
            bs_msg.acceleration_or_force.z = 0.0;// FIXME 修改成对应曲线加速度
            bs_msg.yaw = (float)(0.0);
        }
        // DEBUG
        debug_pub.publish(debug_msg); 
        /*   PUB_BS   */
        bs_pub.publish(bs_msg);

        // VIS
        geometry_msgs::PoseStamped vis_msg;
        vis_msg.header.frame_id = "world";
        vis_msg.pose.position = aim.pose.position;
        vis_path.poses.push_back(vis_msg);
        vis_path.header.frame_id = "world";
        vis_path_pub.publish(vis_path);
}