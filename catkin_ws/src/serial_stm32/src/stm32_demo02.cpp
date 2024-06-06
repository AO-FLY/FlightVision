#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
serial::Serial ros_send;

const unsigned head[2]={0xfd,0x88};
const unsigned end  [2]={0x0d,0x0a};
union sendData
{
        float   d;
        unsigned char data[4];
}t265_x,t265_y,t265_z;
union sendspeed
{
    float speed;
    unsigned char data[4];
}t265_speed_x,t265_speed_y,t265_speed_z;
 union sendyaw
 {
    float angle;
    unsigned char data[4];
 }t265_yaw;
float get_yaw(const nav_msgs::Odometry::ConstPtr& msg);
float get_yaw(const nav_msgs::Odometry::ConstPtr& msg) 
{
    float yaw;
    float siny_cosp = 2 * (msg->pose.pose.orientation.w*msg->pose.pose.orientation.z+msg->pose.pose.orientation.x*msg->pose.pose.orientation.y);
    float cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    // double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    // double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
    yaw=yaw*57.3249;
    return yaw;
}


void write_odom(float t265x,float t265y,float t265z,float speed_x,float speed_y,float speed_z);
void write_odom(float t265x,float t265y,float t265z,float speed_x,float speed_y,float speed_z)
{
    unsigned char buffer[26];
    int i,length=0;

    t265_x.d=t265x*10;
    t265_y.d=t265y*10;
    t265_z.d=t265z*10;
    t265_speed_x.speed=speed_x;
    t265_speed_y.speed=speed_y;
    // t265_speed_z.speed=speed_z;
    t265_yaw.angle=speed_z;
    ROS_INFO("yaw坐标:%f", t265_yaw.angle) ;
    for(i=0;i<2;i++)
    {
        buffer[i]=head[i];//设置消息头 buffer[0]buffer[1]
    }
    // length=12;
    //  buffer[2]=12;
    for(i=0;i<4;i++)
    {
        buffer[i+2]=t265_x.data[i]; //buffer[2]buffer[3]buffer[4]buffer[5]
        buffer[i+6]=t265_y.data[i];///buffer[6]buffer[7]buffer[8]buffer[9]
        buffer[i+10]=t265_z.data[i];///buffer[10]buffer[11]buffer[12]buffer[13]
        buffer[i+14]=t265_speed_x.data[i];
        buffer[i+18]=t265_speed_y.data[i];
        buffer[i+22]=t265_yaw.data[i];
   }
    // for(i=0;i<2;i++)
    // {
    //     buffer[i+15]=end[i];//设置消息头 buffer[14]buffer[15]
    // }
    ros_send.write(buffer,26);
}

void send_odom_yaw(float t265x,float t265y);
void send_odom_yaw(float t265x,float t265y)
{
    unsigned char buffer[5];
    int i; 

    buffer[0]=t265x*10;
    buffer[1]=t265y*10;
    //ROS_INFO("yaw坐标:%c", t265_yaw.angle) ;
    
    for(i=0;i<2;i++)
    {
        buffer[i+2]=head[i];//设置消息头 buffer[0]buffer[1]
    }
    buffer[4]='\n';
    
       ros_send.write(buffer,5);
}

void doMsg(const nav_msgs::Odometry::ConstPtr& msg)
{
  static char pose_x=0,pose_y=0,pose_z=0,speed_x=0,speed_y=0,speed_z=0,angle_yaw=0;
 
  pose_x = (char)msg->pose.pose.position.x;
  pose_y = (char)msg->pose.pose.position.y;
  ROS_INFO("x坐标:%c", pose_x) ;

   ROS_INFO("y坐标:%c", pose_y) ;

//用t265四元数转换得到偏航(PS:飞机的大功率器件产生的磁场十分影响飞控的地磁计，所以我们选择用t265的四元数转换来的偏航传给飞机使用)

send_odom_yaw(pose_x,pose_y);
 
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "air2_t265");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/odom/sample",10,doMsg);
    ros_send.setPort("/dev/ttyTHS2");
    ros_send.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ros_send.setTimeout(to);
    try
    {
            ros_send.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ros_send.isOpen())
    {
        ros_send.flushInput(); //清空缓冲区数据
        ROS_INFO_STREAM("Serial2 Port  has opened");
    }
    else
    {
        return -1;
    }
   ros::Rate loop_rate(50);
    ros::spin();
      return 0;
}
