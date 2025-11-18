#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <serial/serial.h>
#include <string.h>
#include <std_msgs/UInt8.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h" 
uint8_t buffer[16];
serial::Serial ser;
#define K 700
int global_flag=1;//云台标志位 0：自瞄  1：导航  2：巡检
// int get_target = 0;
int global_wxm = 0;
void send_date(int x,int y,int z,int yaw,int pitch,char shoot){
    buffer[0]=0xff;
    buffer[1]=0x01;
    // buffer[2]=0x02;
    if(global_wxm%2==0){
      buffer[2]=0x03;
    }else if(global_wxm%2==1){
      buffer[2]=0x02;
    }
    buffer[3]=0x00;
    buffer[4]=shoot;//发射模式

    buffer[5]=((x>>8)&0xff);
    buffer[6]=(x&0xff);

    buffer[7]=((y>>8)&0xff);
    buffer[8]=(y&0xff);

    buffer[9]=((z>>8)&0xff);
    buffer[10]=(z&0xff);
    buffer[11]=((yaw>>8)&0xff);
    buffer[12]=(yaw&0xff);
    buffer[13]=((pitch>>8)&0xff);
    buffer[14]=(pitch&0xff);
    buffer[15]=0xfe;

    try
  {    
    if (ser.isOpen())
    {
      ROS_INFO("Serial Port opened.");
      ser.write(buffer,(size_t)16);
    }
    else
    {
      ROS_ERROR("Failed to open Serial Port.");
    }
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Serial Port Error: " << e.what());
  }
}

void cameraCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    char shoot = 0x00;
    if (msg->data.size() >= 4) { // 确保至少有两个元素
      if(global_flag == 0){
        float pitch = msg->data[0] ;
        float yaw =   msg->data[1] ;
        float shootf = msg->data[2];
        float get_targetf = msg->data[3];
        shoot = 0x00; 
        // if(get_targetf == 0 ){
        //   shoot = 0x00;     //有目标还没追踪到
        // }else if(get_targetf == 1){
        //   shoot = 0x01 ;     //有目标并且追踪到
        // }else if(get_targetf == 2){
        //   shoot = 0x02;      //没有目标
        // }
        ROS_INFO("Received pitch=%f, yaw=%f", pitch, yaw);
        if(get_targetf == 0)//没有目标
        {
          send_date(0, 0, 2, (int)yaw, (int)pitch, 0x00); // 巡检模式
        }else if (get_targetf == 1)//有的兄弟，有的 
        {
          send_date(0, 0, 0, (int)yaw, (int)pitch, 0x11); // 瞄准模式
        }
        // send_date(0, 0, global_flag, (int)yaw, (int)pitch, 0x00); // 使用接收到的pitch和yaw值发送数据
      }
    }   //1:导航模式  0：自瞄模式   
}
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  int linear_x = msg->linear.x * K;
  int linear_y = msg->linear.y * K;
  int angular_z = -msg->angular.z * 15000;
  ROS_INFO("Received flag: %d", global_flag);
  ROS_INFO("x = %d, y = %d, z = %d", linear_x,linear_y,angular_z);

  send_date(linear_x,linear_y,global_flag,angular_z,0,0x00);
}
void flagCallback(const std_msgs::UInt8::ConstPtr& msg) {
  global_flag = msg->data; 
}
void wxmCallback(const std_msgs::UInt8::ConstPtr& msg) {
  global_wxm = msg->data; 
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "test");

    ros::NodeHandle n;
    
    // 订阅/cmd_vel话题
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmdVelCallback);

    // 新增订阅camera_publisher话题
    ros::Subscriber cam_sub = n.subscribe("/camera_coords", 1000, cameraCallback);
    
    // 新增订阅serial_flag话题
    ros::Subscriber flag_sub = n.subscribe("/serial_flag", 1000, flagCallback);
    ros::Subscriber wxm_sub = n.subscribe("/serial_wxm", 1000, wxmCallback);
    ser.setPort("/dev/ser_tx");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
    ros::Rate rate(100);
    
    // ros::spin();
    while(ros::ok()){    
        send_date(0,0,0,0,0,0x11); //yaw往左为正数，往右为负数
        rate.sleep();
    }
    send_date(0,0,0,0,0,0x00);
    uint8_t a[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    ser.write(a,(size_t)16);
    return 0;
}
