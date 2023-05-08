#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h>      
#include <rcl/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <robot_msgs/msg/joint_angle.hpp>


using namespace std;
using namespace Eigen;

#define SEND_DATA_CHECK   1          //发送数据校验标志位
#define READ_DATA_CHECK   0          //接收数据校验标志位
#define FRAME_HEADER      0X7B       //帧头
#define FRAME_TAIL        0X7D       //帧尾
#define RECEIVE_DATA_SIZE 52         //下位机发送过来的数据的长度  28 = 4+12*2     52 = 4+12*2 + 12*2
#define SEND_DATA_SIZE    27         //ROS向下位机发送的数据的长度 51 = 3+12*4
#define PI 				  3.1415926f //圆周率
#define W                 2          //机器人宽度
#define L                 3          //机器人长度

//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

typedef struct _SEND_DATA_  
{
    uint8_t tx[SEND_DATA_SIZE];
    unsigned char Frame_Header;

    int16_t des_joint_angle[12];

    unsigned char Frame_Tail; 
}SEND_DATA;

typedef struct _RECEIVE_DATA_     
{
	uint8_t rx[RECEIVE_DATA_SIZE];

    uint8_t Flag_Stop;
	unsigned char Frame_Header;

	unsigned char Frame_Tail;

}RECEIVE_DATA;

typedef struct 
{
    // motor1: shoulder; motor2: hip; motor3:knee
    Eigen::Matrix<float,12,1> Leg_Motor_Angle;
    Eigen::Matrix<float,12,1> Leg_Motor_RPM; // Radian per minute

}JOINT_MOTOR_INFO_TYPEDEF;


typedef struct 
{
    // motor1: shoulder; motor2: hip; motor3:knee
    float Leg1_Motor_shoulder;
    float Leg1_Motor_hip;
    float Leg1_Motor_knee;

    float Leg2_Motor_shoulder;
    float Leg2_Motor_hip;
    float Leg2_Motor_knee;

    float Leg3_Motor_shoulder;
    float Leg3_Motor_hip;
    float Leg3_Motor_knee;

    float Leg4_Motor_shoulder;
    float Leg4_Motor_hip;
    float Leg4_Motor_knee;
    
}JOINT_TORQUE_TYPEDEF;



class turn_on_robot : public rclcpp::Node
{
	public:

		turn_on_robot();
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
		//void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象 
        rclcpp::Subscription<robot_msgs::msg::JointAngle>::SharedPtr Joint_Angle_Sub;

        


	private:
		float Sampling_Time;        //采样时间，用于积分求位移(里程)   
        JOINT_MOTOR_INFO_TYPEDEF Joint_Motor_Info;
        JOINT_TORQUE_TYPEDEF Joint_Torque;
        
        // ROS communication Part

        bool Get_Motor_Info(void); // get 12 joint motor angle 
        float Data_Trans_16(uint8_t Data_High,uint8_t Data_Low);
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);           //BBC校验函数
        RECEIVE_DATA Receive_Data; //串口接收数据结构体
        SEND_DATA Send_Data;       //串口发送数据结构体
        
        // Robot Leg Params
        

        void Joint_Angle_Callback(const robot_msgs::msg::JointAngle::SharedPtr des_joint_angle_ptr);
        

};
