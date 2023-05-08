#include <csignal>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serialPort/SerialPort.h"
#include <ctime>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <math.h>
#include "motor_canmsgs/msg/a1_motor_msg.hpp"



using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define Pi  3.14159
#define RAD 57.29578

/*
#define Thigh_Angle_Init -50
#define Thigh_Angle_Init 50
#define Knee_Angle_Init 140
#define Knee_Angle_Init -140
*/

typedef struct{

    double x;
    double y;
    double z;

    double x_vel;
    double y_vel;
    double z_vel;

}trajectoryTypeDef;

/* 
  for every leg, define :
    thigh motor: ID 1;
    knee  motor: ID 0;

*/

typedef struct //__attribute__((__packed__))
{
  double pos0_init, pos1_init; //encoder 
  double pos0_delta,pos1_delta; //encoder
  double pos0_cur,  pos1_cur; //encoder
  double pos0_last, pos1_last; //encoder

  double theta0_des,  theta1_des; // rad
  double theta0_init,  theta1_init; // rad
  double theta0_cur,  theta1_cur; // rad
  double theta0_last,  theta1_last; // rad
  
  /******  Motor0: Knee joint; Motor1: Knee Joint  ******/
  MOTOR_send motor0_run, motor0_stop, motor0_init, motor1_run, motor1_stop, motor1_init;
  // receive message struct
  MOTOR_recv motor0_rev,motor1_rev; // Motor 受到的数据信息
  

}LegUni_A1Typedef;



class UnitreeA1 : public rclcpp::Node
{
  public:
    
    UnitreeA1();
    ~UnitreeA1();
    LegUni_A1Typedef Leg1_UniA1, Leg2_UniA1, Leg3_UniA1, Leg4_UniA1;
    rclcpp::Subscription<motor_canmsgs::msg::A1MotorMsg>::SharedPtr A1Motor_Sub;//接受来自Robot_Control中 A1Motor 的控制消息
    rclcpp::TimerBase::SharedPtr linear_diff_timer;
    
    void motor_control();
    // Motor_Control loop   

  private:
    rclcpp::Node::SharedPtr node_;// 创建基类指针，指向子类对象this
    void A1Motor_Sub_CB(const motor_canmsgs::msg::A1MotorMsg::SharedPtr A1Motor_data_ptr);
    void motor_init();
    void motor_stop(); 


    void linear_diff_timer_callback();
    double theta_des_temp;
    float _percent0,_percent1;
    int _duration;
};
