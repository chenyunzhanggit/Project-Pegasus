#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "motor_canmsgs/msg/motor_msg.hpp"


// 肩关节电机主要做 位置控制 + 力矩控制
class Joint_Motor: public rclcpp::Node  
{
    public:
        Joint_Motor();
        
        rclcpp::Publisher<motor_canmsgs::msg::MotorMsg>::SharedPtr JMotor_Info_Pub;
        void JMotor_Swing_Pos_Ctrl();
        
        ~Joint_Motor();
    private:
        motor_canmsgs::msg::MotorMsg SMotor1_data_send,SMotor2_data_send,SMotor3_data_send,SMotor4_data_send;
        

};

// 驱动轮关节主要做 滚动速度控制 + 行走位置lock控制
class Driving_Motor: public rclcpp::Node  
{
    public:
        Driving_Motor();
        rclcpp::Publisher<motor_canmsgs::msg::MotorMsg>::SharedPtr DMotor_Info_Pub;
        void DMotor_Speed_Ctrl(); // control the speed of the Driving Motor  // 0x52
        void DMotor_Pos_Ctrl(); // lock the wheel to let robot walk     //0x51

        ~Driving_Motor();
    private:
        motor_canmsgs::msg::MotorMsg DMotor1_data_send,DMotor2_data_send,DMotor3_data_send,DMotor4_data_send;

};

// 推杆电机主要做 角度控制 + 速度控制，不过速度控制可以根据下位机去shishi计算控制pwm的占空比。
class Linear_Motor: public rclcpp::Node  
{
    public:
        Linear_Motor();

        ~Linear_Motor();
    private:
    

};

class TEST: public rclcpp::Node
{
    public:
        TEST();

        ~TEST();
    private:

};