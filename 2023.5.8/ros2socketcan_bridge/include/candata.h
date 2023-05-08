
#ifndef __CANDATA_H__
#define __CANDATA_H__


#include <linux/can/raw.h>

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float64.hpp"
#include "motor_canmsgs/msg/motor_msg.hpp"



/*
    规定：该CAN接口一次传输8个字节，
         任务id规定：
            规定id = 0x4X：数据传输 shoulder motor 信息
                    0x41：位置模式
                    0x42：位置+速度混合控制模式
                    0x43：力矩模式

            规定id = 0x5X：数据传输 driver motor 信息
                    0x51: 位置模式（用于锁定轮子）
                    0x52: 速度模式
                    0x53：力矩模式（可能用不到）

            规定id = 0x6X：数据传输 linear motor 信息
                    0x61：传输推杆电机角度信息
                    0x62: 传输推杆电机速度信息

         data规定：
            第1位为发送数据的
*/

class Candata : public rclcpp::Node
{
    public:

        Candata();
        ~Candata();
        void Can_send_main();
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr data_publisher;

        rclcpp::Subscription<motor_canmsgs::msg::MotorMsg>::SharedPtr Motor_Ctrl_subscriber;   
        

    private:
        
        can_msgs::msg::Frame canmsg_send; // 每一次发送的信息

        void Candata_send_test();  // only for test

        void SMotor_Pos(int Motor_ID,_Float64 pos);  //用于逆运动学的位置控制
        void SMotor_Pos_Vel(int Motor_ID,_Float64 pos, _Float64 vel); //用于逆运动学的位置控制
        void SMotor_Torque(int Motor_ID,_Float64 torque); //用于QP等优化方案求出的力矩控制
        
        void DMotor_Pos(int Motor_ID,_Float64 pos);
        void DMotor_Vel(int Motor_ID,_Float64 vel);
        void DMotor_Torque(int Motor_ID,_Float64 torque); // driving motor 不太会去使用力矩控制
         
        void LMotor_Angle(int Motor_ID,_Float64 angle);
        void LMotor_Vel(int Motor_ID,_Float64 vel); // 发送PWM占空比指令以控制推杆电机的速度

        void Motor_Ctrl_Sub_CB(const motor_canmsgs::msg::MotorMsg::SharedPtr data_ptr);
};



#endif