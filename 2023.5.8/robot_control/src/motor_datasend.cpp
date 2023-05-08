#include "motor_datasend.hpp"

/*
    规定：该CAN接口一次传输8个字节，
         任务id规定：
            规定id = 0x4X：数据传输 shoulder motor 信息
                    0x41：位置模式
                    0x42：位置+速度混合控制模式(大概率用不到)
                    0x43：力矩模式

            规定id = 0x5X：数据传输 driver motor 信息
                    0x51: 位置模式（用于锁定轮子） 
                    0x52: 速度模式rpm     // 锁定关节不动则将速度设置为0即可
                    0x53：力矩模式（可能用不到）

            规定id = 0x6X：数据传输 linear motor 信息
                    0x61：传输推杆电机角度信息
                    0x62: 传输推杆电机速度信息

         data规定：
            data第1位为电机的id
*/


using namespace std;

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    Joint_Motor JMotor;
    Driving_Motor DMotor;
    Linear_Motor LMotor;
    while (rclcpp::ok()){
        //JMotor.JMotor_Swing_Pos_Ctrl();
        //DMotor.DMotor_Pos_Ctrl();
    }
    
    return 0;
}




void Joint_Motor::JMotor_Swing_Pos_Ctrl()
{
    SMotor1_data_send.can_id = 0x41;
    SMotor1_data_send.data = 1.28754; // rad
    JMotor_Info_Pub->publish(SMotor1_data_send);

}

void Driving_Motor::DMotor_Pos_Ctrl()
{
    DMotor1_data_send.can_id = 0x51;
    DMotor1_data_send.data = 1.28754; // rad
    DMotor_Info_Pub->publish(DMotor1_data_send);

} // lock the wheel to let robot walk     //0x51

void Driving_Motor::DMotor_Speed_Ctrl()
{
    DMotor1_data_send.can_id = 0x52;

}


Joint_Motor::Joint_Motor() : Node("Joint_Motor")
{
    JMotor_Info_Pub    = this->create_publisher<motor_canmsgs::msg::MotorMsg>("Motor_Ctrl", 100);
}

Driving_Motor::Driving_Motor() : Node("Driving_Motor")
{
    DMotor_Info_Pub    = this->create_publisher<motor_canmsgs::msg::MotorMsg>("Motor_Ctrl", 100);
}

Linear_Motor::Linear_Motor() : Node("Linear_Motor")
{

}





Linear_Motor::~Linear_Motor() 
{}

Driving_Motor::~Driving_Motor()
{}

Joint_Motor::~Joint_Motor() 
{}


// For test
TEST::TEST(): Node("TEST")
{


}

TEST::~TEST()
{


}