#include "candata.h"

/*
    candata.h && candata.cpp 用于与下位机通信的数据处理与数据交换
    除了电机指令以外还会有机器人状态信息的交换
*/

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
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

Candata::Candata(): Node("Candata_Transfer")
{
    //创建发布节点
    data_publisher 		       = this->create_publisher<can_msgs::msg::Frame>("can_send", 100); // 数据发送给下位机的通信节点
    Motor_Ctrl_subscriber      = this->create_subscription<motor_canmsgs::msg::MotorMsg>("Shoulder_Linear_Motor_Ctrl",100,std::bind(&Candata::Motor_Ctrl_Sub_CB, this, _1));  
}

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    
    Candata candata;
    candata.Can_send_main();
    
    return 0;
}



void Candata::Can_send_main()
{
    rclcpp::Node::SharedPtr node_(this); // 创建基类指针，指向子类对象this


    while(rclcpp::ok())
    {
        //Candata_send_test();

        rclcpp::spin_some(node_); // 运行正常   
    }
    

}

void Candata::Candata_send_test() //only for test
{
    /*
    std::array<uint8_t, 8> data_send;

    data_send = {1,2,3,4,5,6,7,8};

    msg_send.id = 0x01;
    msg_send.dlc = 8;
    msg_send.is_extended = false;
    msg_send.is_rtr = false;
    msg_send.is_error = false;
    msg_send.data = data;
    data_publisher->publish(msg_send);
    */

    std::array<uint8_t, 8> data_send; // 8字节的需要发送的数据
    int32_t pos_data;// 经过乘1000处理后的32位整形数据

    pos_data = 32000;
    
    data_send[0] = 1; // 第一位数据定义为电机的ID号
    data_send[1] = 0;
    data_send[2] = 0;
    data_send[3] = 0;
    data_send[4] = *((uint8_t *)(&pos_data) + 0);
	data_send[5] = *((uint8_t *)(&pos_data) + 1);  
	data_send[6] = *((uint8_t *)(&pos_data) + 2);
	data_send[7] = *((uint8_t *)(&pos_data) + 3);


    canmsg_send.id = 0x01;
    canmsg_send.dlc = 8;
    canmsg_send.is_extended = false;
    canmsg_send.is_rtr = false;
    canmsg_send.is_error = false;
    canmsg_send.data = data_send;
    data_publisher->publish(canmsg_send);
}






/********** CallBack **********/
void Candata::Motor_Ctrl_Sub_CB(const motor_canmsgs::msg::MotorMsg::SharedPtr data_ptr)
{
    
    double raw_data;  //接收到的通过计算出的角度数据
    std::array<uint8_t, 8> candata_send;    // 8字节的需要发送的数据
    int32_t data_send;   // 经过乘1000处理后的32位整形数据
    //cout << "get the data" <<endl;
    raw_data = data_ptr-> data;
    data_send = raw_data * 1000;
    cout << data_send << endl;
    candata_send[0] = data_ptr->motor_id; // 第一位数据定义为电机的ID号
    candata_send[1] = NULL;
    candata_send[2] = 0;
    candata_send[3] = 0;
    candata_send[4] = *((uint8_t *)(&data_send) + 0);
	candata_send[5] = *((uint8_t *)(&data_send) + 1);
	candata_send[6] = *((uint8_t *)(&data_send) + 2);
	candata_send[7] = *((uint8_t *)(&data_send) + 3);


    canmsg_send.id = data_ptr->can_id; // 自定义的数据ID
    canmsg_send.dlc = 8;
    canmsg_send.is_extended = false;
    canmsg_send.is_rtr = false;
    canmsg_send.is_error = false;
    canmsg_send.data = candata_send;
    data_publisher->publish(canmsg_send);
    
}



/********** 函数 **********/
Candata::~Candata()
{



}
