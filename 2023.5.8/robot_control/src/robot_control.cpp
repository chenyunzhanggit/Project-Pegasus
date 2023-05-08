#include "robot_control.hpp"



using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/*
    该节点的工作：
        1. 订阅腿部信息，并存入 Class LegState中
        2. 
*/

Robot_Control::Robot_Control() : Node("Robot_Control"), loop_rate(1.0), node_(this)
{

    fixed_stand_flag = 0;

    State_Switch_Sub = this->create_subscription<std_msgs::msg::Int16>("state_switch", 5,std::bind(&Robot_Control::State_Switch_Sub_CB, this, _1));
    Motor_Candata_Pub = this->create_publisher<motor_canmsgs::msg::MotorMsg>("Shoulder_Linear_Motor_Ctrl", 10);
    A1Motor_Data_Pub = this->create_publisher<motor_canmsgs::msg::A1MotorMsg>("A1Motor_Ctrl", 100);
}

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    
    Robot_Control controller;
    controller.control();
    return 0;
}

void Robot_Control::control()
{
    // 创建基类指针，指向子类对象this

    while(rclcpp::ok())
    {
        
        switch(FSM_state.state.data)
        {
            case PASSIVE:
                {
                    // do damping things 
                    
                    //cout << "PASSIVE" <<endl;
                    
                    break;
                }

            case FIXED_STAND:
                {
                    // do 角度控制 Support
                    
                        //1. lock the shoulder motors and the wheels
                        //2. send the target position of the motors on 4 legs
                    
                    //定时器timer2初始化,100ms中断，回调函数为timer2_callback
                    
                    //cout << "FIXED_STAND" <<endl;
                    
                    if (fixed_stand_flag == 0)
                    {
                        FixedStand();
                        cout << "in FIXED STAND" << endl;
                    }
                    //cout << "XXX" << endl;
                    //loop_rate.sleep();
                    break;
                }
            
            case WALK:
                {
                    break;
                }

        }
        

        /*
        if (FSM_state.state.data == PASSIVE)
        {
            timer1 = node_->create_wall_timer(250ms,std::bind(&Robot_Control::timer1_cb, this));
            //cout << "PASSIVE" <<endl;
        }
        else if (FSM_state.state.data == FIXED_STAND)
        {
            timer2 = node_->create_wall_timer(250ms,std::bind(&Robot_Control::timer2_cb, this));
            //cout << "FIXED" <<endl;
        }
        */
        rclcpp::spin_some(node_); // 运行正常   
    }
}




void Robot_Control::FixedStand()
{
    //rclcpp::WallRate loop_rate(30);
    // 1. 发送锁定 shoulder motors and wheels 的指令 （by can_bus）该指令只需要速度为0即可
    /*
    Motor_CANdata_Send(0x42, 1, 0.0);
    Motor_CANdata_Send(0x42, 2, 0.0);
    Motor_CANdata_Send(0x42, 3, 0.0);
    Motor_CANdata_Send(0x42, 4, 0.0);
    Motor_CANdata_Send(0x52, 5, 0.0);
    Motor_CANdata_Send(0x52, 6, 0.0);
    Motor_CANdata_Send(0x52, 7, 0.0);
    Motor_CANdata_Send(0x52, 8, 0.0);
    */

    // 2. 发送A1电机关节角度控制指令
    //for (int i=1;i<5;i++){


    A1Motor_Data_Send(1, 0, 0.0, 0.0, (115/RAD), 0.5, 3.0); // 初始值：140
    A1Motor_Data_Send(1, 1, 0.0, 0.0, (-36/RAD), 0.5, 3.0);   // 初始值  -50
    //}
    //loop_rate.sleep();
    fixed_stand_flag = 1;
    
}





void Robot_Control::State_Switch_Sub_CB(const std_msgs::msg::Int16::SharedPtr state_switch_data_ptr)
{

    FSM_state.state.data = state_switch_data_ptr->data;
    cout << "state = " << FSM_state.state.data << endl;
}


void Robot_Control::Motor_CANdata_Send(uint32_t CAN_id, uint8_t Motor_id, _Float64 data)
{
    shoulder_linear_motor_msg.can_id = CAN_id;
    shoulder_linear_motor_msg.motor_id = Motor_id;
    shoulder_linear_motor_msg.data = data;
    Motor_Candata_Pub->publish(shoulder_linear_motor_msg);
}

void Robot_Control::A1Motor_Data_Send(uint8_t leg_id, uint8_t motor_id, float t, float w,float pos,
                                float k_p,float k_w)
{
    A1motor_msg.leg_id = leg_id;
    A1motor_msg.motor_id = motor_id;
    A1motor_msg.t = t;
    A1motor_msg.w = w;
    A1motor_msg.pos = pos;
    A1motor_msg.k_p = k_p;
    A1motor_msg.k_w = k_w;
    
    A1Motor_Data_Pub->publish(A1motor_msg);
}


Robot_Control::~Robot_Control()
{


    
}
