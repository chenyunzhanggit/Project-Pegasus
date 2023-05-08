#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "FSM_states.hpp"
#include "leg_states.hpp"
#include "motor_canmsgs/msg/motor_msg.hpp"
#include "motor_canmsgs/msg/a1_motor_msg.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#define RAD 57.29578


using namespace Eigen;
using namespace std::chrono_literals;

class Robot_Control:public rclcpp::Node
{
    public:
        Robot_Control();
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr State_Switch_Sub;  // 订阅由 state_switch 节点发出的state切换的信息

        rclcpp::Publisher<motor_canmsgs::msg::MotorMsg>::SharedPtr Motor_Candata_Pub; // Candata Send 推杆电机和轮子电机的信息
        rclcpp::Publisher<motor_canmsgs::msg::A1MotorMsg>::SharedPtr A1Motor_Data_Pub;// 发布宇树A1电机的参数

        rclcpp::Subscription<double>::SharedPtr Motor_Angle_Sub; 

        //创建两个定时器
	      //rclcpp::TimerBase::SharedPtr timer1,timer2;
        rclcpp::WallRate loop_rate; //构造时的单位为 Hz

        
        void control();
        
        ~Robot_Control();
    private:
        rclcpp::Node::SharedPtr node_;
        void State_Switch_Sub_CB(const std_msgs::msg::Int16::SharedPtr state_switch_data_ptr);
        FSM_STATES FSM_state;
        
        motor_canmsgs::msg::MotorMsg shoulder_linear_motor_msg;
        motor_canmsgs::msg::A1MotorMsg A1motor_msg;

        int8_t fixed_stand_flag;
        void FixedStand();
        
        void Motor_CANdata_Send(uint32_t CAN_id, uint8_t Motor_id, _Float64 data);
        void A1Motor_Data_Send(uint8_t leg_id, uint8_t motor_id, float t, float w,float pos,
                                float k_p,float k_w);
        // void trajectory_and_inv_kinematic(double t); // 摆动项中用到的轨迹和逆运动学解

};







/*
void trajectory_and_inv_kinematic(double t)
  {
    
    double Tm = 2.0;  //T of the wobble phase
    double S = 0.1;
    double H = 0.1;

    //cout << "t = " << t << endl;

      if (t < Tm){   // 4: T of the wobble phase
        double Fe_t = t/Tm - 1/(4*Pi) * sin(4*Pi*t/Tm);

        traj.x = S * (t/Tm - 1/(2*Pi) * sin((2*Pi*t)/Tm));
        traj.y = 0.0;
        traj.z = H * (sgn(Tm/2-t) * (2*Fe_t-1)+1);

        traj.x_vel = S * (1/Tm - 1/Tm * cos(2*Pi*t/Tm));
        traj.y_vel = 0.0;
        traj.z_vel = H * (sgn(Tm/2-t) * 2 *(1/Tm - 1/Tm * cos(4*Pi*t/Tm)));
        
      }
    else if (t >= Tm &&  t <= 2*Tm){
         
        traj.x = S*((2*Tm-t)/Tm + 1/(2*Pi)*sin(2*Pi*t/Tm));

        traj.y = 0.0;
        traj.z = 0.0;

        traj.x_vel = S * (-1/Tm + 1/Tm * cos(2*Pi*t/Tm));
        traj.y_vel = 0.0;
        traj.z_vel = 0.0;
        
     }
    // he unit of the above velocity is m/s


    
    cout << "traj_x = " << traj.x <<endl;

    cout << "traj_y = " << traj.y <<endl;
    cout << "traj_z = " << traj.z <<endl;
   

   
    cout << "traj_x_vel = " << traj.x_vel <<endl;

    cout << "traj_y_vel = " << traj.y_vel <<endl;
    cout << "traj_z_vel = " << traj.z_vel <<endl;
    
    
    // inverse kinematics
    double alpha,beta,gamma;
    double lxz_,n;

    
    double init_theta1 = 0.0;
    double init_theta2 = 45/RAD;
    double init_theta3 = -105/RAD;

    // initialized coordinate
    double x_init = l1 * sin(init_theta2) + l2*sin(init_theta2+init_theta3);
    double y_init = l0;
    double z_init = -cos(init_theta1) * (l1*cos(init_theta2)+ l2*cos(init_theta2+init_theta3));    // L0 =0

    double des_x,des_y,des_z;

    
    

    des_x = x_init + traj.x;
    des_y = y_init + traj.y;  // y is a minus value, reasoning by the acutal calculation. 
    des_z = z_init + traj.z;

    //traj.y = h;
    
    lxz_ = sqrt(des_y*des_y + des_z*des_z - l0*l0 + des_x*des_x);
    n = (lxz_*lxz_ - l1*l1 - l2*l2)/(2*l1);

    gamma = -(-atan(des_y/des_z)) + atan(l0/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));        //hip.  In a normal model it always suppose that y is a negative value, but in my model it's a positive one. So I have to add a minus sign.
    
    alpha = acos((l1+n)/lxz_) - atan(des_x/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));         //hu

    beta = -acos(n/l2);                                                                        //hl

    
    //cout << "alpha = " << alpha << endl;
    //cout << "beta = " << beta << endl;
    //cout << "gamma = " << gamma << endl;
    

    float t0,t1,t2;   //rad of 3 joint 

    //cout << "get the imformation "  <<endl;

    t0 = gamma;
    t1 = alpha;
    t2 = beta; 
    

    Jacobian(0) = 0.0;
    Jacobian(1) = l1*cos(t1) + l2 *cos(t1+t2);
    Jacobian(2) = l2*cos(t1+t2);

    Jacobian(3) = cos(t0)*(l1*cos(t1)+l2*cos(t1+t2));
    Jacobian(4) = -sin(t0)*(l1*sin(t1)+l2*sin(t1+t2));
    Jacobian(5) = -sin(t0)*(l2*sin(t1+t2));

    Jacobian(6) = sin(t0)*(l1*cos(t1)+l2*cos(t1+t2));
    Jacobian(7) = cos(t0)*(l1*sin(t1)+l2*sin(t1+t2));
    Jacobian(8) = cos(t0)*l2*sin(t1+t2);

    Jacobian_Inv = Jacobian.inverse();

    //cout << "Jacobian = \n" << Jacobian << endl;
    //cout << "Jacobian_Inv = \n" << Jacobian_Inv << endl;

    des_joint_angle(0) = (float)gamma;
    des_joint_angle(1) = (float)alpha;
    des_joint_angle(2) = (float)beta;

    
    end_vel(1) = traj.x_vel;
    end_vel(0) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    

    
    end_vel(0) = traj.x_vel;
    end_vel(1) = traj.y_vel;
    end_vel(2) = traj.z_vel;
        
    desire_ang_vel = Jacobian_Inv * end_vel;

    des_joint_angle.leg1_x_vel = (float)traj.x_vel;
    des_joint_angle.leg1_y_vel = (float)traj.y_vel;
    des_joint_angle.leg1_z_vel = (float)traj.z_vel;

    //cout  << "desire_angular_velocity: \n" << desire_ang_vel << endl；

  }

*/