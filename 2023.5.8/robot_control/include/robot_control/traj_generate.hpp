#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "std_msgs/msg/float32.hpp"
#include <math.h>
#include "motor_canmsgs/msg/motor_msg.hpp"
#include "motor_canmsgs/msg/a1_motor_msg.hpp"
#include "leg_states.hpp"


#define Pi 3.1415926
#define RAD 57.29578


using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

typedef Eigen::Vector3f coordinate;

typedef struct{

    double x;
    double y;
    double z;

    double x_vel;
    double y_vel;
    double z_vel;

}trajectoryTypeDef;


class Traj_Generator : public rclcpp::Node
{
    public:
        Traj_Generator();
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr Traj_pub_timer;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Traj_Pub; // 发布轨迹点 

        rclcpp::Publisher<motor_canmsgs::msg::MotorMsg>::SharedPtr Motor_Candata_Pub; // Candata Send 推杆电机和轮子电机的信息
        rclcpp::Publisher<motor_canmsgs::msg::A1MotorMsg>::SharedPtr A1Motor_Data_Pub;// 发布宇树A1电机的参数
        

        trajectoryTypeDef traj;
        

        void Traj_Loop();
        ~Traj_Generator();

    private:
        motor_canmsgs::msg::MotorMsg shoulder_linear_motor_msg;
        motor_canmsgs::msg::A1MotorMsg A1motor_msg;


        int8_t flag;
        double Td; //摆动持续时间
        double t;  //进入函数的实际运动时间
        
        coordinate traj_p_0,traj_p_f,traj_p_out; // traj_p_0[3]：起始点 traj_p_f[3]：终点；//traj_p_out：输出轨迹点 
        coordinate pass_point;

        void pos_Generator(double height, double t); // Td为摆动持续时间; t: 实际时间

        coordinate cubicBezier(coordinate p0, coordinate pf,double t,double Tm); // Tm 单段持续时间

        double pos_output[3]; //[0]:shoulder,[1]:hip,[2]:knee
        void inverse_kinematic();



        int sgn(float num);
        // function for test
        void test_traj(double t);
        void test(double t);
        

        void Traj_pub_callback();
        void Motor_CANdata_Send(uint32_t CAN_id, uint8_t Motor_id, _Float64 data);
        void A1Motor_Data_Send(uint8_t leg_id, uint8_t motor_id, float t, float w,float pos,
                                float k_p,float k_w);
        

        Eigen::Matrix<float,3,3> Jacobian,Jacobian_Inv;
        Eigen::Matrix<float,3,1> end_vel,desire_ang_vel;
        Eigen::Matrix<float,3,1> des_joint_angle;
        rclcpp::Time start, end;

};