#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace Eigen;

#define DAMPING         1 // 初始状态与结束状态
#define FIXEDSUPPORT    2 // 静止支撑相，VMC控制
#define SWING           3 // 摆动相
#define ROLLINGSUPPORT  4 // 滚动支撑相，会提供一定的力矩


typedef struct
{
    int16_t leg_states; //腿状态
    double motor_angle[3];  // [0]:shoulder angle, [1]:hip_angle, [2]:knee_angle (rad)
    double leg_length;
    double wheel_rpm;
    double wheel_deflection;
    double hu = 0.25;
    double hl = 0.20;
    Eigen::Matrix3d Jac_;
    

}LegStateTypeDef;




class LEG_STATES
{
    public:
        LEG_STATES();
              
        ~LEG_STATES();

        // 根据机器人的结构：
        double fixed_stand_rad[12] = {0,-0.67, 1.3, 0, 0.67, -1.3, 0, -0.67, 1.3, 0, 0.67, -1.3};
        LegStateTypeDef Leg1,Leg2,Leg3,Leg4;



        void test();
    private:
        

};


extern LEG_STATES legs_states;