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

using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;


#define Pi  3.14159
#define RAD 57.29578

/*
// 初始角度测试
#define ANGLE1 50
#define ANGLE2 -140
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

SerialPort serial1("/dev/ttyUSB0"); // for leg1 LF
//SerialPort serial2("/dev/ttyUSB1"); // for leg2 RF
//SerialPort serial3("/dev/ttyUSB2"); // for leg3 RH
//SerialPort serial4("/dev/ttyUSB3"); // for leg4 LH


typedef struct //__attribute__((__packed__))
{
  double pos0_init, pos1_init;
  double pos0_des, pos1_des;

  /******  Motor0: Knee joint; Motor1: Knee Joint  ******/
  MOTOR_send motor0_run, motor0_stop, motor0_init, motor1_run, motor1_stop, motor1_init;
  // receive message struct
  MOTOR_recv motor0_rev,motor1_rev; // Motor 受到的数据信息
  

}LegUni_A1Typedef;


class UnitreeA1 : public rclcpp::Node
{
  public:
    
    UnitreeA1() : Node("UnitreeA1_test")
    {
        motor_init();
    }

    ~UnitreeA1(){
        motor_stop();
    }
    LegUni_A1Typedef Leg1_UniA1, Leg2_UniA1, Leg3_UniA1, Leg4_UniA1;
    Eigen::Matrix<float,3,3> Jacobian,Jacobian_Inv;
    Eigen::Matrix<float,3,1> end_vel,desire_ang_vel;
    Eigen::Matrix<float,3,1> des_joint_angle;
    clock_t start,end;
    rclcpp::Time start1, end1;
  //long t_interval1;

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


    /*
    cout << "traj_x = " << traj.x <<endl;

    cout << "traj_y = " << traj.y <<endl;
    cout << "traj_z = " << traj.z <<endl;
    */

    /*
    cout << "traj_x_vel = " << traj.x_vel <<endl;

    cout << "traj_y_vel = " << traj.y_vel <<endl;
    cout << "traj_z_vel = " << traj.z_vel <<endl;
    */
    
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
    

    /*
    end_vel(0) = traj.x_vel;
    end_vel(1) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    */

    desire_ang_vel = Jacobian_Inv * end_vel;

    /*
    des_joint_angle.leg1_x_vel = (float)traj.x_vel;
    des_joint_angle.leg1_y_vel = (float)traj.y_vel;
    des_joint_angle.leg1_z_vel = (float)traj.z_vel;
    */


    //cout  << "desire_angular_velocity: \n" << desire_ang_vel << endl;

  

  }

  
  void get_start_time()
   {
     start1 = rclcpp::Node::now();
   } 

   void get_end_time()
   {
     end1 = rclcpp::Node::now();
     //cout << "end1 = " << end1.seconds() <<endl;
   } 
   
   void get_t_interval()
   {

      //cout << "t_interval1 = " << (end1 - start1).seconds() << endl;
   }
  

  private:
    void motor_init();
    void motor_stop(); 
    double l0 = 0.106; // distance from hip joint to hu joint 
    double l1 = 0.250;
    double l2 = 0.200;
    double t; // 函数参考时间
    trajectoryTypeDef traj;




    int sgn(float num)
  {
     if (num >= 0){
         return 1;
     }
     else{
         return -1;
     }
  }
    
   


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  UnitreeA1 unitree_motor;
  double t;
  t = 0.0;


  while (rclcpp::ok())
  {
    unitree_motor.start = clock();

    unitree_motor.get_start_time();
    unitree_motor.trajectory_and_inv_kinematic(t);

    /* **** hu motor ****   */

    //std::cout << "INIT_Pos:    " << unitree_motor.pos_init << std::endl;
    //cout << "pos_des = " <<unitree_motor.pos_des<<endl;
    //unitree_motor.pos_des +=  0.01;
    unitree_motor.Leg1_UniA1.pos1_des = (unitree_motor.des_joint_angle(1)-(45)/RAD) * 9.1;
        //std::cout << "DES_Pos:    " << pos_des << std::endl;
      //}

    //unitree_motor.motor_run.T = 0.01;
    unitree_motor.Leg1_UniA1.motor1_run.Pos = unitree_motor.Leg1_UniA1.pos1_init + unitree_motor.Leg1_UniA1.pos1_des;
    //unitree_motor.motor_run.W = unitree_motor.desire_ang_vel(1) * 9.1;

    //cout << "pos_des+init = " <<unitree_motor.Leg1_UniA1.motor1_run.Pos<<endl;
      // encode data into motor commands
      modify_data(&unitree_motor.Leg1_UniA1.motor1_run);
     
      //unitree_motor.motor_send_temp = unitree_motor.Leg1_UniA1.motor1_run;

      serial1.sendRecv(&unitree_motor.Leg1_UniA1.motor1_run, &unitree_motor.Leg1_UniA1.motor1_rev);
      //decode data from motor states
      extract_data(&unitree_motor.Leg1_UniA1.motor1_rev);

      //std::cout << "Pos:    " << unitree_motor.Leg1_UniA1.motor1_rev.Pos << std::endl;

    /* **** hl motor ****   */

    //std::cout << "INIT_Pos:    " << unitree_motor.pos_init << std::endl;
    //cout << "pos_des = " <<unitree_motor.pos_des<<endl;
    //unitree_motor.pos_des +=  0.01;
    unitree_motor.Leg1_UniA1.pos0_des = (unitree_motor.des_joint_angle(2)-(-105)/RAD) * 9.1;
        //std::cout << "DES_Pos:    " << pos_des << std::endl;
      //}

    //unitree_motor.motor_run.T = 0.01;
    // *** *** 控制的角度 = 初始的角度（由init函数中编码器读取的） + 期望转的角度
    unitree_motor.Leg1_UniA1.motor0_run.Pos = unitree_motor.Leg1_UniA1.pos0_init + unitree_motor.Leg1_UniA1.pos0_des;
    //unitree_motor.motor_run1.W = unitree_motor.desire_ang_vel(2) * 9.1;

    //cout << "pos_des+init1 = " << unitree_motor.Leg1_UniA1.motor0_run.Pos << endl;
      // encode data into motor commands
      modify_data(&unitree_motor.Leg1_UniA1.motor0_run);
     
      
      //unitree_motor.motor_send_temp = unitree_motor.Leg1_UniA1.motor0_run;
      serial1.sendRecv(&unitree_motor.Leg1_UniA1.motor0_run, &unitree_motor.Leg1_UniA1.motor0_rev);
      //decode data from motor states
      extract_data(&unitree_motor.Leg1_UniA1.motor0_rev);
      // 当前旋转的角度（单位：rad） = （当前读数 pos - 初始pos）/9.1（减速比）   

      //std::cout << "Pos1:    " << unitree_motor.Leg1_UniA1.motor0_rev.Pos << std::endl;
      //std::cout << "转过的角度 =    " << (unitree_motor.Leg1_UniA1.motor0_rev.Pos - unitree_motor.Leg1_UniA1.pos0_init)/9.1 << std::endl;

      unitree_motor.end = clock();
      unitree_motor.get_end_time();
      unitree_motor.get_t_interval();

      //t_interval = (double)(unitree_motor.end - unitree_motor.start)/CLOCKS_PER_SEC;
      

      t += (unitree_motor.end1-unitree_motor.start1).seconds();

      if (t >= 4.0)
        t =0.0;
      //cout << "t = " << t << endl;
      
  }


  
  //rclcpp::shutdown();
  return 0;
}


void UnitreeA1::motor_init()
{
    // Motor0是 Knee Joint; Motor1 是 Thigh Joint
    // Leg1 初始化
    Leg1_UniA1.motor0_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg1_UniA1.motor0_init.id = 0;
    Leg1_UniA1.motor0_init.motorType = MotorType::A1;
    Leg1_UniA1.motor0_init.mode = 10;
    Leg1_UniA1.motor0_init.T = 0.0;
    Leg1_UniA1.motor0_init.W = 0;
    Leg1_UniA1.motor0_init.Pos = 0;    
    Leg1_UniA1.motor0_init.K_P = 0.0;
    Leg1_UniA1.motor0_init.K_W = 0.0;   
    modify_data(&Leg1_UniA1.motor0_init);
    serial1.sendRecv(&Leg1_UniA1.motor0_init, &Leg1_UniA1.motor0_rev);
    Leg1_UniA1.pos0_init = Leg1_UniA1.motor0_rev.Pos;
    //std::cout << "Leg1,Motor0, INIT_Pos:    " << Leg1_UniA1.pos0_init << std::endl;

    Leg1_UniA1.motor1_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg1_UniA1.motor1_init.id = 1;
    Leg1_UniA1.motor1_init.motorType = MotorType::A1;
    Leg1_UniA1.motor1_init.mode = 10;
    Leg1_UniA1.motor1_init.T = 0.0;
    Leg1_UniA1.motor1_init.W = 0;
    Leg1_UniA1.motor1_init.Pos = 0;    
    Leg1_UniA1.motor1_init.K_P = 0.0;
    Leg1_UniA1.motor1_init.K_W = 0.0;   
    modify_data(&Leg1_UniA1.motor1_init);
    serial1.sendRecv(&Leg1_UniA1.motor1_init, &Leg1_UniA1.motor1_rev);
    Leg1_UniA1.pos1_init = Leg1_UniA1.motor1_rev.Pos;
    //std::cout << "Leg1,Motor1, INIT_Pos:    " << Leg1_UniA1.pos1_init << std::endl;

    //位置控制
    Leg1_UniA1.motor0_run.id = 0;
    Leg1_UniA1.motor0_run.motorType = MotorType::A1;
    Leg1_UniA1.motor0_run.mode = 10;
    Leg1_UniA1.motor0_run.T = 0.0; // 点击输出扭矩
    Leg1_UniA1.motor0_run.W = 0.0; // 电机转速
    Leg1_UniA1.motor0_run.Pos = 0;
    Leg1_UniA1.motor0_run.K_P = 0.2;
    Leg1_UniA1.motor0_run.K_W = 3.0;

    Leg1_UniA1.motor1_run.id = 1;
    Leg1_UniA1.motor1_run.motorType = MotorType::A1;
    Leg1_UniA1.motor1_run.mode = 10;
    Leg1_UniA1.motor1_run.T = 0.0; // 点击输出扭矩
    Leg1_UniA1.motor1_run.W = 0.0; // 电机转速
    Leg1_UniA1.motor1_run.Pos = 0;    
    Leg1_UniA1.motor1_run.K_P = 0.2;
    Leg1_UniA1.motor1_run.K_W = 3.0;

    /*
    // Leg2
    // Motor0是 Knee Joint; Motor1 是 Thigh Joint
    // Leg1 初始化
    Leg2_UniA1.motor0_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg2_UniA1.motor0_init.id = 0;
    Leg2_UniA1.motor0_init.motorType = MotorType::A1;
    Leg2_UniA1.motor0_init.mode = 10;
    Leg2_UniA1.motor0_init.T = 0.0;
    Leg2_UniA1.motor0_init.W = 0;
    Leg2_UniA1.motor0_init.Pos = 0;    
    Leg2_UniA1.motor0_init.K_P = 0.0;
    Leg2_UniA1.motor0_init.K_W = 0.0;   
    modify_data(&Leg2_UniA1.motor0_init);
    serial2.sendRecv(&Leg2_UniA1.motor0_init, &Leg2_UniA1.motor0_rev);
    Leg2_UniA1.pos0_init = Leg2_UniA1.motor0_rev.Pos;
    //std::cout << "Leg1,Motor0, INIT_Pos:    " << Leg1_UniA1.pos0_init << std::endl;

    Leg2_UniA1.motor1_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg2_UniA1.motor1_init.id = 1;
    Leg2_UniA1.motor1_init.motorType = MotorType::A1;
    Leg2_UniA1.motor1_init.mode = 10;
    Leg2_UniA1.motor1_init.T = 0.0;
    Leg2_UniA1.motor1_init.W = 0;
    Leg2_UniA1.motor1_init.Pos = 0;    
    Leg2_UniA1.motor1_init.K_P = 0.0;
    Leg2_UniA1.motor1_init.K_W = 0.0;   
    modify_data(&Leg2_UniA1.motor1_init);
    serial2.sendRecv(&Leg2_UniA1.motor1_init, &Leg2_UniA1.motor1_rev);
    Leg2_UniA1.pos1_init = Leg2_UniA1.motor1_rev.Pos;
    //std::cout << "Leg1,Motor1, INIT_Pos:    " << Leg1_UniA1.pos1_init << std::endl;
    
    //位置控制
    Leg2_UniA1.motor0_run.id = 0;
    Leg2_UniA1.motor0_run.motorType = MotorType::A1;
    Leg2_UniA1.motor0_run.mode = 10;
    Leg2_UniA1.motor0_run.T = 0.0; // 点击输出扭矩
    Leg2_UniA1.motor0_run.W = 0.1*9.1; // 电机转速
    Leg2_UniA1.motor0_run.Pos = 0;
    Leg2_UniA1.motor0_run.K_P = 0.2;
    Leg2_UniA1.motor0_run.K_W = 3.0;

    Leg2_UniA1.motor1_run.id = 1;
    Leg2_UniA1.motor1_run.motorType = MotorType::A1;
    Leg2_UniA1.motor1_run.mode = 10;
    Leg2_UniA1.motor1_run.T = 0.0; // 点击输出扭矩
    Leg2_UniA1.motor1_run.W = 0.1*9.1; // 电机转速
    Leg2_UniA1.motor1_run.Pos = 0;    
    Leg2_UniA1.motor1_run.K_P = 0.2;
    Leg2_UniA1.motor1_run.K_W = 3.0;

    // Leg3
    // Motor0是 Knee Joint; Motor1 是 Thigh Joint
    // Leg1 初始化
    Leg3_UniA1.motor0_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg3_UniA1.motor0_init.id = 0;
    Leg3_UniA1.motor0_init.motorType = MotorType::A1;
    Leg3_UniA1.motor0_init.mode = 10;
    Leg3_UniA1.motor0_init.T = 0.0;
    Leg3_UniA1.motor0_init.W = 0;
    Leg3_UniA1.motor0_init.Pos = 0;    
    Leg3_UniA1.motor0_init.K_P = 0.0;
    Leg3_UniA1.motor0_init.K_W = 0.0;   
    modify_data(&Leg3_UniA1.motor0_init);
    serial3.sendRecv(&Leg3_UniA1.motor0_init, &Leg3_UniA1.motor0_rev);
    Leg3_UniA1.pos0_init = Leg3_UniA1.motor0_rev.Pos;
    //std::cout << "Leg1,Motor0, INIT_Pos:    " << Leg1_UniA1.pos0_init << std::endl;

    Leg3_UniA1.motor1_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg3_UniA1.motor1_init.id = 1;
    Leg3_UniA1.motor1_init.motorType = MotorType::A1;
    Leg3_UniA1.motor1_init.mode = 10;
    Leg3_UniA1.motor1_init.T = 0.0;
    Leg3_UniA1.motor1_init.W = 0;
    Leg3_UniA1.motor1_init.Pos = 0;    
    Leg3_UniA1.motor1_init.K_P = 0.0;
    Leg3_UniA1.motor1_init.K_W = 0.0;   
    modify_data(&Leg3_UniA1.motor1_init);
    serial3.sendRecv(&Leg3_UniA1.motor1_init, &Leg3_UniA1.motor1_rev);
    Leg3_UniA1.pos1_init = Leg3_UniA1.motor1_rev.Pos;
    //std::cout << "Leg1,Motor1, INIT_Pos:    " << Leg1_UniA1.pos1_init << std::endl;

    //位置控制
    Leg3_UniA1.motor0_run.id = 0;
    Leg3_UniA1.motor0_run.motorType = MotorType::A1;
    Leg3_UniA1.motor0_run.mode = 10;
    Leg3_UniA1.motor0_run.T = 0.0; // 点击输出扭矩
    Leg3_UniA1.motor0_run.W = 0.1*9.1; // 电机转速
    Leg3_UniA1.motor0_run.Pos = 0;
    Leg3_UniA1.motor0_run.K_P = 0.2;
    Leg3_UniA1.motor0_run.K_W = 3.0;

    Leg3_UniA1.motor1_run.id = 1;
    Leg3_UniA1.motor1_run.motorType = MotorType::A1;
    Leg3_UniA1.motor1_run.mode = 10;
    Leg3_UniA1.motor1_run.T = 0.0; // 点击输出扭矩
    Leg3_UniA1.motor1_run.W = 0.1*9.1; // 电机转速
    Leg3_UniA1.motor1_run.Pos = 0;    
    Leg3_UniA1.motor1_run.K_P = 0.2;
    Leg3_UniA1.motor1_run.K_W = 3.0;


    // Leg4
    // Motor0是 Knee Joint; Motor1 是 Thigh Joint
    // Leg1 初始化
    Leg4_UniA1.motor0_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg4_UniA1.motor0_init.id = 0;
    Leg4_UniA1.motor0_init.motorType = MotorType::A1;
    Leg4_UniA1.motor0_init.mode = 10;
    Leg4_UniA1.motor0_init.T = 0.0;
    Leg4_UniA1.motor0_init.W = 0;
    Leg4_UniA1.motor0_init.Pos = 0;    
    Leg4_UniA1.motor0_init.K_P = 0.0;
    Leg4_UniA1.motor0_init.K_W = 0.0;   
    modify_data(&Leg4_UniA1.motor0_init);
    serial4.sendRecv(&Leg4_UniA1.motor0_init, &Leg4_UniA1.motor0_rev);
    Leg4_UniA1.pos0_init = Leg4_UniA1.motor0_rev.Pos;
    //std::cout << "Leg1,Motor0, INIT_Pos:    " << Leg1_UniA1.pos0_init << std::endl;

    Leg4_UniA1.motor1_rev.motorType = MotorType::A1;
    //初始化位置读取
    Leg4_UniA1.motor1_init.id = 1;
    Leg4_UniA1.motor1_init.motorType = MotorType::A1;
    Leg4_UniA1.motor1_init.mode = 10;
    Leg4_UniA1.motor1_init.T = 0.0;
    Leg4_UniA1.motor1_init.W = 0;
    Leg4_UniA1.motor1_init.Pos = 0;    
    Leg4_UniA1.motor1_init.K_P = 0.0;
    Leg4_UniA1.motor1_init.K_W = 0.0;   
    modify_data(&Leg4_UniA1.motor1_init);
    serial4.sendRecv(&Leg4_UniA1.motor1_init, &Leg4_UniA1.motor1_rev);
    Leg4_UniA1.pos1_init = Leg4_UniA1.motor1_rev.Pos;
    //std::cout << "Leg1,Motor1, INIT_Pos:    " << Leg1_UniA1.pos1_init << std::endl;

    //位置控制
    Leg4_UniA1.motor0_run.id = 0;
    Leg4_UniA1.motor0_run.motorType = MotorType::A1;
    Leg4_UniA1.motor0_run.mode = 10;
    Leg4_UniA1.motor0_run.T = 0.0; // 点击输出扭矩
    Leg4_UniA1.motor0_run.W = 0.1*9.1; // 电机转速
    Leg4_UniA1.motor0_run.Pos = 0;
    Leg4_UniA1.motor0_run.K_P = 0.2;
    Leg4_UniA1.motor0_run.K_W = 3.0;

    Leg4_UniA1.motor1_run.id = 1;
    Leg4_UniA1.motor1_run.motorType = MotorType::A1;
    Leg4_UniA1.motor1_run.mode = 10;
    Leg4_UniA1.motor1_run.T = 0.0; // 点击输出扭矩
    Leg4_UniA1.motor1_run.W = 0.1*9.1; // 电机转速
    Leg4_UniA1.motor1_run.Pos = 0;    
    Leg4_UniA1.motor1_run.K_P = 0.2;
    Leg4_UniA1.motor1_run.K_W = 3.0;
    */
}


void UnitreeA1::motor_stop()
{
  // Leg1
      // 电机停止运行
      Leg1_UniA1.motor0_stop.id = Leg1_UniA1.motor0_run.id;
      Leg1_UniA1.motor0_stop.motorType = Leg1_UniA1.motor0_run.motorType;
      Leg1_UniA1.motor0_stop.mode = 0;
      Leg1_UniA1.pos0_init = 0.0;
      modify_data(&Leg1_UniA1.motor0_stop);
      serial1.sendRecv(&Leg1_UniA1.motor0_stop, &Leg1_UniA1.motor0_rev);

      Leg1_UniA1.motor1_stop.id = Leg1_UniA1.motor1_run.id;
      Leg1_UniA1.motor1_stop.motorType = Leg1_UniA1.motor1_run.motorType;
      Leg1_UniA1.motor1_stop.mode = 0;
      Leg1_UniA1.pos1_init = 0.0;
      modify_data(&Leg1_UniA1.motor1_stop);
      serial1.sendRecv(&Leg1_UniA1.motor1_stop, &Leg1_UniA1.motor1_rev);

      Leg1_UniA1.motor0_rev.Pos = 0.0;
      Leg1_UniA1.motor1_rev.Pos = 0.0;

      /*
      // Leg2
      // 电机停止运行
      Leg2_UniA1.motor0_stop.id = Leg2_UniA1.motor0_run.id;
      Leg2_UniA1.motor0_stop.motorType = Leg2_UniA1.motor0_run.motorType;
      Leg2_UniA1.motor0_stop.mode = 0;
      Leg2_UniA1.pos0_init = 0.0;
      modify_data(&Leg2_UniA1.motor0_stop);
      serial2.sendRecv(&Leg2_UniA1.motor0_stop, &Leg2_UniA1.motor0_rev);

      Leg2_UniA1.motor1_stop.id = Leg2_UniA1.motor1_run.id;
      Leg2_UniA1.motor1_stop.motorType = Leg2_UniA1.motor1_run.motorType;
      Leg2_UniA1.motor1_stop.mode = 0;
      Leg2_UniA1.pos1_init = 0.0;
      modify_data(&Leg2_UniA1.motor1_stop);
      serial2.sendRecv(&Leg2_UniA1.motor1_stop, &Leg2_UniA1.motor1_rev);

      Leg2_UniA1.motor0_rev.Pos = 0.0;
      Leg2_UniA1.motor1_rev.Pos = 0.0;

      // Leg3
      // 电机停止运行
      Leg3_UniA1.motor0_stop.id = Leg3_UniA1.motor0_run.id;
      Leg3_UniA1.motor0_stop.motorType = Leg3_UniA1.motor0_run.motorType;
      Leg3_UniA1.motor0_stop.mode = 0;
      Leg3_UniA1.pos0_init = 0.0;
      modify_data(&Leg3_UniA1.motor0_stop);
      serial3.sendRecv(&Leg3_UniA1.motor0_stop, &Leg3_UniA1.motor0_rev);

      Leg3_UniA1.motor1_stop.id = Leg3_UniA1.motor1_run.id;
      Leg3_UniA1.motor1_stop.motorType = Leg3_UniA1.motor1_run.motorType;
      Leg3_UniA1.motor1_stop.mode = 0;
      Leg3_UniA1.pos1_init = 0.0;
      modify_data(&Leg3_UniA1.motor1_stop);
      serial3.sendRecv(&Leg3_UniA1.motor1_stop, &Leg3_UniA1.motor1_rev);

      Leg3_UniA1.motor0_rev.Pos = 0.0;
      Leg3_UniA1.motor1_rev.Pos = 0.0;

      // Leg4
      // 电机停止运行
      Leg4_UniA1.motor0_stop.id = Leg4_UniA1.motor0_run.id;
      Leg4_UniA1.motor0_stop.motorType = Leg4_UniA1.motor0_run.motorType;
      Leg4_UniA1.motor0_stop.mode = 0;
      Leg4_UniA1.pos0_init = 0.0;
      modify_data(&Leg4_UniA1.motor0_stop);
      serial4.sendRecv(&Leg4_UniA1.motor0_stop, &Leg4_UniA1.motor0_rev);

      Leg4_UniA1.motor1_stop.id = Leg4_UniA1.motor1_run.id;
      Leg4_UniA1.motor1_stop.motorType = Leg4_UniA1.motor1_run.motorType;
      Leg4_UniA1.motor1_stop.mode = 0;
      Leg4_UniA1.pos1_init = 0.0;
      modify_data(&Leg4_UniA1.motor1_stop);
      serial4.sendRecv(&Leg4_UniA1.motor1_stop, &Leg4_UniA1.motor1_rev);

      Leg4_UniA1.motor0_rev.Pos = 0.0;
      Leg4_UniA1.motor1_rev.Pos = 0.0;
      */
}