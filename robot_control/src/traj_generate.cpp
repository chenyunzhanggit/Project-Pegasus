#include "traj_generate.hpp"



Traj_Generator::Traj_Generator() : Node("Traj_Generator"), node_(this)
{
  Traj_pub_timer = node_->create_wall_timer(500ms,std::bind(&Traj_Generator::Traj_pub_callback,this));
  Motor_Candata_Pub = this->create_publisher<motor_canmsgs::msg::MotorMsg>("Shoulder_Linear_Motor_Ctrl", 10);
  A1Motor_Data_Pub = this->create_publisher<motor_canmsgs::msg::A1MotorMsg>("A1Motor_Ctrl", 100);

  flag = 0;
  Td = 12.0;

  traj_p_0(0) = 0;
  traj_p_0(1) = 0;
  traj_p_0(2) = 0;

  traj_p_f(0) = 0.08;
  traj_p_f(1) = 0;
  traj_p_f(2) = 0;

  for (int i=0;i<3;i++){
    pass_point(i) = (traj_p_0(i)+traj_p_f(i))/2.0;
  }

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  Traj_Generator traj_generator;
  traj_generator.Traj_Loop();
  
  return 0;
}

void Traj_Generator::Traj_Loop()
{

  double t; // 实际时间
  t= 0.0;
  
  
  while(rclcpp::ok())
    { 
      if (flag == 0)
      {
          start = rclcpp::Node::now();

          pos_Generator(0.1,t);

          end = rclcpp::Node::now();
          t += (end-start).seconds();

          if (t >= Td)
          {
            flag = 1;
            //t = 0.0;
          }
            
          //cout << "t = " << t << endl;
          
      }
      

      rclcpp::spin_some(node_); // 运行正常,回调
    }

}



void Traj_Generator::Traj_pub_callback()
{
  //cout << "********************** timer callback **********************" << endl;
  
  
  //cout << "x = " << traj_p_out(0) << endl;
  //cout << "y = " << traj_p_out(1) << endl;
  //cout << "z = " << traj_p_out(2) << endl;
  //cout << "gamma = " << pos_output[0] << endl;
  cout << "alpha = " << pos_output[1] * RAD << endl;
  cout << "beta = " <<  pos_output[2] * RAD << endl;
  


  //cout <<  "knee : " << des_joint_angle(2) << endl;
  //cout <<  "hip : " << des_joint_angle(1) << endl;
  A1Motor_Data_Send(1,0, 0.0, 0.0, (115/RAD), 0.5, 3.0);
  A1Motor_Data_Send(1,1, 0.0, 0.0, (-36/RAD), 0.5, 3.0); 
  //A1Motor_Data_Send(1,1, 0.0, 0.0, temp, 0.15, 2.0); 
}

void Traj_Generator::pos_Generator(double height, double t)
{
      // 1. 生成期望轨迹 -- 贝塞尔曲线
      for (int i=0;i<3;i++){
         pass_point[i] = (traj_p_0[i]+traj_p_f[i])/2.0;
      }
      pass_point[2] = pass_point[2] + height;

      double T_seg = Td/2.0;
      if (t < T_seg){
         traj_p_out = cubicBezier(traj_p_0, traj_p_0+pass_point,t,T_seg);
      }
      else{
         traj_p_out = cubicBezier(traj_p_0+pass_point,traj_p_f, t-T_seg , Td-T_seg );  // 传入实际时间为  t-T_seg (总时间-已经耗时的T_seg),传入总持续时间为 Td-T_seg(总摆动周期持续时间 - 前一段经过的时间)
      }
      
      // 2. 逆解各关节的pos(rad)
      inverse_kinematic();

}


coordinate Traj_Generator::cubicBezier(coordinate p0, coordinate pf,double t,double Tm)
{
    coordinate p_out;
    double b = ((t/Tm)*(t/Tm)*(t/Tm) + 3*(t/Tm)*(t/Tm)*((Tm-t)/Tm));
    for (int i=0;i<3;i++){
        p_out(i) = p0(i) + b*(pf(i)-p0(i));
    }
    return p_out;

}

void Traj_Generator::inverse_kinematic()
{

    double l0 = 0.106; // distance from hip joint to hu joint -- 腿长
    double l1 = 0.250;
    double l2 = 0.200;



    //2023/5/6 后面会改成静态变量
    // inverse kinematics
    double alpha,beta,gamma;
    double lxz_,n;

    double init_theta1 = 0.0;
    double init_theta2 = -50/RAD;
    double init_theta3 = 140/RAD;

    // initialized coordinate
    double x_init = l1 * sin(init_theta2) + l2*sin(init_theta2+init_theta3);
    double y_init = l0;
    double z_init = -cos(init_theta1) * (l1*cos(init_theta2)+ l2*cos(init_theta2+init_theta3));    // L0 =0

    double des_x,des_y,des_z;


    /*
    //前腿前摆
    des_x = x_init + traj_p_out(0);
    des_y = y_init + traj_p_out(1);  // y is a minus value, reasoning by the acutal model calculation. 
    des_z = z_init + traj_p_out(2);
    */
    // 后腿前摆
    des_x = x_init - traj_p_out(0);
    des_y = y_init + traj_p_out(1);  // y is a minus value, reasoning by the acutal model calculation. 
    des_z = z_init + traj_p_out(2);

    
    lxz_ = sqrt(des_y*des_y + des_z*des_z - l0*l0 + des_x*des_x);
    n = (lxz_*lxz_ - l1*l1 - l2*l2)/(2*l1);

    gamma = -(-atan(des_y/des_z)) + atan(l0/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));        //hip.  In a normal model it always suppose that y is a negative value, but in my model it's a positive one. So I have to add a minus sign.
    
    alpha = acos((l1+n)/lxz_) - atan(des_x/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));         //hu

    beta = -acos(n/l2);                                                                        //hl

    float t0,t1,t2;   //rad of 3 joint 

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

    pos_output[0] = -(float)gamma;
    pos_output[1] = -(float)alpha;
    pos_output[2] = -(float)beta;



    /*
    end_vel(1) = traj.x_vel;
    end_vel(0) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    */

    /*
    end_vel(0) = traj.x_vel;
    end_vel(1) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    */
    desire_ang_vel = Jacobian_Inv * end_vel;

}



int Traj_Generator::sgn(float num)
{
  if (num >= 0){
      return 1;
  }
  else{
      return -1;
  }
}




void Traj_Generator::Motor_CANdata_Send(uint32_t CAN_id, uint8_t Motor_id, _Float64 data)
{
    shoulder_linear_motor_msg.can_id = CAN_id;
    shoulder_linear_motor_msg.motor_id = Motor_id;
    shoulder_linear_motor_msg.data = data;
    Motor_Candata_Pub->publish(shoulder_linear_motor_msg);
}

void Traj_Generator::A1Motor_Data_Send(uint8_t leg_id, uint8_t motor_id, float t, float w,float pos,
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


Traj_Generator::~Traj_Generator()
{


}





















void Traj_Generator::test(double t)
{
  // -50 ~ -60
  double Tm;
  Tm = 1;
  des_joint_angle(2) = (140 * (Tm-t)/Tm  + 150 * t/Tm)/RAD; 

  if (des_joint_angle(2) > 150.0/RAD)
    des_joint_angle(2) = 150/RAD;

  //cout << "pos = " << des_joint_angle(2) << endl;

  des_joint_angle(1) = (-50 * (Tm-t)/Tm  + (-60) * t/Tm)/RAD; 

  if (des_joint_angle(1) < -60.0/RAD)
    des_joint_angle(1) = -60.0/RAD;

  //cout << "pos = " << des_joint_angle(1) << endl;

}


void Traj_Generator::test_traj(double t)
{
    
    double Tm = 2.0;  //T of the wobble phase
    double S = 0.1;
    double H = 0.1;
    double l0 = 0.106; // distance from hip joint to hu joint 
    double l1 = 0.250;
    double l2 = 0.200;

    //cout << "t = " << t << endl;

      if (t < Tm){   // 4: T of the wobble phase
        double Fe_t = t/Tm - 1/(4*Pi) * sin(4*Pi*t/Tm);

        traj.x = S * (t/Tm - 1/(2*Pi) * sin((2*Pi*t)/Tm));
        traj.y = 0.0;
        traj.z = H * (sgn(Tm/2-t) * (2*Fe_t-1)+1);

        traj.x_vel = S * (1/Tm - 1/Tm * cos(2*Pi*t/Tm));
        traj.y_vel = 0.0;
        traj.z_vel = H * (sgn(Tm/2-t) * 2 *(1/Tm - 1/Tm * cos(4*Pi*t/Tm)));

        /*
        traj.x = -traj.x;
        traj.y = -traj.y;
        traj.z = traj.z;

        traj.x_vel = -traj.x_vel;
        traj.y_vel = -traj.y_vel;
        traj.z_vel = traj.z_vel;
        */
      }
    else if (t >= Tm &&  t <= 2*Tm){
         
        traj.x = S*((2*Tm-t)/Tm + 1/(2*Pi)*sin(2*Pi*t/Tm));

        traj.y = 0.0;
        traj.z = 0.0;

        traj.x_vel = S * (-1/Tm + 1/Tm * cos(2*Pi*t/Tm));
        traj.y_vel = 0.0;
        traj.z_vel = 0.0;

        /*
        traj.x = -traj.x;
        traj.y = -traj.y;
        traj.z = traj.z;

        traj.x_vel = -traj.x_vel;
        traj.y_vel = -traj.y_vel;
        traj.z_vel = traj.z_vel; 
        */
     }
    // he unit of the above velocity is m/s


    /*
    cout << "traj_x = " << traj.x <<endl;

    cout << "traj_y = " << traj.y <<endl;
    cout << "traj_z = " << traj.z <<endl;
    

    
    cout << "traj_x_vel = " << traj.x_vel <<endl;

    cout << "traj_y_vel = " << traj.y_vel <<endl;
    cout << "traj_z_vel = " << traj.z_vel <<endl;
    */
    
    // inverse kinematics
    double alpha,beta,gamma;
    double lxz_,n;

    
    double init_theta1 = 0.0;
    double init_theta2 = -50/RAD;
    double init_theta3 = 140/RAD;

    // initialized coordinate
    double x_init = l1 * sin(init_theta2) + l2*sin(init_theta2+init_theta3);
    double y_init = l0;
    double z_init = -cos(init_theta1) * (l1*cos(init_theta2)+ l2*cos(init_theta2+init_theta3));    // L0 =0

    double des_x,des_y,des_z;

    des_x = x_init - traj.x;
    des_y = y_init + traj.y;  // y is a minus value, reasoning by the acutal model calculation. 
    des_z = z_init + traj.z;

    //traj.y = h;
    
    lxz_ = sqrt(des_y*des_y + des_z*des_z - l0*l0 + des_x*des_x);
    n = (lxz_*lxz_ - l1*l1 - l2*l2)/(2*l1);

    /*
    // 前腿前摆
    gamma = -(-atan(des_y/des_z)) + atan(l0/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));        //hip.  In a normal model it always suppose that y is a negative value, but in my model it's a positive one. So I have to add a minus sign.
    
    alpha = acos((l1+n)/lxz_) - atan(des_x/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));         //hu

    beta = -acos(n/l2);     
    */

    // 后退前摆
    gamma = -(-atan(des_y/des_z)) + atan(l0/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));        //hip.  In a normal model it always suppose that y is a negative value, but in my model it's a positive one. So I have to add a minus sign.
    
    alpha = acos((l1+n)/lxz_) + atan(des_x/(sqrt(des_y*des_y + des_z*des_z - l0*l0)));         //hu

    beta = -acos(n/l2);                                                                    //hl

    
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

    des_joint_angle(0) = -(float)gamma;
    des_joint_angle(1) = -(float)alpha;
    des_joint_angle(2) = -(float)beta;

    /*
    cout << "gamma = " << des_joint_angle(0) * RAD << endl;
    cout << "alpha = " << des_joint_angle(1) * RAD<< endl;
    cout << "beta = " << des_joint_angle(2) * RAD<< endl;
    */

    end_vel(1) = traj.x_vel;
    end_vel(0) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    
    /*
    end_vel(0) = traj.x_vel;
    end_vel(1) = traj.y_vel;
    end_vel(2) = traj.z_vel;
    */
    desire_ang_vel = Jacobian_Inv * end_vel;
   
    //cout  << "desire_angular_velocity: \n" << desire_ang_vel << endl;


}