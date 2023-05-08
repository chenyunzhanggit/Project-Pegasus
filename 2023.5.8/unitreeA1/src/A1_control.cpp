#include "A1_control.hpp"



SerialPort serial1("/dev/ttyUSB0");   // for leg1 LF
//SerialPort serial2("/dev/ttyUSB1"); // for leg2 RF
//SerialPort serial3("/dev/ttyUSB2"); // for leg3 RH
//SerialPort serial4("/dev/ttyUSB3"); // for leg4 LH

UnitreeA1::UnitreeA1(): Node("UnitreeA1_control"), node_(this)
{
        linear_diff_timer = node_->create_wall_timer(10ms,std::bind(&UnitreeA1::linear_diff_timer_callback,this));
        A1Motor_Sub = this->create_subscription<motor_canmsgs::msg::A1MotorMsg>("A1Motor_Ctrl",100, std::bind(&UnitreeA1::A1Motor_Sub_CB, this, _1));
        
        _percent0 = 0.0; // _percent 在每一次给新位置后都需要重新置零
        _percent1 = 0.0; // _percent 在每一次给新位置后都需要重新置零
        _duration = 50; // 10ms * 100 = 1s

        motor_init();       
}






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  UnitreeA1 unitree_motor;
  unitree_motor.motor_control();
  
  return 0;
}

void UnitreeA1::motor_control()
{

    while(rclcpp::ok())
    {  
      rclcpp::spin_some(node_); // 运行正常
    }

       
}
  



void UnitreeA1::A1Motor_Sub_CB(const motor_canmsgs::msg::A1MotorMsg::SharedPtr A1Motor_data_ptr)
{
    /* 
      宇树电机采用的是力位混合控制的模式， tao = tao_ff + k_p*(p_des-p) + k_w*(w_des-w)
      除了 p_des 都是可以直接输入或返回的，所以整个过程中需要我们计算的就是 p_des 这个部分
    */
    cout << "/****** Callback *******/ " << endl;
    

    switch (A1Motor_data_ptr->leg_id)
    {
    
      case 1:
      {
        //到达目标位置,当前位置作为下一次的初值
        Leg1_UniA1.pos0_last = Leg1_UniA1.pos0_cur;
        Leg1_UniA1.pos1_last = Leg1_UniA1.pos1_cur;
        Leg1_UniA1.theta0_last = Leg1_UniA1.theta0_cur;
        Leg1_UniA1.theta1_last = Leg1_UniA1.theta1_cur;

        //std::cout << "theta0_cur    " << Leg1_UniA1.theta0_cur << std::endl;
        //std::cout << "theta1_cur    " << Leg1_UniA1.theta1_cur << std::endl;
        

        if (A1Motor_data_ptr->motor_id == 0){
            Leg1_UniA1.motor0_run.T = A1Motor_data_ptr->t;
            Leg1_UniA1.motor0_run.W = A1Motor_data_ptr->w;
            Leg1_UniA1.theta0_des = A1Motor_data_ptr->pos;
            Leg1_UniA1.motor0_run.K_P = A1Motor_data_ptr->k_p;
            Leg1_UniA1.motor0_run.K_W = A1Motor_data_ptr->k_w;
            _percent0 = 0.0;

        }
        else if (A1Motor_data_ptr->motor_id == 1){
            Leg1_UniA1.motor1_run.T = A1Motor_data_ptr->t;
            Leg1_UniA1.motor1_run.W = A1Motor_data_ptr->w;
            Leg1_UniA1.theta1_des = A1Motor_data_ptr->pos;
            Leg1_UniA1.motor1_run.K_P = A1Motor_data_ptr->k_p;
            Leg1_UniA1.motor1_run.K_W = A1Motor_data_ptr->k_w;
            _percent1 = 0.0;


        }

        
        

        break;
      } 
      case 2:
      {
        
        break;
      } 
      case 3:
      {
        
        break;
      } 
      case 4:
      {
        
        break;
      } 
         


    }
}


void UnitreeA1::linear_diff_timer_callback()
{
    _percent0 += 1.0 / _duration; // 定时器时间 * duration times = 完成这个动作消耗的总时间 
    
    _percent0 = _percent0 > 1 ? 1:_percent0;//限制到1
    
    theta_des_temp = (1-_percent0)*Leg1_UniA1.theta0_last + _percent0 * Leg1_UniA1.theta0_des;
    Leg1_UniA1.pos0_delta = - (theta_des_temp - Leg1_UniA1.theta0_last) * 9.1; //这一步是基于先前的初始位置来计算电机需要走多少 pos_delta，是必须的
    Leg1_UniA1.motor0_run.Pos = Leg1_UniA1.pos0_last + Leg1_UniA1.pos0_delta;//Leg1_UniA1.pos0_delta;


    modify_data(&Leg1_UniA1.motor0_run);
     
    serial1.sendRecv(&Leg1_UniA1.motor0_run, &Leg1_UniA1.motor0_rev);

    extract_data(&Leg1_UniA1.motor0_rev);

    Leg1_UniA1.pos0_cur = Leg1_UniA1.motor0_rev.Pos;


    Leg1_UniA1.theta0_cur = - (Leg1_UniA1.pos0_cur - Leg1_UniA1.pos0_last)/9.1 + Leg1_UniA1.theta0_last; // 计算当前的角度值并发送出去




    _percent1 += 1.0 / _duration; // 定时器时间 * duration times = 完成这个动作消耗的总时间 
    
    _percent1 = _percent1 >1 ? 1:_percent1;//限制到1

    theta_des_temp = (1-_percent1)*Leg1_UniA1.theta1_last +  _percent1 * Leg1_UniA1.theta1_des;
    Leg1_UniA1.pos1_delta = - (theta_des_temp - Leg1_UniA1.theta1_last) * 9.1; //这一步是基于先前的初始位置来计算电机需要走多少 pos_delta，是必须的
    Leg1_UniA1.motor1_run.Pos = Leg1_UniA1.pos1_last + Leg1_UniA1.pos1_delta;//Leg1_UniA1.pos1_delta;

    modify_data(&Leg1_UniA1.motor1_run);
     
    serial1.sendRecv(&Leg1_UniA1.motor1_run, &Leg1_UniA1.motor1_rev);

    extract_data(&Leg1_UniA1.motor1_rev);

    Leg1_UniA1.pos1_cur = Leg1_UniA1.motor1_rev.Pos;
    

    Leg1_UniA1.theta1_cur = - (Leg1_UniA1.pos1_cur - Leg1_UniA1.pos1_last)/9.1 + Leg1_UniA1.theta1_last; // 计算当前的角度值并发送出去


    std::cout << "theta0_cur    " << Leg1_UniA1.theta0_cur << std::endl;
    std::cout << "theta1_cur    " << Leg1_UniA1.theta1_cur << std::endl;
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
    std::cout << "Leg1,Motor0, INIT_Pos:    " << Leg1_UniA1.pos0_init << std::endl;

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
    std::cout << "Leg1,Motor1, INIT_Pos:    " << Leg1_UniA1.pos1_init << std::endl;

    Leg1_UniA1.pos0_last =  Leg1_UniA1.pos0_init;
    Leg1_UniA1.pos1_last =  Leg1_UniA1.pos1_init;

    Leg1_UniA1.pos0_cur =  Leg1_UniA1.pos0_init;
    Leg1_UniA1.pos1_cur =  Leg1_UniA1.pos1_init;



    Leg1_UniA1.theta0_init = 140/RAD;
    Leg1_UniA1.theta1_init = -50/RAD;

    Leg1_UniA1.theta0_des = Leg1_UniA1.theta0_init ; // 初始角度PASSIVE
    Leg1_UniA1.theta1_des = Leg1_UniA1.theta1_init ; // 初始角度PASSIVE

    Leg1_UniA1.theta0_last = Leg1_UniA1.theta0_init;
    Leg1_UniA1.theta1_last = Leg1_UniA1.theta1_init;


    //位置控制
    Leg1_UniA1.motor0_run.id = 0;
    Leg1_UniA1.motor0_run.motorType = MotorType::A1;
    Leg1_UniA1.motor0_run.mode = 10;
    Leg1_UniA1.motor0_run.T = 0.0; // 点击输出扭矩
    Leg1_UniA1.motor0_run.W = 0.0; // 电机转速
    Leg1_UniA1.motor0_run.Pos = 0;
    Leg1_UniA1.motor0_run.K_P = 0.5;
    Leg1_UniA1.motor0_run.K_W = 3.0;

    Leg1_UniA1.motor1_run.id = 1;
    Leg1_UniA1.motor1_run.motorType = MotorType::A1;
    Leg1_UniA1.motor1_run.mode = 10;
    Leg1_UniA1.motor1_run.T = 0.0; // 点击输出扭矩
    Leg1_UniA1.motor1_run.W = 0.0; // 电机转速
    Leg1_UniA1.motor1_run.Pos = 0;    
    Leg1_UniA1.motor1_run.K_P = 0.5;
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


UnitreeA1::~UnitreeA1()
{
    motor_stop();
}