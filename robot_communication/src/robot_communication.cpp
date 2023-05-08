#include "robot_communication/robot_communication.hpp"
#include <cmath>


using std::placeholders::_1;
using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    turn_on_robot Robot_Control;
    
    Robot_Control.Control();
    return 0;
}


turn_on_robot::turn_on_robot() : rclcpp::Node ("LW_Robot_Commu_Node")
{
    // publish the info of joint_motor_angle

    Joint_Angle_Sub = this->create_subscription<robot_msgs::msg::JointAngle>("desire_joint_angle",100,std::bind(&turn_on_robot::Joint_Angle_Callback, this, _1));

    try
    { 
        Stm32_Serial.setPort("/dev/ttyUSB0"); //Select the serial port number to enable //选择要开启的串口号
        Stm32_Serial.setBaudrate(115200); //Set the baud rate //设置波特率
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open(); //Open the serial port //开启串口
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
    }
    if(Stm32_Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(),"Serial port opened"); //Serial port opened successfully //串口开启成功提示
    }
}


// send the data to the lower machine,considering that the value of torque would be larger than 100,even 1000, we should use type of 'int' to transit the data
void turn_on_robot::Joint_Angle_Callback(const robot_msgs::msg::JointAngle::SharedPtr des_joint_angle_ptr)
{   


    // multiply 100 to "float" data
    Send_Data.des_joint_angle[0] = (int16_t)des_joint_angle_ptr->leg1_hip_angle * 1000;
    Send_Data.des_joint_angle[1] = (int16_t)des_joint_angle_ptr->leg1_hl_angle * 1000;
    Send_Data.des_joint_angle[2] = (int16_t)des_joint_angle_ptr->leg1_hu_angle * 1000;

    Send_Data.des_joint_angle[3] = (int16_t)des_joint_angle_ptr->leg2_hip_angle * 1000;
    Send_Data.des_joint_angle[4] = (int16_t)des_joint_angle_ptr->leg2_hl_angle * 1000;
    Send_Data.des_joint_angle[5] = (int16_t)des_joint_angle_ptr->leg2_hu_angle * 1000;

    Send_Data.des_joint_angle[6] = (int16_t)des_joint_angle_ptr->leg3_hip_angle * 1000;
    Send_Data.des_joint_angle[7] = (int16_t)des_joint_angle_ptr->leg3_hl_angle * 1000;
    Send_Data.des_joint_angle[8] = (int16_t)des_joint_angle_ptr->leg3_hu_angle * 1000;

    Send_Data.des_joint_angle[9] = (int16_t)des_joint_angle_ptr->leg4_hip_angle * 1000;
    Send_Data.des_joint_angle[10] = (int16_t)des_joint_angle_ptr->leg4_hl_angle * 1000;
    Send_Data.des_joint_angle[11] = (int16_t)des_joint_angle_ptr->leg4_hu_angle * 1000;
    

    //RCLCPP_INFO(this->get_logger(),"I'm sengind the message to stm32!");
    Send_Data.tx[0]=FRAME_HEADER;//帧头0X7B
    
    for (int i=0;i<12;i++){
        Send_Data.tx[1+2*i] = Send_Data.des_joint_angle[i] >> 8; 
        Send_Data.tx[2+2*i] = Send_Data.des_joint_angle[i];
    }

    //cout << "get data already " << endl;

    Send_Data.tx[25] = Check_Sum(24,SEND_DATA_CHECK);
    Send_Data.tx[26] = FRAME_TAIL; //帧尾0X7D

    try
    {
        Stm32_Serial.write(Send_Data.tx,sizeof(Send_Data.tx));//通过串口向下位机发送数据 
    }
    catch (serial::IOException& e)   
    {
        RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); //如果发送数据失败，打印错误信息
    }
}



// get Motor Angle from lower machine 
bool turn_on_robot::Get_Motor_Info() 
{
  short transition_16=0; //Intermediate variable //中间变量
  uint8_t i=0,check=0, error=1,Receive_Data_Pr[1]; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  static int count; //Static variable for counting //静态变量，用于计数
  Stm32_Serial.read(Receive_Data_Pr,sizeof(Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据
  //RCLCPP_INFO(this->get_logger(),"***test****");
  Receive_Data.rx[count] = Receive_Data_Pr[0]; //Fill the array with serial data //串口数据填入数组

  Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[51];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
    count++;
  else 
  	count=0;
  if(count == 52) //Verify the length of the packet //验证数据包的长度
  {

    count=0;  //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
    if(Receive_Data.Frame_Tail == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
    {
      check=Check_Sum(50, READ_DATA_CHECK);  //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错
        
      if(check == Receive_Data.rx[50])  
      {
        error=0;  //XOR bit check successful //异或位校验成功
      }
      if(error == 0)
      {

        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        
        // get joint_motor_angle, motor1: shoulder; motor2: hip; motor3:knee
        for (int i=0;i<12;i++){
            Joint_Motor_Info.Leg_Motor_Angle[i] = Data_Trans_16(Receive_Data.rx[2+2*i],Receive_Data.rx[2+1+2*i]);
        }

        // get joint angle rpm data
        for (int i=0;i<12;i++){
            Joint_Motor_Info.Leg_Motor_RPM(i) = Data_Trans_16(Receive_Data.rx[26+2*i],Receive_Data.rx[26+1+2*i]);
        }
          

        return true;
      }
    }
  }
  return false;
}    




void turn_on_robot::Control()
{
    rclcpp::Node::SharedPtr node_(this); // 创建基类指针，指向子类对象this

    while(rclcpp::ok())
    {
        
        rclcpp::spin_some(node_); // 运行正常
        
        
    }
}





// from stm32, the type of the data is short(int_16)
float turn_on_robot::Data_Trans_16(uint8_t Data_High,uint8_t Data_Low)
{
  float output;

  output = (int16_t)(Data_High << 8 | Data_Low) * 0.001; // short type

  return output;     
}


unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;
    if(mode==0) //接收数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //按位异或
        }
    }
    if(mode==1) //发送数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k];   //按位异或
        }
    }
    return check_sum; //返回按位异或结果
}






// send the message to make the robot return to initialized state.
turn_on_robot::~turn_on_robot() 
{
    
    Send_Data.des_joint_angle[0] = (int16_t)0;
    Send_Data.des_joint_angle[1] = (int16_t)0;
    Send_Data.des_joint_angle[2] = (int16_t)0;

    Send_Data.des_joint_angle[3] = (int16_t)0;
    Send_Data.des_joint_angle[4] = (int16_t)0;
    Send_Data.des_joint_angle[5] = (int16_t)0;

    Send_Data.des_joint_angle[6] = (int16_t)0;
    Send_Data.des_joint_angle[7] = (int16_t)0;
    Send_Data.des_joint_angle[8] = (int16_t)0;

    Send_Data.des_joint_angle[9] = (int16_t)0;
    Send_Data.des_joint_angle[10] = (int16_t)0;
    Send_Data.des_joint_angle[11] = (int16_t)0;

    //RCLCPP_INFO(this->get_logger(),"I'm sengind the message to stm32!");
    Send_Data.tx[0]=FRAME_HEADER;//帧头0X7B
    
    for (int i=0;i<12;i++){
        Send_Data.tx[1+2*i] = Send_Data.des_joint_angle[i] >> 8; 
        Send_Data.tx[2+2*i] = Send_Data.des_joint_angle[i];
    }


    try
    {
        Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
    }
    catch (serial::IOException& e)   
    {
        RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
    }
    
    Stm32_Serial.close(); //Close the serial port //关闭串口  
    RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息

}

