#include "main.h"
//std include
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include <chrono>
#include <functional>
#include <memory>

//ros include
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>

//usr include
#include "saber_ros_inc/saber_serial.h"
#include "saber_ros_inc/saber_macro.h"
#include "saber_ros_inc/saber_protocol.h"
#include "saber_ros_inc/saber_config.h"
#include "saber_ros_inc/saber_tool.h"

using namespace std;

class IMUPublisher : public rclcpp::Node
{
  public:
    IMUPublisher(): Node("imu_publisher"), count_(0)
    {
        dataBuf = &frmBuf[0];
        nFD = SaberInitConfig("/home/chenyun/ROS2/legged_wheeled_robot/src/imu_publish/saber_cfg.json");

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("Imu_data", 20);
        timer_ = this->create_wall_timer(10ms, std::bind(&IMUPublisher::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
        sensor_msgs::msg::Imu imuMsg;
        //step 2: align Saber data frame from the serial stream
        packLengthFW = SaberAlign(nFD, dataBuf);
        if(packLengthFW == 0)
        {
            std::cout << "Header Searching ... " << std::endl;
        }
        
        //step 4:parser a whole frame to generate ros publish data         
        SaberParserDataPacket(&saberDataHandle, &dataBuf[SABER_HEAD_LEN], packLengthFW,fpLog);

        imuMsg.header.stamp = rclcpp::Node::now();
        imuMsg.header.frame_id = "imu_link";

        //C2G C3G C4 C6 C7
        imuMsg.linear_acceleration.x = saberDataHandle.accLinear.accX;
        imuMsg.linear_acceleration.y = saberDataHandle.accLinear.accY;
        imuMsg.linear_acceleration.z = saberDataHandle.accLinear.accZ;

        imuMsg.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
        imuMsg.orientation_covariance[4] = 1e6;
        imuMsg.orientation_covariance[8] = 1e-6;

        imuMsg.angular_velocity.x = saberDataHandle.gyroCal.gyroX*DEG_RAD;
        imuMsg.angular_velocity.y = saberDataHandle.gyroCal.gyroY*DEG_RAD;
        imuMsg.angular_velocity.z = saberDataHandle.gyroCal.gyroZ*DEG_RAD;

        imuMsg.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
        imuMsg.angular_velocity_covariance[4] = 1e6;
        imuMsg.angular_velocity_covariance[8] = 1e-6;

        imuMsg.orientation.w = saberDataHandle.quat.Q0.float_x;
        imuMsg.orientation.x = saberDataHandle.quat.Q1.float_x;
        imuMsg.orientation.y = saberDataHandle.quat.Q2.float_x;
        imuMsg.orientation.z = saberDataHandle.quat.Q3.float_x;

        count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing count: '%d'", count_);
        publisher_->publish(imuMsg);
    }

    unsigned char nFD = 0;
    int packLengthFW = 0;
    int pkgLen = 0;
    int pubCnt = 0;
    FILE *fpLog = NULL;
    u8 ret = 0;
    int seq = 0;
    bool met;
    int cycleCnt = 0;
    int errCnt = 0;
    unsigned char * dataBuf = NULL;
    unsigned char frmBuf[256] = { 0 };
    SaberData saberDataHandle;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}

