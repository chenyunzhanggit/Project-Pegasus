
#ifndef __ros2_socketcan_H__
#define __ros2_socketcan_H__


#include <linux/can/raw.h>

#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

#include "log.h"

const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

class ros2socketcan : public rclcpp::Node
{
    public:

        ros2socketcan(std::string can_socket2 = "can0");
        
        void Init(const char* can_socket = "can0");
        void CanPublisher(const can_msgs::msg::Frame::SharedPtr msg);
        ~ros2socketcan();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr test_pub_;
        
        
        can_msgs::msg::Frame current_frame;
        
        void CanSendConfirm();
        
        
        
        void CanSend(const can_msgs::msg::Frame msg);
        
        void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);
        

        void stop();
        
        boost::asio::io_service ios;
        boost::asio::posix::basic_stream_descriptor<> stream;
        boost::asio::signal_set signals;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame rec_frame;
        struct ifreq ifr;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive;
        std::stringstream topicname_transmit;
        std::stringstream servername;
};
#endif
