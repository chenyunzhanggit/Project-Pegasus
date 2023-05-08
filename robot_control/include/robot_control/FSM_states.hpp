#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#define PASSIVE     1 // 初始状态与结束状态
#define FIXED_STAND 2 // 仅站立
#define WALK        3 // WALK Gait

class FSM_STATES
{
    public:

        FSM_STATES();

        std_msgs::msg::Int16 state;
        
        void do_loop();

        ~FSM_STATES();
        
    private:



};