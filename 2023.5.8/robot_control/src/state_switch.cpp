#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include "std_msgs/msg/int16.hpp"
#include "FSM_states.hpp"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_q 0x71
#define KEYCODE_z 0x7A
#define KEYCODE_x 0x78


class TeleopTurtle : public rclcpp::Node
{
public:
  TeleopTurtle(): Node("State_Switch"){
    FSM_state_pub = this->create_publisher<std_msgs::msg::Int16>("state_switch", 5);
    
  }
  void keyLoop();
  
private:
  
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr FSM_state_pub;

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  TeleopTurtle teleop_turtle;

  signal(SIGINT, quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  // bool dirty=false;
  std_msgs::msg::Int16 state;
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to pob command.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
  
    switch(c)
    {
      case KEYCODE_L:
        
        break;
      case KEYCODE_R:
        
        break;
      case KEYCODE_U:
        
        break;
      case KEYCODE_D:
        
        break;
      case KEYCODE_z:
        state.data = PASSIVE; 
        break;
      case KEYCODE_x:
        state.data = FIXED_STAND;      
        break;
      case KEYCODE_q:
        
        break;
    }
    
    FSM_state_pub->publish(state);

  }

  return;
}


