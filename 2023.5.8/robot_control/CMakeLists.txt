cmake_minimum_required(VERSION 3.5)
project(robot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_canmsgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)




add_executable(motor_datasend src/motor_datasend.cpp)
ament_target_dependencies(motor_datasend rclcpp std_msgs motor_canmsgs)
add_executable(state_switch src/state_switch.cpp)
ament_target_dependencies(state_switch rclcpp std_msgs motor_canmsgs geometry_msgs)
add_executable(robot_control src/robot_control.cpp src/FSM_states.cpp src/leg_states.cpp)
ament_target_dependencies(robot_control rclcpp std_msgs motor_canmsgs geometry_msgs)
add_executable(traj_generate src/traj_generate.cpp src/FSM_states.cpp src/leg_states.cpp)
ament_target_dependencies(traj_generate rclcpp std_msgs motor_canmsgs geometry_msgs)



include_directories(include/robot_control)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

install(TARGETS
  motor_datasend
  state_switch
  robot_control
  traj_generate
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
