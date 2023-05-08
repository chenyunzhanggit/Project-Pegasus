cmake_minimum_required(VERSION 3.5)
project(unitreeA1)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(motor_canmsgs REQUIRED)


include_directories(
    include
    include/unitreeMotor
    
)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

link_directories(
    lib
)

add_executable(test_motor src/test_motor.cpp)
target_link_libraries(test_motor libunitreeMotorSDK_Linux64.so)
ament_target_dependencies(test_motor rclcpp std_msgs motor_canmsgs)

add_executable(A1_control src/A1_control.cpp)
target_link_libraries(A1_control libunitreeMotorSDK_Linux64.so)
ament_target_dependencies(A1_control rclcpp std_msgs motor_canmsgs)


#add_executable(swing_control src/swing_control.cpp)
#target_link_libraries(swing_control libunitreeMotorSDK_Linux64.so)
#ament_target_dependencies(swing_control rclcpp std_msgs)

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

install(TARGETS 
  test_motor 
  A1_control
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
