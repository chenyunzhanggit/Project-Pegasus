cmake_minimum_required(VERSION 3.5)
project(imu_publish)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  saber_ros_inc
  saber_ros_src
  config_tool
)


add_executable(imu main.cpp saber_ros_src/saber_config.cpp saber_ros_src/saber_protocol.cpp  saber_ros_src/saber_serial.cpp saber_ros_src/saber_tool.cpp config_tool/cJSON.c)
ament_target_dependencies(imu rclcpp std_msgs sensor_msgs)

install(TARGETS
imu

DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


ament_package()
