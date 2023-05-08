// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_canmsgs:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_
#define MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_

#include "motor_canmsgs/msg/detail/motor_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace motor_canmsgs
{

namespace msg
{

namespace builder
{

class Init_MotorMsg_data
{
public:
  explicit Init_MotorMsg_data(::motor_canmsgs::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  ::motor_canmsgs::msg::MotorMsg data(::motor_canmsgs::msg::MotorMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_canmsgs::msg::MotorMsg msg_;
};

class Init_MotorMsg_motor_id
{
public:
  explicit Init_MotorMsg_motor_id(::motor_canmsgs::msg::MotorMsg & msg)
  : msg_(msg)
  {}
  Init_MotorMsg_data motor_id(::motor_canmsgs::msg::MotorMsg::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorMsg_data(msg_);
  }

private:
  ::motor_canmsgs::msg::MotorMsg msg_;
};

class Init_MotorMsg_can_id
{
public:
  Init_MotorMsg_can_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorMsg_motor_id can_id(::motor_canmsgs::msg::MotorMsg::_can_id_type arg)
  {
    msg_.can_id = std::move(arg);
    return Init_MotorMsg_motor_id(msg_);
  }

private:
  ::motor_canmsgs::msg::MotorMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_canmsgs::msg::MotorMsg>()
{
  return motor_canmsgs::msg::builder::Init_MotorMsg_can_id();
}

}  // namespace motor_canmsgs

#endif  // MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__BUILDER_HPP_
