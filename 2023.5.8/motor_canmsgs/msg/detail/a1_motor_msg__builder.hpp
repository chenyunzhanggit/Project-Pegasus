// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_canmsgs:msg/A1MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__BUILDER_HPP_
#define MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__BUILDER_HPP_

#include "motor_canmsgs/msg/detail/a1_motor_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace motor_canmsgs
{

namespace msg
{

namespace builder
{

class Init_A1MotorMsg_k_w
{
public:
  explicit Init_A1MotorMsg_k_w(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  ::motor_canmsgs::msg::A1MotorMsg k_w(::motor_canmsgs::msg::A1MotorMsg::_k_w_type arg)
  {
    msg_.k_w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_k_p
{
public:
  explicit Init_A1MotorMsg_k_p(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  Init_A1MotorMsg_k_w k_p(::motor_canmsgs::msg::A1MotorMsg::_k_p_type arg)
  {
    msg_.k_p = std::move(arg);
    return Init_A1MotorMsg_k_w(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_pos
{
public:
  explicit Init_A1MotorMsg_pos(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  Init_A1MotorMsg_k_p pos(::motor_canmsgs::msg::A1MotorMsg::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_A1MotorMsg_k_p(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_w
{
public:
  explicit Init_A1MotorMsg_w(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  Init_A1MotorMsg_pos w(::motor_canmsgs::msg::A1MotorMsg::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_A1MotorMsg_pos(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_t
{
public:
  explicit Init_A1MotorMsg_t(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  Init_A1MotorMsg_w t(::motor_canmsgs::msg::A1MotorMsg::_t_type arg)
  {
    msg_.t = std::move(arg);
    return Init_A1MotorMsg_w(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_motor_id
{
public:
  explicit Init_A1MotorMsg_motor_id(::motor_canmsgs::msg::A1MotorMsg & msg)
  : msg_(msg)
  {}
  Init_A1MotorMsg_t motor_id(::motor_canmsgs::msg::A1MotorMsg::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_A1MotorMsg_t(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

class Init_A1MotorMsg_leg_id
{
public:
  Init_A1MotorMsg_leg_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_A1MotorMsg_motor_id leg_id(::motor_canmsgs::msg::A1MotorMsg::_leg_id_type arg)
  {
    msg_.leg_id = std::move(arg);
    return Init_A1MotorMsg_motor_id(msg_);
  }

private:
  ::motor_canmsgs::msg::A1MotorMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_canmsgs::msg::A1MotorMsg>()
{
  return motor_canmsgs::msg::builder::Init_A1MotorMsg_leg_id();
}

}  // namespace motor_canmsgs

#endif  // MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__BUILDER_HPP_
