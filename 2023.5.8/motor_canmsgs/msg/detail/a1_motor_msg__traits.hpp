// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_canmsgs:msg/A1MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__TRAITS_HPP_
#define MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__TRAITS_HPP_

#include "motor_canmsgs/msg/detail/a1_motor_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<motor_canmsgs::msg::A1MotorMsg>()
{
  return "motor_canmsgs::msg::A1MotorMsg";
}

template<>
inline const char * name<motor_canmsgs::msg::A1MotorMsg>()
{
  return "motor_canmsgs/msg/A1MotorMsg";
}

template<>
struct has_fixed_size<motor_canmsgs::msg::A1MotorMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<motor_canmsgs::msg::A1MotorMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<motor_canmsgs::msg::A1MotorMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__TRAITS_HPP_
