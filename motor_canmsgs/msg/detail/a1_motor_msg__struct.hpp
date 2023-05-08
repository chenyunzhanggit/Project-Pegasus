// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_canmsgs:msg/A1MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__STRUCT_HPP_
#define MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__motor_canmsgs__msg__A1MotorMsg __attribute__((deprecated))
#else
# define DEPRECATED__motor_canmsgs__msg__A1MotorMsg __declspec(deprecated)
#endif

namespace motor_canmsgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct A1MotorMsg_
{
  using Type = A1MotorMsg_<ContainerAllocator>;

  explicit A1MotorMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leg_id = 0;
      this->motor_id = 0;
      this->t = 0.0f;
      this->w = 0.0f;
      this->pos = 0.0f;
      this->k_p = 0.0f;
      this->k_w = 0.0f;
    }
  }

  explicit A1MotorMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->leg_id = 0;
      this->motor_id = 0;
      this->t = 0.0f;
      this->w = 0.0f;
      this->pos = 0.0f;
      this->k_p = 0.0f;
      this->k_w = 0.0f;
    }
  }

  // field types and members
  using _leg_id_type =
    uint8_t;
  _leg_id_type leg_id;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _t_type =
    float;
  _t_type t;
  using _w_type =
    float;
  _w_type w;
  using _pos_type =
    float;
  _pos_type pos;
  using _k_p_type =
    float;
  _k_p_type k_p;
  using _k_w_type =
    float;
  _k_w_type k_w;

  // setters for named parameter idiom
  Type & set__leg_id(
    const uint8_t & _arg)
  {
    this->leg_id = _arg;
    return *this;
  }
  Type & set__motor_id(
    const uint8_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__t(
    const float & _arg)
  {
    this->t = _arg;
    return *this;
  }
  Type & set__w(
    const float & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__pos(
    const float & _arg)
  {
    this->pos = _arg;
    return *this;
  }
  Type & set__k_p(
    const float & _arg)
  {
    this->k_p = _arg;
    return *this;
  }
  Type & set__k_w(
    const float & _arg)
  {
    this->k_w = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_canmsgs__msg__A1MotorMsg
    std::shared_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_canmsgs__msg__A1MotorMsg
    std::shared_ptr<motor_canmsgs::msg::A1MotorMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const A1MotorMsg_ & other) const
  {
    if (this->leg_id != other.leg_id) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->t != other.t) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->pos != other.pos) {
      return false;
    }
    if (this->k_p != other.k_p) {
      return false;
    }
    if (this->k_w != other.k_w) {
      return false;
    }
    return true;
  }
  bool operator!=(const A1MotorMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct A1MotorMsg_

// alias to use template instance with default allocator
using A1MotorMsg =
  motor_canmsgs::msg::A1MotorMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_canmsgs

#endif  // MOTOR_CANMSGS__MSG__DETAIL__A1_MOTOR_MSG__STRUCT_HPP_
