// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_canmsgs:msg/MotorMsg.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_
#define MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__motor_canmsgs__msg__MotorMsg __attribute__((deprecated))
#else
# define DEPRECATED__motor_canmsgs__msg__MotorMsg __declspec(deprecated)
#endif

namespace motor_canmsgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorMsg_
{
  using Type = MotorMsg_<ContainerAllocator>;

  explicit MotorMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0ul;
      this->motor_id = 0;
      this->data = 0.0;
    }
  }

  explicit MotorMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->can_id = 0ul;
      this->motor_id = 0;
      this->data = 0.0;
    }
  }

  // field types and members
  using _can_id_type =
    uint32_t;
  _can_id_type can_id;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _data_type =
    double;
  _data_type data;

  // setters for named parameter idiom
  Type & set__can_id(
    const uint32_t & _arg)
  {
    this->can_id = _arg;
    return *this;
  }
  Type & set__motor_id(
    const uint8_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__data(
    const double & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_canmsgs::msg::MotorMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_canmsgs::msg::MotorMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_canmsgs::msg::MotorMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_canmsgs::msg::MotorMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_canmsgs__msg__MotorMsg
    std::shared_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_canmsgs__msg__MotorMsg
    std::shared_ptr<motor_canmsgs::msg::MotorMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorMsg_ & other) const
  {
    if (this->can_id != other.can_id) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorMsg_

// alias to use template instance with default allocator
using MotorMsg =
  motor_canmsgs::msg::MotorMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_canmsgs

#endif  // MOTOR_CANMSGS__MSG__DETAIL__MOTOR_MSG__STRUCT_HPP_
