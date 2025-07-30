// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinates __attribute__((deprecated))
#else
# define DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinates __declspec(deprecated)
#endif

namespace ufactory_ellmer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetCoordinates_
{
  using Type = TargetCoordinates_<ContainerAllocator>;

  explicit TargetCoordinates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->confidence = 0.0f;
    }
  }

  explicit TargetCoordinates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->roll = 0.0f;
      this->pitch = 0.0f;
      this->yaw = 0.0f;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _roll_type =
    float;
  _roll_type roll;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__roll(
    const float & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> *;
  using ConstRawPtr =
    const ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinates
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinates
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCoordinates_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCoordinates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCoordinates_

// alias to use template instance with default allocator
using TargetCoordinates =
  ufactory_ellmer_msgs::msg::TargetCoordinates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ufactory_ellmer_msgs

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_HPP_
