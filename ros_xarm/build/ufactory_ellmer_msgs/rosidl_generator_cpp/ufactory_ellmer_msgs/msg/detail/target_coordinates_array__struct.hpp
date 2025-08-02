// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'targets'
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinatesArray __attribute__((deprecated))
#else
# define DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinatesArray __declspec(deprecated)
#endif

namespace ufactory_ellmer_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetCoordinatesArray_
{
  using Type = TargetCoordinatesArray_<ContainerAllocator>;

  explicit TargetCoordinatesArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TargetCoordinatesArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _targets_type =
    std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>>;
  _targets_type targets;

  // setters for named parameter idiom
  Type & set__targets(
    const std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ufactory_ellmer_msgs::msg::TargetCoordinates_<ContainerAllocator>>> & _arg)
  {
    this->targets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinatesArray
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ufactory_ellmer_msgs__msg__TargetCoordinatesArray
    std::shared_ptr<ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetCoordinatesArray_ & other) const
  {
    if (this->targets != other.targets) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetCoordinatesArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetCoordinatesArray_

// alias to use template instance with default allocator
using TargetCoordinatesArray =
  ufactory_ellmer_msgs::msg::TargetCoordinatesArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ufactory_ellmer_msgs

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_HPP_
