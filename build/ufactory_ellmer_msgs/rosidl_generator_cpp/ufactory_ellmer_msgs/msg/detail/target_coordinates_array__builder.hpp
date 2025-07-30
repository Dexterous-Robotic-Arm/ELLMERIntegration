// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__BUILDER_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ufactory_ellmer_msgs
{

namespace msg
{

namespace builder
{

class Init_TargetCoordinatesArray_targets
{
public:
  Init_TargetCoordinatesArray_targets()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ufactory_ellmer_msgs::msg::TargetCoordinatesArray targets(::ufactory_ellmer_msgs::msg::TargetCoordinatesArray::_targets_type arg)
  {
    msg_.targets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinatesArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ufactory_ellmer_msgs::msg::TargetCoordinatesArray>()
{
  return ufactory_ellmer_msgs::msg::builder::Init_TargetCoordinatesArray_targets();
}

}  // namespace ufactory_ellmer_msgs

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__BUILDER_HPP_
