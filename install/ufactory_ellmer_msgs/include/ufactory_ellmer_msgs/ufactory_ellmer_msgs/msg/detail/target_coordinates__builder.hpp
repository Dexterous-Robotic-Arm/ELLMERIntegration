// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__BUILDER_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ufactory_ellmer_msgs
{

namespace msg
{

namespace builder
{

class Init_TargetCoordinates_confidence
{
public:
  explicit Init_TargetCoordinates_confidence(::ufactory_ellmer_msgs::msg::TargetCoordinates & msg)
  : msg_(msg)
  {}
  ::ufactory_ellmer_msgs::msg::TargetCoordinates confidence(::ufactory_ellmer_msgs::msg::TargetCoordinates::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinates msg_;
};

class Init_TargetCoordinates_yaw
{
public:
  explicit Init_TargetCoordinates_yaw(::ufactory_ellmer_msgs::msg::TargetCoordinates & msg)
  : msg_(msg)
  {}
  Init_TargetCoordinates_confidence yaw(::ufactory_ellmer_msgs::msg::TargetCoordinates::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TargetCoordinates_confidence(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinates msg_;
};

class Init_TargetCoordinates_pitch
{
public:
  explicit Init_TargetCoordinates_pitch(::ufactory_ellmer_msgs::msg::TargetCoordinates & msg)
  : msg_(msg)
  {}
  Init_TargetCoordinates_yaw pitch(::ufactory_ellmer_msgs::msg::TargetCoordinates::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_TargetCoordinates_yaw(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinates msg_;
};

class Init_TargetCoordinates_roll
{
public:
  explicit Init_TargetCoordinates_roll(::ufactory_ellmer_msgs::msg::TargetCoordinates & msg)
  : msg_(msg)
  {}
  Init_TargetCoordinates_pitch roll(::ufactory_ellmer_msgs::msg::TargetCoordinates::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_TargetCoordinates_pitch(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinates msg_;
};

class Init_TargetCoordinates_position
{
public:
  Init_TargetCoordinates_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetCoordinates_roll position(::ufactory_ellmer_msgs::msg::TargetCoordinates::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TargetCoordinates_roll(msg_);
  }

private:
  ::ufactory_ellmer_msgs::msg::TargetCoordinates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ufactory_ellmer_msgs::msg::TargetCoordinates>()
{
  return ufactory_ellmer_msgs::msg::builder::Init_TargetCoordinates_position();
}

}  // namespace ufactory_ellmer_msgs

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__BUILDER_HPP_
