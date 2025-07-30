// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__TRAITS_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace ufactory_ellmer_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetCoordinates & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetCoordinates & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetCoordinates & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ufactory_ellmer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ufactory_ellmer_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ufactory_ellmer_msgs::msg::TargetCoordinates & msg,
  std::ostream & out, size_t indentation = 0)
{
  ufactory_ellmer_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ufactory_ellmer_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ufactory_ellmer_msgs::msg::TargetCoordinates & msg)
{
  return ufactory_ellmer_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ufactory_ellmer_msgs::msg::TargetCoordinates>()
{
  return "ufactory_ellmer_msgs::msg::TargetCoordinates";
}

template<>
inline const char * name<ufactory_ellmer_msgs::msg::TargetCoordinates>()
{
  return "ufactory_ellmer_msgs/msg/TargetCoordinates";
}

template<>
struct has_fixed_size<ufactory_ellmer_msgs::msg::TargetCoordinates>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<ufactory_ellmer_msgs::msg::TargetCoordinates>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<ufactory_ellmer_msgs::msg::TargetCoordinates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__TRAITS_HPP_
