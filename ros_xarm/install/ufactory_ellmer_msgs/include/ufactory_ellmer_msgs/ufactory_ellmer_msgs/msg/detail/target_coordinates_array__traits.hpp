// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__TRAITS_HPP_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'targets'
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__traits.hpp"

namespace ufactory_ellmer_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetCoordinatesArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: targets
  {
    if (msg.targets.size() == 0) {
      out << "targets: []";
    } else {
      out << "targets: [";
      size_t pending_items = msg.targets.size();
      for (auto item : msg.targets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetCoordinatesArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: targets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.targets.size() == 0) {
      out << "targets: []\n";
    } else {
      out << "targets:\n";
      for (auto item : msg.targets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetCoordinatesArray & msg, bool use_flow_style = false)
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
  const ufactory_ellmer_msgs::msg::TargetCoordinatesArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  ufactory_ellmer_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ufactory_ellmer_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ufactory_ellmer_msgs::msg::TargetCoordinatesArray & msg)
{
  return ufactory_ellmer_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>()
{
  return "ufactory_ellmer_msgs::msg::TargetCoordinatesArray";
}

template<>
inline const char * name<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>()
{
  return "ufactory_ellmer_msgs/msg/TargetCoordinatesArray";
}

template<>
struct has_fixed_size<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__TRAITS_HPP_
