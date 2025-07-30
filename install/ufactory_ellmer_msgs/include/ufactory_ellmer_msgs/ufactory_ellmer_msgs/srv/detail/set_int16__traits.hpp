// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ufactory_ellmer_msgs:srv/SetInt16.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__TRAITS_HPP_
#define UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ufactory_ellmer_msgs/srv/detail/set_int16__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ufactory_ellmer_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetInt16_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetInt16_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetInt16_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ufactory_ellmer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ufactory_ellmer_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ufactory_ellmer_msgs::srv::SetInt16_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ufactory_ellmer_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ufactory_ellmer_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ufactory_ellmer_msgs::srv::SetInt16_Request & msg)
{
  return ufactory_ellmer_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ufactory_ellmer_msgs::srv::SetInt16_Request>()
{
  return "ufactory_ellmer_msgs::srv::SetInt16_Request";
}

template<>
inline const char * name<ufactory_ellmer_msgs::srv::SetInt16_Request>()
{
  return "ufactory_ellmer_msgs/srv/SetInt16_Request";
}

template<>
struct has_fixed_size<ufactory_ellmer_msgs::srv::SetInt16_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ufactory_ellmer_msgs::srv::SetInt16_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ufactory_ellmer_msgs::srv::SetInt16_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ufactory_ellmer_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetInt16_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ret
  {
    out << "ret: ";
    rosidl_generator_traits::value_to_yaml(msg.ret, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetInt16_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ret
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ret: ";
    rosidl_generator_traits::value_to_yaml(msg.ret, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetInt16_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ufactory_ellmer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ufactory_ellmer_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ufactory_ellmer_msgs::srv::SetInt16_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ufactory_ellmer_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ufactory_ellmer_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const ufactory_ellmer_msgs::srv::SetInt16_Response & msg)
{
  return ufactory_ellmer_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ufactory_ellmer_msgs::srv::SetInt16_Response>()
{
  return "ufactory_ellmer_msgs::srv::SetInt16_Response";
}

template<>
inline const char * name<ufactory_ellmer_msgs::srv::SetInt16_Response>()
{
  return "ufactory_ellmer_msgs/srv/SetInt16_Response";
}

template<>
struct has_fixed_size<ufactory_ellmer_msgs::srv::SetInt16_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ufactory_ellmer_msgs::srv::SetInt16_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ufactory_ellmer_msgs::srv::SetInt16_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ufactory_ellmer_msgs::srv::SetInt16>()
{
  return "ufactory_ellmer_msgs::srv::SetInt16";
}

template<>
inline const char * name<ufactory_ellmer_msgs::srv::SetInt16>()
{
  return "ufactory_ellmer_msgs/srv/SetInt16";
}

template<>
struct has_fixed_size<ufactory_ellmer_msgs::srv::SetInt16>
  : std::integral_constant<
    bool,
    has_fixed_size<ufactory_ellmer_msgs::srv::SetInt16_Request>::value &&
    has_fixed_size<ufactory_ellmer_msgs::srv::SetInt16_Response>::value
  >
{
};

template<>
struct has_bounded_size<ufactory_ellmer_msgs::srv::SetInt16>
  : std::integral_constant<
    bool,
    has_bounded_size<ufactory_ellmer_msgs::srv::SetInt16_Request>::value &&
    has_bounded_size<ufactory_ellmer_msgs::srv::SetInt16_Response>::value
  >
{
};

template<>
struct is_service<ufactory_ellmer_msgs::srv::SetInt16>
  : std::true_type
{
};

template<>
struct is_service_request<ufactory_ellmer_msgs::srv::SetInt16_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ufactory_ellmer_msgs::srv::SetInt16_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__TRAITS_HPP_
