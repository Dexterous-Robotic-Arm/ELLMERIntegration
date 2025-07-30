// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ufactory_ellmer_msgs:srv/SetInt16.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ufactory_ellmer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ufactory_ellmer_msgs/srv/detail/set_int16__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ufactory_ellmer_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
cdr_serialize(
  const ufactory_ellmer_msgs::srv::SetInt16_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ufactory_ellmer_msgs::srv::SetInt16_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
get_serialized_size(
  const ufactory_ellmer_msgs::srv::SetInt16_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
max_serialized_size_SetInt16_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ufactory_ellmer_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ufactory_ellmer_msgs, srv, SetInt16_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ufactory_ellmer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "ufactory_ellmer_msgs/srv/detail/set_int16__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace ufactory_ellmer_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
cdr_serialize(
  const ufactory_ellmer_msgs::srv::SetInt16_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ufactory_ellmer_msgs::srv::SetInt16_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
get_serialized_size(
  const ufactory_ellmer_msgs::srv::SetInt16_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
max_serialized_size_SetInt16_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ufactory_ellmer_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ufactory_ellmer_msgs, srv, SetInt16_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "ufactory_ellmer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ufactory_ellmer_msgs
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ufactory_ellmer_msgs, srv, SetInt16)();

#ifdef __cplusplus
}
#endif

#endif  // UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
