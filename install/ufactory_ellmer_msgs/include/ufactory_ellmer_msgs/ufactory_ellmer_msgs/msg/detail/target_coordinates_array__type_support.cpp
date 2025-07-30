// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ufactory_ellmer_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TargetCoordinatesArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ufactory_ellmer_msgs::msg::TargetCoordinatesArray(_init);
}

void TargetCoordinatesArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ufactory_ellmer_msgs::msg::TargetCoordinatesArray *>(message_memory);
  typed_message->~TargetCoordinatesArray();
}

size_t size_function__TargetCoordinatesArray__targets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TargetCoordinatesArray__targets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetCoordinatesArray__targets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetCoordinatesArray__targets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const ufactory_ellmer_msgs::msg::TargetCoordinates *>(
    get_const_function__TargetCoordinatesArray__targets(untyped_member, index));
  auto & value = *reinterpret_cast<ufactory_ellmer_msgs::msg::TargetCoordinates *>(untyped_value);
  value = item;
}

void assign_function__TargetCoordinatesArray__targets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<ufactory_ellmer_msgs::msg::TargetCoordinates *>(
    get_function__TargetCoordinatesArray__targets(untyped_member, index));
  const auto & value = *reinterpret_cast<const ufactory_ellmer_msgs::msg::TargetCoordinates *>(untyped_value);
  item = value;
}

void resize_function__TargetCoordinatesArray__targets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<ufactory_ellmer_msgs::msg::TargetCoordinates> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TargetCoordinatesArray_message_member_array[1] = {
  {
    "targets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<ufactory_ellmer_msgs::msg::TargetCoordinates>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs::msg::TargetCoordinatesArray, targets),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetCoordinatesArray__targets,  // size() function pointer
    get_const_function__TargetCoordinatesArray__targets,  // get_const(index) function pointer
    get_function__TargetCoordinatesArray__targets,  // get(index) function pointer
    fetch_function__TargetCoordinatesArray__targets,  // fetch(index, &value) function pointer
    assign_function__TargetCoordinatesArray__targets,  // assign(index, value) function pointer
    resize_function__TargetCoordinatesArray__targets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TargetCoordinatesArray_message_members = {
  "ufactory_ellmer_msgs::msg",  // message namespace
  "TargetCoordinatesArray",  // message name
  1,  // number of fields
  sizeof(ufactory_ellmer_msgs::msg::TargetCoordinatesArray),
  TargetCoordinatesArray_message_member_array,  // message members
  TargetCoordinatesArray_init_function,  // function to initialize message memory (memory has to be allocated)
  TargetCoordinatesArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TargetCoordinatesArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TargetCoordinatesArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ufactory_ellmer_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ufactory_ellmer_msgs::msg::TargetCoordinatesArray>()
{
  return &::ufactory_ellmer_msgs::msg::rosidl_typesupport_introspection_cpp::TargetCoordinatesArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ufactory_ellmer_msgs, msg, TargetCoordinatesArray)() {
  return &::ufactory_ellmer_msgs::msg::rosidl_typesupport_introspection_cpp::TargetCoordinatesArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
