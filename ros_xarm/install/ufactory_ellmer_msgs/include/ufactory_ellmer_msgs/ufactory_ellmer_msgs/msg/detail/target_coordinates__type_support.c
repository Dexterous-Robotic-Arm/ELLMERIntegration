// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__rosidl_typesupport_introspection_c.h"
#include "ufactory_ellmer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__functions.h"
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__struct.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ufactory_ellmer_msgs__msg__TargetCoordinates__init(message_memory);
}

void ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_fini_function(void * message_memory)
{
  ufactory_ellmer_msgs__msg__TargetCoordinates__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_member_array[5] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinates, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinates, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinates, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinates, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinates, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_members = {
  "ufactory_ellmer_msgs__msg",  // message namespace
  "TargetCoordinates",  // message name
  5,  // number of fields
  sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates),
  ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_member_array,  // message members
  ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_init_function,  // function to initialize message memory (memory has to be allocated)
  ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_type_support_handle = {
  0,
  &ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ufactory_ellmer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ufactory_ellmer_msgs, msg, TargetCoordinates)() {
  ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_type_support_handle.typesupport_identifier) {
    ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ufactory_ellmer_msgs__msg__TargetCoordinates__rosidl_typesupport_introspection_c__TargetCoordinates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
