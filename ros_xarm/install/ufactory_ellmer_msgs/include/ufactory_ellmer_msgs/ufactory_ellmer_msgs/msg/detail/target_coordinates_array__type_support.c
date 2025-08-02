// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__rosidl_typesupport_introspection_c.h"
#include "ufactory_ellmer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__functions.h"
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__struct.h"


// Include directives for member types
// Member `targets`
#include "ufactory_ellmer_msgs/msg/target_coordinates.h"
// Member `targets`
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__init(message_memory);
}

void ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_fini_function(void * message_memory)
{
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(message_memory);
}

size_t ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__size_function__TargetCoordinatesArray__targets(
  const void * untyped_member)
{
  const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * member =
    (const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *)(untyped_member);
  return member->size;
}

const void * ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_const_function__TargetCoordinatesArray__targets(
  const void * untyped_member, size_t index)
{
  const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * member =
    (const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_function__TargetCoordinatesArray__targets(
  void * untyped_member, size_t index)
{
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * member =
    (ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *)(untyped_member);
  return &member->data[index];
}

void ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__fetch_function__TargetCoordinatesArray__targets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ufactory_ellmer_msgs__msg__TargetCoordinates * item =
    ((const ufactory_ellmer_msgs__msg__TargetCoordinates *)
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_const_function__TargetCoordinatesArray__targets(untyped_member, index));
  ufactory_ellmer_msgs__msg__TargetCoordinates * value =
    (ufactory_ellmer_msgs__msg__TargetCoordinates *)(untyped_value);
  *value = *item;
}

void ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__assign_function__TargetCoordinatesArray__targets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ufactory_ellmer_msgs__msg__TargetCoordinates * item =
    ((ufactory_ellmer_msgs__msg__TargetCoordinates *)
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_function__TargetCoordinatesArray__targets(untyped_member, index));
  const ufactory_ellmer_msgs__msg__TargetCoordinates * value =
    (const ufactory_ellmer_msgs__msg__TargetCoordinates *)(untyped_value);
  *item = *value;
}

bool ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__resize_function__TargetCoordinatesArray__targets(
  void * untyped_member, size_t size)
{
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * member =
    (ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *)(untyped_member);
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__fini(member);
  return ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_member_array[1] = {
  {
    "targets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray, targets),  // bytes offset in struct
    NULL,  // default value
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__size_function__TargetCoordinatesArray__targets,  // size() function pointer
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_const_function__TargetCoordinatesArray__targets,  // get_const(index) function pointer
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__get_function__TargetCoordinatesArray__targets,  // get(index) function pointer
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__fetch_function__TargetCoordinatesArray__targets,  // fetch(index, &value) function pointer
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__assign_function__TargetCoordinatesArray__targets,  // assign(index, value) function pointer
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__resize_function__TargetCoordinatesArray__targets  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_members = {
  "ufactory_ellmer_msgs__msg",  // message namespace
  "TargetCoordinatesArray",  // message name
  1,  // number of fields
  sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray),
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_member_array,  // message members
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_init_function,  // function to initialize message memory (memory has to be allocated)
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_type_support_handle = {
  0,
  &ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ufactory_ellmer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ufactory_ellmer_msgs, msg, TargetCoordinatesArray)() {
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ufactory_ellmer_msgs, msg, TargetCoordinates)();
  if (!ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_type_support_handle.typesupport_identifier) {
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ufactory_ellmer_msgs__msg__TargetCoordinatesArray__rosidl_typesupport_introspection_c__TargetCoordinatesArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
