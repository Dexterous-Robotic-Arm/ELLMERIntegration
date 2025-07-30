// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ufactory_ellmer_msgs:srv/SetInt16.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__STRUCT_H_
#define UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetInt16 in the package ufactory_ellmer_msgs.
typedef struct ufactory_ellmer_msgs__srv__SetInt16_Request
{
  /// e.g. 0 = READY, 4 = PAUSE …
  int16_t data;
} ufactory_ellmer_msgs__srv__SetInt16_Request;

// Struct for a sequence of ufactory_ellmer_msgs__srv__SetInt16_Request.
typedef struct ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence
{
  ufactory_ellmer_msgs__srv__SetInt16_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetInt16 in the package ufactory_ellmer_msgs.
typedef struct ufactory_ellmer_msgs__srv__SetInt16_Response
{
  /// 0 = success,  anything else = xArm error code
  int16_t ret;
  /// echoed or diagnostic text
  rosidl_runtime_c__String message;
} ufactory_ellmer_msgs__srv__SetInt16_Response;

// Struct for a sequence of ufactory_ellmer_msgs__srv__SetInt16_Response.
typedef struct ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence
{
  ufactory_ellmer_msgs__srv__SetInt16_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UFACTORY_ELLMER_MSGS__SRV__DETAIL__SET_INT16__STRUCT_H_
