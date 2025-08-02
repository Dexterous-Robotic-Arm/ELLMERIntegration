// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_H_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'targets'
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__struct.h"

/// Struct defined in msg/TargetCoordinatesArray in the package ufactory_ellmer_msgs.
typedef struct ufactory_ellmer_msgs__msg__TargetCoordinatesArray
{
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence targets;
} ufactory_ellmer_msgs__msg__TargetCoordinatesArray;

// Struct for a sequence of ufactory_ellmer_msgs__msg__TargetCoordinatesArray.
typedef struct ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence
{
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES_ARRAY__STRUCT_H_
