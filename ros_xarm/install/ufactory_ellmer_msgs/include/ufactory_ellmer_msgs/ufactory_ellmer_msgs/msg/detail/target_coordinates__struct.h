// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice

#ifndef UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_H_
#define UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/TargetCoordinates in the package ufactory_ellmer_msgs.
/**
  * Pose of a single grasp/way‑point in the robot base frame
 */
typedef struct ufactory_ellmer_msgs__msg__TargetCoordinates
{
  geometry_msgs__msg__Point position;
  float roll;
  float pitch;
  float yaw;
  float confidence;
} ufactory_ellmer_msgs__msg__TargetCoordinates;

// Struct for a sequence of ufactory_ellmer_msgs__msg__TargetCoordinates.
typedef struct ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence
{
  ufactory_ellmer_msgs__msg__TargetCoordinates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UFACTORY_ELLMER_MSGS__MSG__DETAIL__TARGET_COORDINATES__STRUCT_H_
