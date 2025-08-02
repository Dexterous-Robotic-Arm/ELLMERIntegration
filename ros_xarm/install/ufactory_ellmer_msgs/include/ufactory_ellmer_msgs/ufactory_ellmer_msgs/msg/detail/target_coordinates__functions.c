// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinates.idl
// generated code does not contain a copyright notice
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
ufactory_ellmer_msgs__msg__TargetCoordinates__init(ufactory_ellmer_msgs__msg__TargetCoordinates * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    ufactory_ellmer_msgs__msg__TargetCoordinates__fini(msg);
    return false;
  }
  // roll
  // pitch
  // yaw
  // confidence
  return true;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinates__fini(ufactory_ellmer_msgs__msg__TargetCoordinates * msg)
{
  if (!msg) {
    return;
  }
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // roll
  // pitch
  // yaw
  // confidence
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinates__are_equal(const ufactory_ellmer_msgs__msg__TargetCoordinates * lhs, const ufactory_ellmer_msgs__msg__TargetCoordinates * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinates__copy(
  const ufactory_ellmer_msgs__msg__TargetCoordinates * input,
  ufactory_ellmer_msgs__msg__TargetCoordinates * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // confidence
  output->confidence = input->confidence;
  return true;
}

ufactory_ellmer_msgs__msg__TargetCoordinates *
ufactory_ellmer_msgs__msg__TargetCoordinates__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinates * msg = (ufactory_ellmer_msgs__msg__TargetCoordinates *)allocator.allocate(sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates));
  bool success = ufactory_ellmer_msgs__msg__TargetCoordinates__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinates__destroy(ufactory_ellmer_msgs__msg__TargetCoordinates * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ufactory_ellmer_msgs__msg__TargetCoordinates__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__init(ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinates * data = NULL;

  if (size) {
    data = (ufactory_ellmer_msgs__msg__TargetCoordinates *)allocator.zero_allocate(size, sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ufactory_ellmer_msgs__msg__TargetCoordinates__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ufactory_ellmer_msgs__msg__TargetCoordinates__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__fini(ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ufactory_ellmer_msgs__msg__TargetCoordinates__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * array = (ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence *)allocator.allocate(sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__destroy(ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__are_equal(const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * lhs, const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ufactory_ellmer_msgs__msg__TargetCoordinates__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__copy(
  const ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * input,
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ufactory_ellmer_msgs__msg__TargetCoordinates);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ufactory_ellmer_msgs__msg__TargetCoordinates * data =
      (ufactory_ellmer_msgs__msg__TargetCoordinates *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ufactory_ellmer_msgs__msg__TargetCoordinates__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ufactory_ellmer_msgs__msg__TargetCoordinates__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ufactory_ellmer_msgs__msg__TargetCoordinates__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
