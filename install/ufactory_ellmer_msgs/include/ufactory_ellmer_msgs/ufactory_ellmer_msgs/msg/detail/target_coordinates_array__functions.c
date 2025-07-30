// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ufactory_ellmer_msgs:msg/TargetCoordinatesArray.idl
// generated code does not contain a copyright notice
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `targets`
#include "ufactory_ellmer_msgs/msg/detail/target_coordinates__functions.h"

bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__init(ufactory_ellmer_msgs__msg__TargetCoordinatesArray * msg)
{
  if (!msg) {
    return false;
  }
  // targets
  if (!ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__init(&msg->targets, 0)) {
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(msg);
    return false;
  }
  return true;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(ufactory_ellmer_msgs__msg__TargetCoordinatesArray * msg)
{
  if (!msg) {
    return;
  }
  // targets
  ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__fini(&msg->targets);
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__are_equal(const ufactory_ellmer_msgs__msg__TargetCoordinatesArray * lhs, const ufactory_ellmer_msgs__msg__TargetCoordinatesArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // targets
  if (!ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__are_equal(
      &(lhs->targets), &(rhs->targets)))
  {
    return false;
  }
  return true;
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__copy(
  const ufactory_ellmer_msgs__msg__TargetCoordinatesArray * input,
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray * output)
{
  if (!input || !output) {
    return false;
  }
  // targets
  if (!ufactory_ellmer_msgs__msg__TargetCoordinates__Sequence__copy(
      &(input->targets), &(output->targets)))
  {
    return false;
  }
  return true;
}

ufactory_ellmer_msgs__msg__TargetCoordinatesArray *
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray * msg = (ufactory_ellmer_msgs__msg__TargetCoordinatesArray *)allocator.allocate(sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray));
  bool success = ufactory_ellmer_msgs__msg__TargetCoordinatesArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__destroy(ufactory_ellmer_msgs__msg__TargetCoordinatesArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__init(ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray * data = NULL;

  if (size) {
    data = (ufactory_ellmer_msgs__msg__TargetCoordinatesArray *)allocator.zero_allocate(size, sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ufactory_ellmer_msgs__msg__TargetCoordinatesArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(&data[i - 1]);
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
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__fini(ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * array)
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
      ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(&array->data[i]);
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

ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence *
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * array = (ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence *)allocator.allocate(sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__destroy(ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__are_equal(const ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * lhs, const ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ufactory_ellmer_msgs__msg__TargetCoordinatesArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence__copy(
  const ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * input,
  ufactory_ellmer_msgs__msg__TargetCoordinatesArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ufactory_ellmer_msgs__msg__TargetCoordinatesArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ufactory_ellmer_msgs__msg__TargetCoordinatesArray * data =
      (ufactory_ellmer_msgs__msg__TargetCoordinatesArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ufactory_ellmer_msgs__msg__TargetCoordinatesArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ufactory_ellmer_msgs__msg__TargetCoordinatesArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ufactory_ellmer_msgs__msg__TargetCoordinatesArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
