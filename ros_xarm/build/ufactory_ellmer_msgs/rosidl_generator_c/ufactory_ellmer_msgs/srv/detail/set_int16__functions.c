// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ufactory_ellmer_msgs:srv/SetInt16.idl
// generated code does not contain a copyright notice
#include "ufactory_ellmer_msgs/srv/detail/set_int16__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ufactory_ellmer_msgs__srv__SetInt16_Request__init(ufactory_ellmer_msgs__srv__SetInt16_Request * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Request__fini(ufactory_ellmer_msgs__srv__SetInt16_Request * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Request__are_equal(const ufactory_ellmer_msgs__srv__SetInt16_Request * lhs, const ufactory_ellmer_msgs__srv__SetInt16_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (lhs->data != rhs->data) {
    return false;
  }
  return true;
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Request__copy(
  const ufactory_ellmer_msgs__srv__SetInt16_Request * input,
  ufactory_ellmer_msgs__srv__SetInt16_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  output->data = input->data;
  return true;
}

ufactory_ellmer_msgs__srv__SetInt16_Request *
ufactory_ellmer_msgs__srv__SetInt16_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Request * msg = (ufactory_ellmer_msgs__srv__SetInt16_Request *)allocator.allocate(sizeof(ufactory_ellmer_msgs__srv__SetInt16_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ufactory_ellmer_msgs__srv__SetInt16_Request));
  bool success = ufactory_ellmer_msgs__srv__SetInt16_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Request__destroy(ufactory_ellmer_msgs__srv__SetInt16_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ufactory_ellmer_msgs__srv__SetInt16_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__init(ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Request * data = NULL;

  if (size) {
    data = (ufactory_ellmer_msgs__srv__SetInt16_Request *)allocator.zero_allocate(size, sizeof(ufactory_ellmer_msgs__srv__SetInt16_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ufactory_ellmer_msgs__srv__SetInt16_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ufactory_ellmer_msgs__srv__SetInt16_Request__fini(&data[i - 1]);
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
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__fini(ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * array)
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
      ufactory_ellmer_msgs__srv__SetInt16_Request__fini(&array->data[i]);
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

ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence *
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * array = (ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence *)allocator.allocate(sizeof(ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__destroy(ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__are_equal(const ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * lhs, const ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ufactory_ellmer_msgs__srv__SetInt16_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence__copy(
  const ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * input,
  ufactory_ellmer_msgs__srv__SetInt16_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ufactory_ellmer_msgs__srv__SetInt16_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ufactory_ellmer_msgs__srv__SetInt16_Request * data =
      (ufactory_ellmer_msgs__srv__SetInt16_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ufactory_ellmer_msgs__srv__SetInt16_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ufactory_ellmer_msgs__srv__SetInt16_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ufactory_ellmer_msgs__srv__SetInt16_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
ufactory_ellmer_msgs__srv__SetInt16_Response__init(ufactory_ellmer_msgs__srv__SetInt16_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ret
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    ufactory_ellmer_msgs__srv__SetInt16_Response__fini(msg);
    return false;
  }
  return true;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Response__fini(ufactory_ellmer_msgs__srv__SetInt16_Response * msg)
{
  if (!msg) {
    return;
  }
  // ret
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Response__are_equal(const ufactory_ellmer_msgs__srv__SetInt16_Response * lhs, const ufactory_ellmer_msgs__srv__SetInt16_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ret
  if (lhs->ret != rhs->ret) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Response__copy(
  const ufactory_ellmer_msgs__srv__SetInt16_Response * input,
  ufactory_ellmer_msgs__srv__SetInt16_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // ret
  output->ret = input->ret;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

ufactory_ellmer_msgs__srv__SetInt16_Response *
ufactory_ellmer_msgs__srv__SetInt16_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Response * msg = (ufactory_ellmer_msgs__srv__SetInt16_Response *)allocator.allocate(sizeof(ufactory_ellmer_msgs__srv__SetInt16_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ufactory_ellmer_msgs__srv__SetInt16_Response));
  bool success = ufactory_ellmer_msgs__srv__SetInt16_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Response__destroy(ufactory_ellmer_msgs__srv__SetInt16_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ufactory_ellmer_msgs__srv__SetInt16_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__init(ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Response * data = NULL;

  if (size) {
    data = (ufactory_ellmer_msgs__srv__SetInt16_Response *)allocator.zero_allocate(size, sizeof(ufactory_ellmer_msgs__srv__SetInt16_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ufactory_ellmer_msgs__srv__SetInt16_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ufactory_ellmer_msgs__srv__SetInt16_Response__fini(&data[i - 1]);
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
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__fini(ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * array)
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
      ufactory_ellmer_msgs__srv__SetInt16_Response__fini(&array->data[i]);
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

ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence *
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * array = (ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence *)allocator.allocate(sizeof(ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__destroy(ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__are_equal(const ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * lhs, const ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ufactory_ellmer_msgs__srv__SetInt16_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence__copy(
  const ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * input,
  ufactory_ellmer_msgs__srv__SetInt16_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ufactory_ellmer_msgs__srv__SetInt16_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ufactory_ellmer_msgs__srv__SetInt16_Response * data =
      (ufactory_ellmer_msgs__srv__SetInt16_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ufactory_ellmer_msgs__srv__SetInt16_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ufactory_ellmer_msgs__srv__SetInt16_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ufactory_ellmer_msgs__srv__SetInt16_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
