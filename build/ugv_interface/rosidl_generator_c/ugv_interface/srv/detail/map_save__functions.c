// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ugv_interface:srv/MapSave.idl
// generated code does not contain a copyright notice
#include "ugv_interface/srv/detail/map_save__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mapname`
#include "rosidl_runtime_c/string_functions.h"

bool
ugv_interface__srv__MapSave_Request__init(ugv_interface__srv__MapSave_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mapname
  if (!rosidl_runtime_c__String__init(&msg->mapname)) {
    ugv_interface__srv__MapSave_Request__fini(msg);
    return false;
  }
  return true;
}

void
ugv_interface__srv__MapSave_Request__fini(ugv_interface__srv__MapSave_Request * msg)
{
  if (!msg) {
    return;
  }
  // mapname
  rosidl_runtime_c__String__fini(&msg->mapname);
}

bool
ugv_interface__srv__MapSave_Request__are_equal(const ugv_interface__srv__MapSave_Request * lhs, const ugv_interface__srv__MapSave_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mapname
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mapname), &(rhs->mapname)))
  {
    return false;
  }
  return true;
}

bool
ugv_interface__srv__MapSave_Request__copy(
  const ugv_interface__srv__MapSave_Request * input,
  ugv_interface__srv__MapSave_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mapname
  if (!rosidl_runtime_c__String__copy(
      &(input->mapname), &(output->mapname)))
  {
    return false;
  }
  return true;
}

ugv_interface__srv__MapSave_Request *
ugv_interface__srv__MapSave_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Request * msg = (ugv_interface__srv__MapSave_Request *)allocator.allocate(sizeof(ugv_interface__srv__MapSave_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ugv_interface__srv__MapSave_Request));
  bool success = ugv_interface__srv__MapSave_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ugv_interface__srv__MapSave_Request__destroy(ugv_interface__srv__MapSave_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ugv_interface__srv__MapSave_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ugv_interface__srv__MapSave_Request__Sequence__init(ugv_interface__srv__MapSave_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Request * data = NULL;

  if (size) {
    data = (ugv_interface__srv__MapSave_Request *)allocator.zero_allocate(size, sizeof(ugv_interface__srv__MapSave_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ugv_interface__srv__MapSave_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ugv_interface__srv__MapSave_Request__fini(&data[i - 1]);
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
ugv_interface__srv__MapSave_Request__Sequence__fini(ugv_interface__srv__MapSave_Request__Sequence * array)
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
      ugv_interface__srv__MapSave_Request__fini(&array->data[i]);
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

ugv_interface__srv__MapSave_Request__Sequence *
ugv_interface__srv__MapSave_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Request__Sequence * array = (ugv_interface__srv__MapSave_Request__Sequence *)allocator.allocate(sizeof(ugv_interface__srv__MapSave_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ugv_interface__srv__MapSave_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ugv_interface__srv__MapSave_Request__Sequence__destroy(ugv_interface__srv__MapSave_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ugv_interface__srv__MapSave_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ugv_interface__srv__MapSave_Request__Sequence__are_equal(const ugv_interface__srv__MapSave_Request__Sequence * lhs, const ugv_interface__srv__MapSave_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ugv_interface__srv__MapSave_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ugv_interface__srv__MapSave_Request__Sequence__copy(
  const ugv_interface__srv__MapSave_Request__Sequence * input,
  ugv_interface__srv__MapSave_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ugv_interface__srv__MapSave_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ugv_interface__srv__MapSave_Request * data =
      (ugv_interface__srv__MapSave_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ugv_interface__srv__MapSave_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ugv_interface__srv__MapSave_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ugv_interface__srv__MapSave_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `response`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
ugv_interface__srv__MapSave_Response__init(ugv_interface__srv__MapSave_Response * msg)
{
  if (!msg) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__init(&msg->response)) {
    ugv_interface__srv__MapSave_Response__fini(msg);
    return false;
  }
  return true;
}

void
ugv_interface__srv__MapSave_Response__fini(ugv_interface__srv__MapSave_Response * msg)
{
  if (!msg) {
    return;
  }
  // response
  rosidl_runtime_c__String__fini(&msg->response);
}

bool
ugv_interface__srv__MapSave_Response__are_equal(const ugv_interface__srv__MapSave_Response * lhs, const ugv_interface__srv__MapSave_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
ugv_interface__srv__MapSave_Response__copy(
  const ugv_interface__srv__MapSave_Response * input,
  ugv_interface__srv__MapSave_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // response
  if (!rosidl_runtime_c__String__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

ugv_interface__srv__MapSave_Response *
ugv_interface__srv__MapSave_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Response * msg = (ugv_interface__srv__MapSave_Response *)allocator.allocate(sizeof(ugv_interface__srv__MapSave_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ugv_interface__srv__MapSave_Response));
  bool success = ugv_interface__srv__MapSave_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ugv_interface__srv__MapSave_Response__destroy(ugv_interface__srv__MapSave_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ugv_interface__srv__MapSave_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ugv_interface__srv__MapSave_Response__Sequence__init(ugv_interface__srv__MapSave_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Response * data = NULL;

  if (size) {
    data = (ugv_interface__srv__MapSave_Response *)allocator.zero_allocate(size, sizeof(ugv_interface__srv__MapSave_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ugv_interface__srv__MapSave_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ugv_interface__srv__MapSave_Response__fini(&data[i - 1]);
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
ugv_interface__srv__MapSave_Response__Sequence__fini(ugv_interface__srv__MapSave_Response__Sequence * array)
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
      ugv_interface__srv__MapSave_Response__fini(&array->data[i]);
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

ugv_interface__srv__MapSave_Response__Sequence *
ugv_interface__srv__MapSave_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ugv_interface__srv__MapSave_Response__Sequence * array = (ugv_interface__srv__MapSave_Response__Sequence *)allocator.allocate(sizeof(ugv_interface__srv__MapSave_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ugv_interface__srv__MapSave_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ugv_interface__srv__MapSave_Response__Sequence__destroy(ugv_interface__srv__MapSave_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ugv_interface__srv__MapSave_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ugv_interface__srv__MapSave_Response__Sequence__are_equal(const ugv_interface__srv__MapSave_Response__Sequence * lhs, const ugv_interface__srv__MapSave_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ugv_interface__srv__MapSave_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ugv_interface__srv__MapSave_Response__Sequence__copy(
  const ugv_interface__srv__MapSave_Response__Sequence * input,
  ugv_interface__srv__MapSave_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ugv_interface__srv__MapSave_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ugv_interface__srv__MapSave_Response * data =
      (ugv_interface__srv__MapSave_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ugv_interface__srv__MapSave_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ugv_interface__srv__MapSave_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ugv_interface__srv__MapSave_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
