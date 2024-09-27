// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robethichor_interfaces:srv/NegotiationService.idl
// generated code does not contain a copyright notice
#include "robethichor_interfaces/srv/detail/negotiation_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `tasks`
#include "rosidl_runtime_c/string_functions.h"

bool
robethichor_interfaces__srv__NegotiationService_Request__init(robethichor_interfaces__srv__NegotiationService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // tasks
  if (!rosidl_runtime_c__String__Sequence__init(&msg->tasks, 0)) {
    robethichor_interfaces__srv__NegotiationService_Request__fini(msg);
    return false;
  }
  return true;
}

void
robethichor_interfaces__srv__NegotiationService_Request__fini(robethichor_interfaces__srv__NegotiationService_Request * msg)
{
  if (!msg) {
    return;
  }
  // tasks
  rosidl_runtime_c__String__Sequence__fini(&msg->tasks);
}

bool
robethichor_interfaces__srv__NegotiationService_Request__are_equal(const robethichor_interfaces__srv__NegotiationService_Request * lhs, const robethichor_interfaces__srv__NegotiationService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tasks
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->tasks), &(rhs->tasks)))
  {
    return false;
  }
  return true;
}

bool
robethichor_interfaces__srv__NegotiationService_Request__copy(
  const robethichor_interfaces__srv__NegotiationService_Request * input,
  robethichor_interfaces__srv__NegotiationService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // tasks
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->tasks), &(output->tasks)))
  {
    return false;
  }
  return true;
}

robethichor_interfaces__srv__NegotiationService_Request *
robethichor_interfaces__srv__NegotiationService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Request * msg = (robethichor_interfaces__srv__NegotiationService_Request *)allocator.allocate(sizeof(robethichor_interfaces__srv__NegotiationService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robethichor_interfaces__srv__NegotiationService_Request));
  bool success = robethichor_interfaces__srv__NegotiationService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robethichor_interfaces__srv__NegotiationService_Request__destroy(robethichor_interfaces__srv__NegotiationService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robethichor_interfaces__srv__NegotiationService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robethichor_interfaces__srv__NegotiationService_Request__Sequence__init(robethichor_interfaces__srv__NegotiationService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Request * data = NULL;

  if (size) {
    data = (robethichor_interfaces__srv__NegotiationService_Request *)allocator.zero_allocate(size, sizeof(robethichor_interfaces__srv__NegotiationService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robethichor_interfaces__srv__NegotiationService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robethichor_interfaces__srv__NegotiationService_Request__fini(&data[i - 1]);
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
robethichor_interfaces__srv__NegotiationService_Request__Sequence__fini(robethichor_interfaces__srv__NegotiationService_Request__Sequence * array)
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
      robethichor_interfaces__srv__NegotiationService_Request__fini(&array->data[i]);
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

robethichor_interfaces__srv__NegotiationService_Request__Sequence *
robethichor_interfaces__srv__NegotiationService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Request__Sequence * array = (robethichor_interfaces__srv__NegotiationService_Request__Sequence *)allocator.allocate(sizeof(robethichor_interfaces__srv__NegotiationService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robethichor_interfaces__srv__NegotiationService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robethichor_interfaces__srv__NegotiationService_Request__Sequence__destroy(robethichor_interfaces__srv__NegotiationService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robethichor_interfaces__srv__NegotiationService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robethichor_interfaces__srv__NegotiationService_Request__Sequence__are_equal(const robethichor_interfaces__srv__NegotiationService_Request__Sequence * lhs, const robethichor_interfaces__srv__NegotiationService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robethichor_interfaces__srv__NegotiationService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robethichor_interfaces__srv__NegotiationService_Request__Sequence__copy(
  const robethichor_interfaces__srv__NegotiationService_Request__Sequence * input,
  robethichor_interfaces__srv__NegotiationService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robethichor_interfaces__srv__NegotiationService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robethichor_interfaces__srv__NegotiationService_Request * data =
      (robethichor_interfaces__srv__NegotiationService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robethichor_interfaces__srv__NegotiationService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robethichor_interfaces__srv__NegotiationService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robethichor_interfaces__srv__NegotiationService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `outcome`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
robethichor_interfaces__srv__NegotiationService_Response__init(robethichor_interfaces__srv__NegotiationService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // outcome
  if (!rosidl_runtime_c__String__init(&msg->outcome)) {
    robethichor_interfaces__srv__NegotiationService_Response__fini(msg);
    return false;
  }
  return true;
}

void
robethichor_interfaces__srv__NegotiationService_Response__fini(robethichor_interfaces__srv__NegotiationService_Response * msg)
{
  if (!msg) {
    return;
  }
  // outcome
  rosidl_runtime_c__String__fini(&msg->outcome);
}

bool
robethichor_interfaces__srv__NegotiationService_Response__are_equal(const robethichor_interfaces__srv__NegotiationService_Response * lhs, const robethichor_interfaces__srv__NegotiationService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // outcome
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->outcome), &(rhs->outcome)))
  {
    return false;
  }
  return true;
}

bool
robethichor_interfaces__srv__NegotiationService_Response__copy(
  const robethichor_interfaces__srv__NegotiationService_Response * input,
  robethichor_interfaces__srv__NegotiationService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // outcome
  if (!rosidl_runtime_c__String__copy(
      &(input->outcome), &(output->outcome)))
  {
    return false;
  }
  return true;
}

robethichor_interfaces__srv__NegotiationService_Response *
robethichor_interfaces__srv__NegotiationService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Response * msg = (robethichor_interfaces__srv__NegotiationService_Response *)allocator.allocate(sizeof(robethichor_interfaces__srv__NegotiationService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robethichor_interfaces__srv__NegotiationService_Response));
  bool success = robethichor_interfaces__srv__NegotiationService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robethichor_interfaces__srv__NegotiationService_Response__destroy(robethichor_interfaces__srv__NegotiationService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robethichor_interfaces__srv__NegotiationService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robethichor_interfaces__srv__NegotiationService_Response__Sequence__init(robethichor_interfaces__srv__NegotiationService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Response * data = NULL;

  if (size) {
    data = (robethichor_interfaces__srv__NegotiationService_Response *)allocator.zero_allocate(size, sizeof(robethichor_interfaces__srv__NegotiationService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robethichor_interfaces__srv__NegotiationService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robethichor_interfaces__srv__NegotiationService_Response__fini(&data[i - 1]);
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
robethichor_interfaces__srv__NegotiationService_Response__Sequence__fini(robethichor_interfaces__srv__NegotiationService_Response__Sequence * array)
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
      robethichor_interfaces__srv__NegotiationService_Response__fini(&array->data[i]);
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

robethichor_interfaces__srv__NegotiationService_Response__Sequence *
robethichor_interfaces__srv__NegotiationService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robethichor_interfaces__srv__NegotiationService_Response__Sequence * array = (robethichor_interfaces__srv__NegotiationService_Response__Sequence *)allocator.allocate(sizeof(robethichor_interfaces__srv__NegotiationService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robethichor_interfaces__srv__NegotiationService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robethichor_interfaces__srv__NegotiationService_Response__Sequence__destroy(robethichor_interfaces__srv__NegotiationService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robethichor_interfaces__srv__NegotiationService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robethichor_interfaces__srv__NegotiationService_Response__Sequence__are_equal(const robethichor_interfaces__srv__NegotiationService_Response__Sequence * lhs, const robethichor_interfaces__srv__NegotiationService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robethichor_interfaces__srv__NegotiationService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robethichor_interfaces__srv__NegotiationService_Response__Sequence__copy(
  const robethichor_interfaces__srv__NegotiationService_Response__Sequence * input,
  robethichor_interfaces__srv__NegotiationService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robethichor_interfaces__srv__NegotiationService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robethichor_interfaces__srv__NegotiationService_Response * data =
      (robethichor_interfaces__srv__NegotiationService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robethichor_interfaces__srv__NegotiationService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robethichor_interfaces__srv__NegotiationService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robethichor_interfaces__srv__NegotiationService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
