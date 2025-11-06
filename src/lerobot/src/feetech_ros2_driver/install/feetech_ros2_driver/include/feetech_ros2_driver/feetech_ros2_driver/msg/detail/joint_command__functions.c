// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice
#include "feetech_ros2_driver/msg/detail/joint_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ids`
// Member `positions`
// Member `velocities`
// Member `efforts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
feetech_ros2_driver__msg__JointCommand__init(feetech_ros2_driver__msg__JointCommand * msg)
{
  if (!msg) {
    return false;
  }
  // ids
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->ids, 0)) {
    feetech_ros2_driver__msg__JointCommand__fini(msg);
    return false;
  }
  // positions
  if (!rosidl_runtime_c__float__Sequence__init(&msg->positions, 0)) {
    feetech_ros2_driver__msg__JointCommand__fini(msg);
    return false;
  }
  // velocities
  if (!rosidl_runtime_c__float__Sequence__init(&msg->velocities, 0)) {
    feetech_ros2_driver__msg__JointCommand__fini(msg);
    return false;
  }
  // efforts
  if (!rosidl_runtime_c__float__Sequence__init(&msg->efforts, 0)) {
    feetech_ros2_driver__msg__JointCommand__fini(msg);
    return false;
  }
  return true;
}

void
feetech_ros2_driver__msg__JointCommand__fini(feetech_ros2_driver__msg__JointCommand * msg)
{
  if (!msg) {
    return;
  }
  // ids
  rosidl_runtime_c__int32__Sequence__fini(&msg->ids);
  // positions
  rosidl_runtime_c__float__Sequence__fini(&msg->positions);
  // velocities
  rosidl_runtime_c__float__Sequence__fini(&msg->velocities);
  // efforts
  rosidl_runtime_c__float__Sequence__fini(&msg->efforts);
}

bool
feetech_ros2_driver__msg__JointCommand__are_equal(const feetech_ros2_driver__msg__JointCommand * lhs, const feetech_ros2_driver__msg__JointCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ids
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->ids), &(rhs->ids)))
  {
    return false;
  }
  // positions
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  // velocities
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->velocities), &(rhs->velocities)))
  {
    return false;
  }
  // efforts
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->efforts), &(rhs->efforts)))
  {
    return false;
  }
  return true;
}

bool
feetech_ros2_driver__msg__JointCommand__copy(
  const feetech_ros2_driver__msg__JointCommand * input,
  feetech_ros2_driver__msg__JointCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // ids
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->ids), &(output->ids)))
  {
    return false;
  }
  // positions
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  // velocities
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->velocities), &(output->velocities)))
  {
    return false;
  }
  // efforts
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->efforts), &(output->efforts)))
  {
    return false;
  }
  return true;
}

feetech_ros2_driver__msg__JointCommand *
feetech_ros2_driver__msg__JointCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2_driver__msg__JointCommand * msg = (feetech_ros2_driver__msg__JointCommand *)allocator.allocate(sizeof(feetech_ros2_driver__msg__JointCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feetech_ros2_driver__msg__JointCommand));
  bool success = feetech_ros2_driver__msg__JointCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feetech_ros2_driver__msg__JointCommand__destroy(feetech_ros2_driver__msg__JointCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feetech_ros2_driver__msg__JointCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feetech_ros2_driver__msg__JointCommand__Sequence__init(feetech_ros2_driver__msg__JointCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2_driver__msg__JointCommand * data = NULL;

  if (size) {
    data = (feetech_ros2_driver__msg__JointCommand *)allocator.zero_allocate(size, sizeof(feetech_ros2_driver__msg__JointCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feetech_ros2_driver__msg__JointCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feetech_ros2_driver__msg__JointCommand__fini(&data[i - 1]);
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
feetech_ros2_driver__msg__JointCommand__Sequence__fini(feetech_ros2_driver__msg__JointCommand__Sequence * array)
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
      feetech_ros2_driver__msg__JointCommand__fini(&array->data[i]);
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

feetech_ros2_driver__msg__JointCommand__Sequence *
feetech_ros2_driver__msg__JointCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2_driver__msg__JointCommand__Sequence * array = (feetech_ros2_driver__msg__JointCommand__Sequence *)allocator.allocate(sizeof(feetech_ros2_driver__msg__JointCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feetech_ros2_driver__msg__JointCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feetech_ros2_driver__msg__JointCommand__Sequence__destroy(feetech_ros2_driver__msg__JointCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feetech_ros2_driver__msg__JointCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feetech_ros2_driver__msg__JointCommand__Sequence__are_equal(const feetech_ros2_driver__msg__JointCommand__Sequence * lhs, const feetech_ros2_driver__msg__JointCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feetech_ros2_driver__msg__JointCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feetech_ros2_driver__msg__JointCommand__Sequence__copy(
  const feetech_ros2_driver__msg__JointCommand__Sequence * input,
  feetech_ros2_driver__msg__JointCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feetech_ros2_driver__msg__JointCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feetech_ros2_driver__msg__JointCommand * data =
      (feetech_ros2_driver__msg__JointCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feetech_ros2_driver__msg__JointCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feetech_ros2_driver__msg__JointCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feetech_ros2_driver__msg__JointCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
