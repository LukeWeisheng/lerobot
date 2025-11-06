// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
#define FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ids'
// Member 'positions'
// Member 'velocities'
// Member 'efforts'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointCommand in the package feetech_ros2_driver.
typedef struct feetech_ros2_driver__msg__JointCommand
{
  rosidl_runtime_c__int32__Sequence ids;
  rosidl_runtime_c__float__Sequence positions;
  rosidl_runtime_c__float__Sequence velocities;
  rosidl_runtime_c__float__Sequence efforts;
} feetech_ros2_driver__msg__JointCommand;

// Struct for a sequence of feetech_ros2_driver__msg__JointCommand.
typedef struct feetech_ros2_driver__msg__JointCommand__Sequence
{
  feetech_ros2_driver__msg__JointCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feetech_ros2_driver__msg__JointCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
