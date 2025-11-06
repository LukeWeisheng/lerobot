// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "feetech_ros2_driver/msg/detail/joint_command__rosidl_typesupport_introspection_c.h"
#include "feetech_ros2_driver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "feetech_ros2_driver/msg/detail/joint_command__functions.h"
#include "feetech_ros2_driver/msg/detail/joint_command__struct.h"


// Include directives for member types
// Member `ids`
// Member `positions`
// Member `velocities`
// Member `efforts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  feetech_ros2_driver__msg__JointCommand__init(message_memory);
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function(void * message_memory)
{
  feetech_ros2_driver__msg__JointCommand__fini(message_memory);
}

size_t feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__ids(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__ids(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__positions(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__positions(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocities(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocities(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__efforts(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__efforts(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__efforts(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__efforts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__efforts(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__efforts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__efforts(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__efforts(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array[4] = {
  {
    "ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feetech_ros2_driver__msg__JointCommand, ids),  // bytes offset in struct
    NULL,  // default value
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__ids,  // size() function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__ids,  // get_const(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__ids,  // get(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__ids,  // fetch(index, &value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__ids,  // assign(index, value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__ids  // resize(index) function pointer
  },
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feetech_ros2_driver__msg__JointCommand, positions),  // bytes offset in struct
    NULL,  // default value
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__positions,  // size() function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__positions,  // get_const(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__positions,  // get(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__positions,  // fetch(index, &value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__positions,  // assign(index, value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__positions  // resize(index) function pointer
  },
  {
    "velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feetech_ros2_driver__msg__JointCommand, velocities),  // bytes offset in struct
    NULL,  // default value
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__velocities,  // size() function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocities,  // get_const(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocities,  // get(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__velocities,  // fetch(index, &value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__velocities,  // assign(index, value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__velocities  // resize(index) function pointer
  },
  {
    "efforts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feetech_ros2_driver__msg__JointCommand, efforts),  // bytes offset in struct
    NULL,  // default value
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__efforts,  // size() function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__efforts,  // get_const(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__efforts,  // get(index) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__efforts,  // fetch(index, &value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__efforts,  // assign(index, value) function pointer
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__efforts  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members = {
  "feetech_ros2_driver__msg",  // message namespace
  "JointCommand",  // message name
  4,  // number of fields
  sizeof(feetech_ros2_driver__msg__JointCommand),
  feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array,  // message members
  feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle = {
  0,
  &feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_feetech_ros2_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, feetech_ros2_driver, msg, JointCommand)() {
  if (!feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier) {
    feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &feetech_ros2_driver__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
