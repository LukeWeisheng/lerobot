// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_HPP_
#define FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__feetech_ros2_driver__msg__JointCommand __attribute__((deprecated))
#else
# define DEPRECATED__feetech_ros2_driver__msg__JointCommand __declspec(deprecated)
#endif

namespace feetech_ros2_driver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointCommand_
{
  using Type = JointCommand_<ContainerAllocator>;

  explicit JointCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit JointCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _ids_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _ids_type ids;
  using _positions_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _positions_type positions;
  using _velocities_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _velocities_type velocities;
  using _efforts_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _efforts_type efforts;

  // setters for named parameter idiom
  Type & set__ids(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->ids = _arg;
    return *this;
  }
  Type & set__positions(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }
  Type & set__velocities(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->velocities = _arg;
    return *this;
  }
  Type & set__efforts(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->efforts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feetech_ros2_driver__msg__JointCommand
    std::shared_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feetech_ros2_driver__msg__JointCommand
    std::shared_ptr<feetech_ros2_driver::msg::JointCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointCommand_ & other) const
  {
    if (this->ids != other.ids) {
      return false;
    }
    if (this->positions != other.positions) {
      return false;
    }
    if (this->velocities != other.velocities) {
      return false;
    }
    if (this->efforts != other.efforts) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointCommand_

// alias to use template instance with default allocator
using JointCommand =
  feetech_ros2_driver::msg::JointCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace feetech_ros2_driver

#endif  // FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__STRUCT_HPP_
