// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
#define FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "feetech_ros2_driver/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace feetech_ros2_driver
{

namespace msg
{

namespace builder
{

class Init_JointCommand_efforts
{
public:
  explicit Init_JointCommand_efforts(::feetech_ros2_driver::msg::JointCommand & msg)
  : msg_(msg)
  {}
  ::feetech_ros2_driver::msg::JointCommand efforts(::feetech_ros2_driver::msg::JointCommand::_efforts_type arg)
  {
    msg_.efforts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feetech_ros2_driver::msg::JointCommand msg_;
};

class Init_JointCommand_velocities
{
public:
  explicit Init_JointCommand_velocities(::feetech_ros2_driver::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_efforts velocities(::feetech_ros2_driver::msg::JointCommand::_velocities_type arg)
  {
    msg_.velocities = std::move(arg);
    return Init_JointCommand_efforts(msg_);
  }

private:
  ::feetech_ros2_driver::msg::JointCommand msg_;
};

class Init_JointCommand_positions
{
public:
  explicit Init_JointCommand_positions(::feetech_ros2_driver::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_velocities positions(::feetech_ros2_driver::msg::JointCommand::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return Init_JointCommand_velocities(msg_);
  }

private:
  ::feetech_ros2_driver::msg::JointCommand msg_;
};

class Init_JointCommand_ids
{
public:
  Init_JointCommand_ids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointCommand_positions ids(::feetech_ros2_driver::msg::JointCommand::_ids_type arg)
  {
    msg_.ids = std::move(arg);
    return Init_JointCommand_positions(msg_);
  }

private:
  ::feetech_ros2_driver::msg::JointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::feetech_ros2_driver::msg::JointCommand>()
{
  return feetech_ros2_driver::msg::builder::Init_JointCommand_ids();
}

}  // namespace feetech_ros2_driver

#endif  // FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
