// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from feetech_ros2_driver:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_
#define FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "feetech_ros2_driver/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace feetech_ros2_driver
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: ids
  {
    if (msg.ids.size() == 0) {
      out << "ids: []";
    } else {
      out << "ids: [";
      size_t pending_items = msg.ids.size();
      for (auto item : msg.ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocities
  {
    if (msg.velocities.size() == 0) {
      out << "velocities: []";
    } else {
      out << "velocities: [";
      size_t pending_items = msg.velocities.size();
      for (auto item : msg.velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: efforts
  {
    if (msg.efforts.size() == 0) {
      out << "efforts: []";
    } else {
      out << "efforts: [";
      size_t pending_items = msg.efforts.size();
      for (auto item : msg.efforts) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ids.size() == 0) {
      out << "ids: []\n";
    } else {
      out << "ids:\n";
      for (auto item : msg.ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocities.size() == 0) {
      out << "velocities: []\n";
    } else {
      out << "velocities:\n";
      for (auto item : msg.velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: efforts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.efforts.size() == 0) {
      out << "efforts: []\n";
    } else {
      out << "efforts:\n";
      for (auto item : msg.efforts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace feetech_ros2_driver

namespace rosidl_generator_traits
{

[[deprecated("use feetech_ros2_driver::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feetech_ros2_driver::msg::JointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  feetech_ros2_driver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feetech_ros2_driver::msg::to_yaml() instead")]]
inline std::string to_yaml(const feetech_ros2_driver::msg::JointCommand & msg)
{
  return feetech_ros2_driver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<feetech_ros2_driver::msg::JointCommand>()
{
  return "feetech_ros2_driver::msg::JointCommand";
}

template<>
inline const char * name<feetech_ros2_driver::msg::JointCommand>()
{
  return "feetech_ros2_driver/msg/JointCommand";
}

template<>
struct has_fixed_size<feetech_ros2_driver::msg::JointCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<feetech_ros2_driver::msg::JointCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<feetech_ros2_driver::msg::JointCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FEETECH_ROS2_DRIVER__MSG__DETAIL__JOINT_COMMAND__TRAITS_HPP_
