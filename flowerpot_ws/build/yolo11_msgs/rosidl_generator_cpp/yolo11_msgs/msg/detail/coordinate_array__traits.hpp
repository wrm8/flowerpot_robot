// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__TRAITS_HPP_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo11_msgs/msg/detail/coordinate_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'array'
#include "yolo11_msgs/msg/detail/coordinate__traits.hpp"

namespace yolo11_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CoordinateArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: array
  {
    if (msg.array.size() == 0) {
      out << "array: []";
    } else {
      out << "array: [";
      size_t pending_items = msg.array.size();
      for (auto item : msg.array) {
        to_flow_style_yaml(item, out);
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
  const CoordinateArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: array
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.array.size() == 0) {
      out << "array: []\n";
    } else {
      out << "array:\n";
      for (auto item : msg.array) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CoordinateArray & msg, bool use_flow_style = false)
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

}  // namespace yolo11_msgs

namespace rosidl_generator_traits
{

[[deprecated("use yolo11_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo11_msgs::msg::CoordinateArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo11_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo11_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo11_msgs::msg::CoordinateArray & msg)
{
  return yolo11_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo11_msgs::msg::CoordinateArray>()
{
  return "yolo11_msgs::msg::CoordinateArray";
}

template<>
inline const char * name<yolo11_msgs::msg::CoordinateArray>()
{
  return "yolo11_msgs/msg/CoordinateArray";
}

template<>
struct has_fixed_size<yolo11_msgs::msg::CoordinateArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo11_msgs::msg::CoordinateArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo11_msgs::msg::CoordinateArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__TRAITS_HPP_
