// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__BUILDER_HPP_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo11_msgs/msg/detail/coordinate_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo11_msgs
{

namespace msg
{

namespace builder
{

class Init_CoordinateArray_array
{
public:
  explicit Init_CoordinateArray_array(::yolo11_msgs::msg::CoordinateArray & msg)
  : msg_(msg)
  {}
  ::yolo11_msgs::msg::CoordinateArray array(::yolo11_msgs::msg::CoordinateArray::_array_type arg)
  {
    msg_.array = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo11_msgs::msg::CoordinateArray msg_;
};

class Init_CoordinateArray_header
{
public:
  Init_CoordinateArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CoordinateArray_array header(::yolo11_msgs::msg::CoordinateArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CoordinateArray_array(msg_);
  }

private:
  ::yolo11_msgs::msg::CoordinateArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo11_msgs::msg::CoordinateArray>()
{
  return yolo11_msgs::msg::builder::Init_CoordinateArray_header();
}

}  // namespace yolo11_msgs

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__BUILDER_HPP_
