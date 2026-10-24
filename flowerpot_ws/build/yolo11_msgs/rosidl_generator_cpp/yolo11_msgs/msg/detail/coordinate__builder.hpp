// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo11_msgs:msg/Coordinate.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE__BUILDER_HPP_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo11_msgs/msg/detail/coordinate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo11_msgs
{

namespace msg
{

namespace builder
{

class Init_Coordinate_y
{
public:
  explicit Init_Coordinate_y(::yolo11_msgs::msg::Coordinate & msg)
  : msg_(msg)
  {}
  ::yolo11_msgs::msg::Coordinate y(::yolo11_msgs::msg::Coordinate::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo11_msgs::msg::Coordinate msg_;
};

class Init_Coordinate_x
{
public:
  Init_Coordinate_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Coordinate_y x(::yolo11_msgs::msg::Coordinate::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Coordinate_y(msg_);
  }

private:
  ::yolo11_msgs::msg::Coordinate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo11_msgs::msg::Coordinate>()
{
  return yolo11_msgs::msg::builder::Init_Coordinate_x();
}

}  // namespace yolo11_msgs

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE__BUILDER_HPP_
