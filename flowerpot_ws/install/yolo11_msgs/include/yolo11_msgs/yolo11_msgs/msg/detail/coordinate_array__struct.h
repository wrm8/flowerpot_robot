// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__STRUCT_H_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'array'
#include "yolo11_msgs/msg/detail/coordinate__struct.h"

/// Struct defined in msg/CoordinateArray in the package yolo11_msgs.
typedef struct yolo11_msgs__msg__CoordinateArray
{
  std_msgs__msg__Header header;
  yolo11_msgs__msg__Coordinate__Sequence array;
} yolo11_msgs__msg__CoordinateArray;

// Struct for a sequence of yolo11_msgs__msg__CoordinateArray.
typedef struct yolo11_msgs__msg__CoordinateArray__Sequence
{
  yolo11_msgs__msg__CoordinateArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo11_msgs__msg__CoordinateArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__STRUCT_H_
