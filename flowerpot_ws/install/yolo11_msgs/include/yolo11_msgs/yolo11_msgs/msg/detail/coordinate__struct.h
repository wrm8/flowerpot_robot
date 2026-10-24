// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo11_msgs:msg/Coordinate.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE__STRUCT_H_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Coordinate in the package yolo11_msgs.
typedef struct yolo11_msgs__msg__Coordinate
{
  float x;
  float y;
} yolo11_msgs__msg__Coordinate;

// Struct for a sequence of yolo11_msgs__msg__Coordinate.
typedef struct yolo11_msgs__msg__Coordinate__Sequence
{
  yolo11_msgs__msg__Coordinate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo11_msgs__msg__Coordinate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE__STRUCT_H_
