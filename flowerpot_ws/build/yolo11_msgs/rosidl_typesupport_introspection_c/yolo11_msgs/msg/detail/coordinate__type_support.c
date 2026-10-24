// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo11_msgs:msg/Coordinate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo11_msgs/msg/detail/coordinate__rosidl_typesupport_introspection_c.h"
#include "yolo11_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo11_msgs/msg/detail/coordinate__functions.h"
#include "yolo11_msgs/msg/detail/coordinate__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo11_msgs__msg__Coordinate__init(message_memory);
}

void yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_fini_function(void * message_memory)
{
  yolo11_msgs__msg__Coordinate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo11_msgs__msg__Coordinate, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo11_msgs__msg__Coordinate, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_members = {
  "yolo11_msgs__msg",  // message namespace
  "Coordinate",  // message name
  2,  // number of fields
  sizeof(yolo11_msgs__msg__Coordinate),
  yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_member_array,  // message members
  yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_type_support_handle = {
  0,
  &yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo11_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo11_msgs, msg, Coordinate)() {
  if (!yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_type_support_handle.typesupport_identifier) {
    yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo11_msgs__msg__Coordinate__rosidl_typesupport_introspection_c__Coordinate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
