// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo11_msgs/msg/detail/coordinate_array__rosidl_typesupport_introspection_c.h"
#include "yolo11_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo11_msgs/msg/detail/coordinate_array__functions.h"
#include "yolo11_msgs/msg/detail/coordinate_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `array`
#include "yolo11_msgs/msg/coordinate.h"
// Member `array`
#include "yolo11_msgs/msg/detail/coordinate__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo11_msgs__msg__CoordinateArray__init(message_memory);
}

void yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_fini_function(void * message_memory)
{
  yolo11_msgs__msg__CoordinateArray__fini(message_memory);
}

size_t yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__size_function__CoordinateArray__array(
  const void * untyped_member)
{
  const yolo11_msgs__msg__Coordinate__Sequence * member =
    (const yolo11_msgs__msg__Coordinate__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_const_function__CoordinateArray__array(
  const void * untyped_member, size_t index)
{
  const yolo11_msgs__msg__Coordinate__Sequence * member =
    (const yolo11_msgs__msg__Coordinate__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_function__CoordinateArray__array(
  void * untyped_member, size_t index)
{
  yolo11_msgs__msg__Coordinate__Sequence * member =
    (yolo11_msgs__msg__Coordinate__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__fetch_function__CoordinateArray__array(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo11_msgs__msg__Coordinate * item =
    ((const yolo11_msgs__msg__Coordinate *)
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_const_function__CoordinateArray__array(untyped_member, index));
  yolo11_msgs__msg__Coordinate * value =
    (yolo11_msgs__msg__Coordinate *)(untyped_value);
  *value = *item;
}

void yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__assign_function__CoordinateArray__array(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo11_msgs__msg__Coordinate * item =
    ((yolo11_msgs__msg__Coordinate *)
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_function__CoordinateArray__array(untyped_member, index));
  const yolo11_msgs__msg__Coordinate * value =
    (const yolo11_msgs__msg__Coordinate *)(untyped_value);
  *item = *value;
}

bool yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__resize_function__CoordinateArray__array(
  void * untyped_member, size_t size)
{
  yolo11_msgs__msg__Coordinate__Sequence * member =
    (yolo11_msgs__msg__Coordinate__Sequence *)(untyped_member);
  yolo11_msgs__msg__Coordinate__Sequence__fini(member);
  return yolo11_msgs__msg__Coordinate__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo11_msgs__msg__CoordinateArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "array",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo11_msgs__msg__CoordinateArray, array),  // bytes offset in struct
    NULL,  // default value
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__size_function__CoordinateArray__array,  // size() function pointer
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_const_function__CoordinateArray__array,  // get_const(index) function pointer
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__get_function__CoordinateArray__array,  // get(index) function pointer
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__fetch_function__CoordinateArray__array,  // fetch(index, &value) function pointer
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__assign_function__CoordinateArray__array,  // assign(index, value) function pointer
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__resize_function__CoordinateArray__array  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_members = {
  "yolo11_msgs__msg",  // message namespace
  "CoordinateArray",  // message name
  2,  // number of fields
  sizeof(yolo11_msgs__msg__CoordinateArray),
  yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_member_array,  // message members
  yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_type_support_handle = {
  0,
  &yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo11_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo11_msgs, msg, CoordinateArray)() {
  yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo11_msgs, msg, Coordinate)();
  if (!yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_type_support_handle.typesupport_identifier) {
    yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo11_msgs__msg__CoordinateArray__rosidl_typesupport_introspection_c__CoordinateArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
