// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice

#ifndef YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__FUNCTIONS_H_
#define YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yolo11_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "yolo11_msgs/msg/detail/coordinate_array__struct.h"

/// Initialize msg/CoordinateArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yolo11_msgs__msg__CoordinateArray
 * )) before or use
 * yolo11_msgs__msg__CoordinateArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__init(yolo11_msgs__msg__CoordinateArray * msg);

/// Finalize msg/CoordinateArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
void
yolo11_msgs__msg__CoordinateArray__fini(yolo11_msgs__msg__CoordinateArray * msg);

/// Create msg/CoordinateArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yolo11_msgs__msg__CoordinateArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
yolo11_msgs__msg__CoordinateArray *
yolo11_msgs__msg__CoordinateArray__create();

/// Destroy msg/CoordinateArray message.
/**
 * It calls
 * yolo11_msgs__msg__CoordinateArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
void
yolo11_msgs__msg__CoordinateArray__destroy(yolo11_msgs__msg__CoordinateArray * msg);

/// Check for msg/CoordinateArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__are_equal(const yolo11_msgs__msg__CoordinateArray * lhs, const yolo11_msgs__msg__CoordinateArray * rhs);

/// Copy a msg/CoordinateArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__copy(
  const yolo11_msgs__msg__CoordinateArray * input,
  yolo11_msgs__msg__CoordinateArray * output);

/// Initialize array of msg/CoordinateArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * yolo11_msgs__msg__CoordinateArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__Sequence__init(yolo11_msgs__msg__CoordinateArray__Sequence * array, size_t size);

/// Finalize array of msg/CoordinateArray messages.
/**
 * It calls
 * yolo11_msgs__msg__CoordinateArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
void
yolo11_msgs__msg__CoordinateArray__Sequence__fini(yolo11_msgs__msg__CoordinateArray__Sequence * array);

/// Create array of msg/CoordinateArray messages.
/**
 * It allocates the memory for the array and calls
 * yolo11_msgs__msg__CoordinateArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
yolo11_msgs__msg__CoordinateArray__Sequence *
yolo11_msgs__msg__CoordinateArray__Sequence__create(size_t size);

/// Destroy array of msg/CoordinateArray messages.
/**
 * It calls
 * yolo11_msgs__msg__CoordinateArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
void
yolo11_msgs__msg__CoordinateArray__Sequence__destroy(yolo11_msgs__msg__CoordinateArray__Sequence * array);

/// Check for msg/CoordinateArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__Sequence__are_equal(const yolo11_msgs__msg__CoordinateArray__Sequence * lhs, const yolo11_msgs__msg__CoordinateArray__Sequence * rhs);

/// Copy an array of msg/CoordinateArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo11_msgs
bool
yolo11_msgs__msg__CoordinateArray__Sequence__copy(
  const yolo11_msgs__msg__CoordinateArray__Sequence * input,
  yolo11_msgs__msg__CoordinateArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YOLO11_MSGS__MSG__DETAIL__COORDINATE_ARRAY__FUNCTIONS_H_
