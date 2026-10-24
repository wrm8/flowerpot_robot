// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo11_msgs:msg/CoordinateArray.idl
// generated code does not contain a copyright notice
#include "yolo11_msgs/msg/detail/coordinate_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `array`
#include "yolo11_msgs/msg/detail/coordinate__functions.h"

bool
yolo11_msgs__msg__CoordinateArray__init(yolo11_msgs__msg__CoordinateArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolo11_msgs__msg__CoordinateArray__fini(msg);
    return false;
  }
  // array
  if (!yolo11_msgs__msg__Coordinate__Sequence__init(&msg->array, 0)) {
    yolo11_msgs__msg__CoordinateArray__fini(msg);
    return false;
  }
  return true;
}

void
yolo11_msgs__msg__CoordinateArray__fini(yolo11_msgs__msg__CoordinateArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // array
  yolo11_msgs__msg__Coordinate__Sequence__fini(&msg->array);
}

bool
yolo11_msgs__msg__CoordinateArray__are_equal(const yolo11_msgs__msg__CoordinateArray * lhs, const yolo11_msgs__msg__CoordinateArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // array
  if (!yolo11_msgs__msg__Coordinate__Sequence__are_equal(
      &(lhs->array), &(rhs->array)))
  {
    return false;
  }
  return true;
}

bool
yolo11_msgs__msg__CoordinateArray__copy(
  const yolo11_msgs__msg__CoordinateArray * input,
  yolo11_msgs__msg__CoordinateArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // array
  if (!yolo11_msgs__msg__Coordinate__Sequence__copy(
      &(input->array), &(output->array)))
  {
    return false;
  }
  return true;
}

yolo11_msgs__msg__CoordinateArray *
yolo11_msgs__msg__CoordinateArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo11_msgs__msg__CoordinateArray * msg = (yolo11_msgs__msg__CoordinateArray *)allocator.allocate(sizeof(yolo11_msgs__msg__CoordinateArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo11_msgs__msg__CoordinateArray));
  bool success = yolo11_msgs__msg__CoordinateArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo11_msgs__msg__CoordinateArray__destroy(yolo11_msgs__msg__CoordinateArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo11_msgs__msg__CoordinateArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo11_msgs__msg__CoordinateArray__Sequence__init(yolo11_msgs__msg__CoordinateArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo11_msgs__msg__CoordinateArray * data = NULL;

  if (size) {
    data = (yolo11_msgs__msg__CoordinateArray *)allocator.zero_allocate(size, sizeof(yolo11_msgs__msg__CoordinateArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo11_msgs__msg__CoordinateArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo11_msgs__msg__CoordinateArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
yolo11_msgs__msg__CoordinateArray__Sequence__fini(yolo11_msgs__msg__CoordinateArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      yolo11_msgs__msg__CoordinateArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

yolo11_msgs__msg__CoordinateArray__Sequence *
yolo11_msgs__msg__CoordinateArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo11_msgs__msg__CoordinateArray__Sequence * array = (yolo11_msgs__msg__CoordinateArray__Sequence *)allocator.allocate(sizeof(yolo11_msgs__msg__CoordinateArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo11_msgs__msg__CoordinateArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo11_msgs__msg__CoordinateArray__Sequence__destroy(yolo11_msgs__msg__CoordinateArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo11_msgs__msg__CoordinateArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo11_msgs__msg__CoordinateArray__Sequence__are_equal(const yolo11_msgs__msg__CoordinateArray__Sequence * lhs, const yolo11_msgs__msg__CoordinateArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo11_msgs__msg__CoordinateArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo11_msgs__msg__CoordinateArray__Sequence__copy(
  const yolo11_msgs__msg__CoordinateArray__Sequence * input,
  yolo11_msgs__msg__CoordinateArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo11_msgs__msg__CoordinateArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo11_msgs__msg__CoordinateArray * data =
      (yolo11_msgs__msg__CoordinateArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo11_msgs__msg__CoordinateArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo11_msgs__msg__CoordinateArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo11_msgs__msg__CoordinateArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
