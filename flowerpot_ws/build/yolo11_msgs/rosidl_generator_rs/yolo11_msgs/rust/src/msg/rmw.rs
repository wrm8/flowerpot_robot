#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "yolo11_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__yolo11_msgs__msg__Coordinate() -> *const std::ffi::c_void;
}

#[link(name = "yolo11_msgs__rosidl_generator_c")]
extern "C" {
    fn yolo11_msgs__msg__Coordinate__init(msg: *mut Coordinate) -> bool;
    fn yolo11_msgs__msg__Coordinate__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Coordinate>, size: usize) -> bool;
    fn yolo11_msgs__msg__Coordinate__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Coordinate>);
    fn yolo11_msgs__msg__Coordinate__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Coordinate>, out_seq: *mut rosidl_runtime_rs::Sequence<Coordinate>) -> bool;
}

// Corresponds to yolo11_msgs__msg__Coordinate
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Coordinate {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,

}



impl Default for Coordinate {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !yolo11_msgs__msg__Coordinate__init(&mut msg as *mut _) {
        panic!("Call to yolo11_msgs__msg__Coordinate__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Coordinate {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__Coordinate__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__Coordinate__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__Coordinate__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Coordinate {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Coordinate where Self: Sized {
  const TYPE_NAME: &'static str = "yolo11_msgs/msg/Coordinate";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__yolo11_msgs__msg__Coordinate() }
  }
}


#[link(name = "yolo11_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__yolo11_msgs__msg__CoordinateArray() -> *const std::ffi::c_void;
}

#[link(name = "yolo11_msgs__rosidl_generator_c")]
extern "C" {
    fn yolo11_msgs__msg__CoordinateArray__init(msg: *mut CoordinateArray) -> bool;
    fn yolo11_msgs__msg__CoordinateArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CoordinateArray>, size: usize) -> bool;
    fn yolo11_msgs__msg__CoordinateArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CoordinateArray>);
    fn yolo11_msgs__msg__CoordinateArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CoordinateArray>, out_seq: *mut rosidl_runtime_rs::Sequence<CoordinateArray>) -> bool;
}

// Corresponds to yolo11_msgs__msg__CoordinateArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CoordinateArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub array: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Coordinate>,

}



impl Default for CoordinateArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !yolo11_msgs__msg__CoordinateArray__init(&mut msg as *mut _) {
        panic!("Call to yolo11_msgs__msg__CoordinateArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CoordinateArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__CoordinateArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__CoordinateArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo11_msgs__msg__CoordinateArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CoordinateArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CoordinateArray where Self: Sized {
  const TYPE_NAME: &'static str = "yolo11_msgs/msg/CoordinateArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__yolo11_msgs__msg__CoordinateArray() }
  }
}


