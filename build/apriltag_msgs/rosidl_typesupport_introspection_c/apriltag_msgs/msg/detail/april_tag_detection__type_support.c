// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from apriltag_msgs:msg/AprilTagDetection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "apriltag_msgs/msg/detail/april_tag_detection__rosidl_typesupport_introspection_c.h"
#include "apriltag_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "apriltag_msgs/msg/detail/april_tag_detection__functions.h"
#include "apriltag_msgs/msg/detail/april_tag_detection__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose_with_covariance_stamped.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  apriltag_msgs__msg__AprilTagDetection__init(message_memory);
}

void apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_fini_function(void * message_memory)
{
  apriltag_msgs__msg__AprilTagDetection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array[3] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_members = {
  "apriltag_msgs__msg",  // message namespace
  "AprilTagDetection",  // message name
  3,  // number of fields
  sizeof(apriltag_msgs__msg__AprilTagDetection),
  apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array,  // message members
  apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_init_function,  // function to initialize message memory (memory has to be allocated)
  apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle = {
  0,
  &apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_apriltag_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apriltag_msgs, msg, AprilTagDetection)() {
  apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovarianceStamped)();
  if (!apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle.typesupport_identifier) {
    apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &apriltag_msgs__msg__AprilTagDetection__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
