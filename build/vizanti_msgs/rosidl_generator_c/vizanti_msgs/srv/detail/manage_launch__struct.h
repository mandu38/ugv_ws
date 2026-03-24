// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vizanti_msgs:srv/ManageLaunch.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__STRUCT_H_
#define VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'launch'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ManageLaunch in the package vizanti_msgs.
typedef struct vizanti_msgs__srv__ManageLaunch_Request
{
  rosidl_runtime_c__String launch;
} vizanti_msgs__srv__ManageLaunch_Request;

// Struct for a sequence of vizanti_msgs__srv__ManageLaunch_Request.
typedef struct vizanti_msgs__srv__ManageLaunch_Request__Sequence
{
  vizanti_msgs__srv__ManageLaunch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vizanti_msgs__srv__ManageLaunch_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ManageLaunch in the package vizanti_msgs.
typedef struct vizanti_msgs__srv__ManageLaunch_Response
{
  bool success;
  rosidl_runtime_c__String message;
} vizanti_msgs__srv__ManageLaunch_Response;

// Struct for a sequence of vizanti_msgs__srv__ManageLaunch_Response.
typedef struct vizanti_msgs__srv__ManageLaunch_Response__Sequence
{
  vizanti_msgs__srv__ManageLaunch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vizanti_msgs__srv__ManageLaunch_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__STRUCT_H_
