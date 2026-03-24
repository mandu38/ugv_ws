// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ugv_interface:srv/MapSave.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_H_
#define UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mapname'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MapSave in the package ugv_interface.
typedef struct ugv_interface__srv__MapSave_Request
{
  rosidl_runtime_c__String mapname;
} ugv_interface__srv__MapSave_Request;

// Struct for a sequence of ugv_interface__srv__MapSave_Request.
typedef struct ugv_interface__srv__MapSave_Request__Sequence
{
  ugv_interface__srv__MapSave_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__srv__MapSave_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MapSave in the package ugv_interface.
typedef struct ugv_interface__srv__MapSave_Response
{
  rosidl_runtime_c__String response;
} ugv_interface__srv__MapSave_Response;

// Struct for a sequence of ugv_interface__srv__MapSave_Response.
typedef struct ugv_interface__srv__MapSave_Response__Sequence
{
  ugv_interface__srv__MapSave_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__srv__MapSave_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_H_
