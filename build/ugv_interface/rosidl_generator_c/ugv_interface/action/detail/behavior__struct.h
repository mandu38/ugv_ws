// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ugv_interface:action/Behavior.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__STRUCT_H_
#define UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_Goal
{
  rosidl_runtime_c__String command;
} ugv_interface__action__Behavior_Goal;

// Struct for a sequence of ugv_interface__action__Behavior_Goal.
typedef struct ugv_interface__action__Behavior_Goal__Sequence
{
  ugv_interface__action__Behavior_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_Result
{
  bool result;
} ugv_interface__action__Behavior_Result;

// Struct for a sequence of ugv_interface__action__Behavior_Result.
typedef struct ugv_interface__action__Behavior_Result__Sequence
{
  ugv_interface__action__Behavior_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_Feedback
{
  bool feedback;
} ugv_interface__action__Behavior_Feedback;

// Struct for a sequence of ugv_interface__action__Behavior_Feedback.
typedef struct ugv_interface__action__Behavior_Feedback__Sequence
{
  ugv_interface__action__Behavior_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ugv_interface/action/detail/behavior__struct.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ugv_interface__action__Behavior_Goal goal;
} ugv_interface__action__Behavior_SendGoal_Request;

// Struct for a sequence of ugv_interface__action__Behavior_SendGoal_Request.
typedef struct ugv_interface__action__Behavior_SendGoal_Request__Sequence
{
  ugv_interface__action__Behavior_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ugv_interface__action__Behavior_SendGoal_Response;

// Struct for a sequence of ugv_interface__action__Behavior_SendGoal_Response.
typedef struct ugv_interface__action__Behavior_SendGoal_Response__Sequence
{
  ugv_interface__action__Behavior_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ugv_interface__action__Behavior_GetResult_Request;

// Struct for a sequence of ugv_interface__action__Behavior_GetResult_Request.
typedef struct ugv_interface__action__Behavior_GetResult_Request__Sequence
{
  ugv_interface__action__Behavior_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_GetResult_Response
{
  int8_t status;
  ugv_interface__action__Behavior_Result result;
} ugv_interface__action__Behavior_GetResult_Response;

// Struct for a sequence of ugv_interface__action__Behavior_GetResult_Response.
typedef struct ugv_interface__action__Behavior_GetResult_Response__Sequence
{
  ugv_interface__action__Behavior_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"

/// Struct defined in action/Behavior in the package ugv_interface.
typedef struct ugv_interface__action__Behavior_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ugv_interface__action__Behavior_Feedback feedback;
} ugv_interface__action__Behavior_FeedbackMessage;

// Struct for a sequence of ugv_interface__action__Behavior_FeedbackMessage.
typedef struct ugv_interface__action__Behavior_FeedbackMessage__Sequence
{
  ugv_interface__action__Behavior_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ugv_interface__action__Behavior_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__STRUCT_H_
