// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vizanti_msgs:srv/ManageLaunch.idl
// generated code does not contain a copyright notice
#include "vizanti_msgs/srv/detail/manage_launch__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vizanti_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vizanti_msgs/srv/detail/manage_launch__struct.h"
#include "vizanti_msgs/srv/detail/manage_launch__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // launch
#include "rosidl_runtime_c/string_functions.h"  // launch

// forward declare type support functions


using _ManageLaunch_Request__ros_msg_type = vizanti_msgs__srv__ManageLaunch_Request;

static bool _ManageLaunch_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ManageLaunch_Request__ros_msg_type * ros_message = static_cast<const _ManageLaunch_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: launch
  {
    const rosidl_runtime_c__String * str = &ros_message->launch;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ManageLaunch_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ManageLaunch_Request__ros_msg_type * ros_message = static_cast<_ManageLaunch_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: launch
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->launch.data) {
      rosidl_runtime_c__String__init(&ros_message->launch);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->launch,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'launch'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vizanti_msgs
size_t get_serialized_size_vizanti_msgs__srv__ManageLaunch_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ManageLaunch_Request__ros_msg_type * ros_message = static_cast<const _ManageLaunch_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name launch
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->launch.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ManageLaunch_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vizanti_msgs__srv__ManageLaunch_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vizanti_msgs
size_t max_serialized_size_vizanti_msgs__srv__ManageLaunch_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: launch
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vizanti_msgs__srv__ManageLaunch_Request;
    is_plain =
      (
      offsetof(DataType, launch) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ManageLaunch_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vizanti_msgs__srv__ManageLaunch_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ManageLaunch_Request = {
  "vizanti_msgs::srv",
  "ManageLaunch_Request",
  _ManageLaunch_Request__cdr_serialize,
  _ManageLaunch_Request__cdr_deserialize,
  _ManageLaunch_Request__get_serialized_size,
  _ManageLaunch_Request__max_serialized_size
};

static rosidl_message_type_support_t _ManageLaunch_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ManageLaunch_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vizanti_msgs, srv, ManageLaunch_Request)() {
  return &_ManageLaunch_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "vizanti_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "vizanti_msgs/srv/detail/manage_launch__struct.h"
// already included above
// #include "vizanti_msgs/srv/detail/manage_launch__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // message
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _ManageLaunch_Response__ros_msg_type = vizanti_msgs__srv__ManageLaunch_Response;

static bool _ManageLaunch_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ManageLaunch_Response__ros_msg_type * ros_message = static_cast<const _ManageLaunch_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _ManageLaunch_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ManageLaunch_Response__ros_msg_type * ros_message = static_cast<_ManageLaunch_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vizanti_msgs
size_t get_serialized_size_vizanti_msgs__srv__ManageLaunch_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ManageLaunch_Response__ros_msg_type * ros_message = static_cast<const _ManageLaunch_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _ManageLaunch_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vizanti_msgs__srv__ManageLaunch_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vizanti_msgs
size_t max_serialized_size_vizanti_msgs__srv__ManageLaunch_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vizanti_msgs__srv__ManageLaunch_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ManageLaunch_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vizanti_msgs__srv__ManageLaunch_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ManageLaunch_Response = {
  "vizanti_msgs::srv",
  "ManageLaunch_Response",
  _ManageLaunch_Response__cdr_serialize,
  _ManageLaunch_Response__cdr_deserialize,
  _ManageLaunch_Response__get_serialized_size,
  _ManageLaunch_Response__max_serialized_size
};

static rosidl_message_type_support_t _ManageLaunch_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ManageLaunch_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vizanti_msgs, srv, ManageLaunch_Response)() {
  return &_ManageLaunch_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "vizanti_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vizanti_msgs/srv/manage_launch.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ManageLaunch__callbacks = {
  "vizanti_msgs::srv",
  "ManageLaunch",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vizanti_msgs, srv, ManageLaunch_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vizanti_msgs, srv, ManageLaunch_Response)(),
};

static rosidl_service_type_support_t ManageLaunch__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ManageLaunch__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vizanti_msgs, srv, ManageLaunch)() {
  return &ManageLaunch__handle;
}

#if defined(__cplusplus)
}
#endif
