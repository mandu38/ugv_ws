// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ugv_interface:action/Behavior.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ugv_interface/action/detail/behavior__struct.h"
#include "ugv_interface/action/detail/behavior__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_Goal_type_support_ids_t;

static const _Behavior_Goal_type_support_ids_t _Behavior_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_Goal_type_support_symbol_names_t _Behavior_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_Goal)),
  }
};

typedef struct _Behavior_Goal_type_support_data_t
{
  void * data[2];
} _Behavior_Goal_type_support_data_t;

static _Behavior_Goal_type_support_data_t _Behavior_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_Goal_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_Goal)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_Result_type_support_ids_t;

static const _Behavior_Result_type_support_ids_t _Behavior_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_Result_type_support_symbol_names_t _Behavior_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_Result)),
  }
};

typedef struct _Behavior_Result_type_support_data_t
{
  void * data[2];
} _Behavior_Result_type_support_data_t;

static _Behavior_Result_type_support_data_t _Behavior_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_Result_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_Result)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_Feedback_type_support_ids_t;

static const _Behavior_Feedback_type_support_ids_t _Behavior_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_Feedback_type_support_symbol_names_t _Behavior_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_Feedback)),
  }
};

typedef struct _Behavior_Feedback_type_support_data_t
{
  void * data[2];
} _Behavior_Feedback_type_support_data_t;

static _Behavior_Feedback_type_support_data_t _Behavior_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_Feedback_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_Feedback)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_SendGoal_Request_type_support_ids_t;

static const _Behavior_SendGoal_Request_type_support_ids_t _Behavior_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_SendGoal_Request_type_support_symbol_names_t _Behavior_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_SendGoal_Request)),
  }
};

typedef struct _Behavior_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Behavior_SendGoal_Request_type_support_data_t;

static _Behavior_SendGoal_Request_type_support_data_t _Behavior_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_SendGoal_Request_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_SendGoal_Request)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_SendGoal_Response_type_support_ids_t;

static const _Behavior_SendGoal_Response_type_support_ids_t _Behavior_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_SendGoal_Response_type_support_symbol_names_t _Behavior_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_SendGoal_Response)),
  }
};

typedef struct _Behavior_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Behavior_SendGoal_Response_type_support_data_t;

static _Behavior_SendGoal_Response_type_support_data_t _Behavior_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_SendGoal_Response_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_SendGoal_Response)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_SendGoal_type_support_ids_t;

static const _Behavior_SendGoal_type_support_ids_t _Behavior_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_SendGoal_type_support_symbol_names_t _Behavior_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_SendGoal)),
  }
};

typedef struct _Behavior_SendGoal_type_support_data_t
{
  void * data[2];
} _Behavior_SendGoal_type_support_data_t;

static _Behavior_SendGoal_type_support_data_t _Behavior_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_SendGoal_service_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Behavior_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Behavior_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Behavior_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_SendGoal)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_GetResult_Request_type_support_ids_t;

static const _Behavior_GetResult_Request_type_support_ids_t _Behavior_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_GetResult_Request_type_support_symbol_names_t _Behavior_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_GetResult_Request)),
  }
};

typedef struct _Behavior_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Behavior_GetResult_Request_type_support_data_t;

static _Behavior_GetResult_Request_type_support_data_t _Behavior_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_GetResult_Request_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_GetResult_Request)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_GetResult_Response_type_support_ids_t;

static const _Behavior_GetResult_Response_type_support_ids_t _Behavior_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_GetResult_Response_type_support_symbol_names_t _Behavior_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_GetResult_Response)),
  }
};

typedef struct _Behavior_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Behavior_GetResult_Response_type_support_data_t;

static _Behavior_GetResult_Response_type_support_data_t _Behavior_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_GetResult_Response_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_GetResult_Response)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_GetResult_type_support_ids_t;

static const _Behavior_GetResult_type_support_ids_t _Behavior_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_GetResult_type_support_symbol_names_t _Behavior_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_GetResult)),
  }
};

typedef struct _Behavior_GetResult_type_support_data_t
{
  void * data[2];
} _Behavior_GetResult_type_support_data_t;

static _Behavior_GetResult_type_support_data_t _Behavior_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_GetResult_service_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Behavior_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Behavior_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Behavior_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_GetResult)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__struct.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ugv_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Behavior_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Behavior_FeedbackMessage_type_support_ids_t;

static const _Behavior_FeedbackMessage_type_support_ids_t _Behavior_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Behavior_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Behavior_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Behavior_FeedbackMessage_type_support_symbol_names_t _Behavior_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ugv_interface, action, Behavior_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ugv_interface, action, Behavior_FeedbackMessage)),
  }
};

typedef struct _Behavior_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Behavior_FeedbackMessage_type_support_data_t;

static _Behavior_FeedbackMessage_type_support_data_t _Behavior_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Behavior_FeedbackMessage_message_typesupport_map = {
  2,
  "ugv_interface",
  &_Behavior_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Behavior_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Behavior_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Behavior_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Behavior_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ugv_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ugv_interface, action, Behavior_FeedbackMessage)() {
  return &::ugv_interface::action::rosidl_typesupport_c::Behavior_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "ugv_interface/action/behavior.h"
// already included above
// #include "ugv_interface/action/detail/behavior__type_support.h"

static rosidl_action_type_support_t _ugv_interface__action__Behavior__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, ugv_interface, action, Behavior)()
{
  // Thread-safe by always writing the same values to the static struct
  _ugv_interface__action__Behavior__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ugv_interface, action, Behavior_SendGoal)();
  _ugv_interface__action__Behavior__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ugv_interface, action, Behavior_GetResult)();
  _ugv_interface__action__Behavior__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _ugv_interface__action__Behavior__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, ugv_interface, action, Behavior_FeedbackMessage)();
  _ugv_interface__action__Behavior__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_ugv_interface__action__Behavior__typesupport_c;
}

#ifdef __cplusplus
}
#endif
