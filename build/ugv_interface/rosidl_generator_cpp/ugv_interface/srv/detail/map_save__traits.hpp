// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ugv_interface:srv/MapSave.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__TRAITS_HPP_
#define UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ugv_interface/srv/detail/map_save__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ugv_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MapSave_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mapname
  {
    out << "mapname: ";
    rosidl_generator_traits::value_to_yaml(msg.mapname, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MapSave_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mapname
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mapname: ";
    rosidl_generator_traits::value_to_yaml(msg.mapname, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MapSave_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ugv_interface

namespace rosidl_generator_traits
{

[[deprecated("use ugv_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ugv_interface::srv::MapSave_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ugv_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ugv_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const ugv_interface::srv::MapSave_Request & msg)
{
  return ugv_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ugv_interface::srv::MapSave_Request>()
{
  return "ugv_interface::srv::MapSave_Request";
}

template<>
inline const char * name<ugv_interface::srv::MapSave_Request>()
{
  return "ugv_interface/srv/MapSave_Request";
}

template<>
struct has_fixed_size<ugv_interface::srv::MapSave_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ugv_interface::srv::MapSave_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ugv_interface::srv::MapSave_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ugv_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const MapSave_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MapSave_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MapSave_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ugv_interface

namespace rosidl_generator_traits
{

[[deprecated("use ugv_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ugv_interface::srv::MapSave_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ugv_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ugv_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const ugv_interface::srv::MapSave_Response & msg)
{
  return ugv_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ugv_interface::srv::MapSave_Response>()
{
  return "ugv_interface::srv::MapSave_Response";
}

template<>
inline const char * name<ugv_interface::srv::MapSave_Response>()
{
  return "ugv_interface/srv/MapSave_Response";
}

template<>
struct has_fixed_size<ugv_interface::srv::MapSave_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ugv_interface::srv::MapSave_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ugv_interface::srv::MapSave_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ugv_interface::srv::MapSave>()
{
  return "ugv_interface::srv::MapSave";
}

template<>
inline const char * name<ugv_interface::srv::MapSave>()
{
  return "ugv_interface/srv/MapSave";
}

template<>
struct has_fixed_size<ugv_interface::srv::MapSave>
  : std::integral_constant<
    bool,
    has_fixed_size<ugv_interface::srv::MapSave_Request>::value &&
    has_fixed_size<ugv_interface::srv::MapSave_Response>::value
  >
{
};

template<>
struct has_bounded_size<ugv_interface::srv::MapSave>
  : std::integral_constant<
    bool,
    has_bounded_size<ugv_interface::srv::MapSave_Request>::value &&
    has_bounded_size<ugv_interface::srv::MapSave_Response>::value
  >
{
};

template<>
struct is_service<ugv_interface::srv::MapSave>
  : std::true_type
{
};

template<>
struct is_service_request<ugv_interface::srv::MapSave_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ugv_interface::srv::MapSave_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__TRAITS_HPP_
