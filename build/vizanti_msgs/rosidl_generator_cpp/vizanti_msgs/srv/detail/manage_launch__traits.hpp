// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vizanti_msgs:srv/ManageLaunch.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__TRAITS_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vizanti_msgs/srv/detail/manage_launch__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManageLaunch_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: launch
  {
    out << "launch: ";
    rosidl_generator_traits::value_to_yaml(msg.launch, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManageLaunch_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: launch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "launch: ";
    rosidl_generator_traits::value_to_yaml(msg.launch, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManageLaunch_Request & msg, bool use_flow_style = false)
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

}  // namespace vizanti_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vizanti_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vizanti_msgs::srv::ManageLaunch_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::ManageLaunch_Request & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::ManageLaunch_Request>()
{
  return "vizanti_msgs::srv::ManageLaunch_Request";
}

template<>
inline const char * name<vizanti_msgs::srv::ManageLaunch_Request>()
{
  return "vizanti_msgs/srv/ManageLaunch_Request";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ManageLaunch_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::ManageLaunch_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vizanti_msgs::srv::ManageLaunch_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManageLaunch_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManageLaunch_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManageLaunch_Response & msg, bool use_flow_style = false)
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

}  // namespace vizanti_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vizanti_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vizanti_msgs::srv::ManageLaunch_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::ManageLaunch_Response & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::ManageLaunch_Response>()
{
  return "vizanti_msgs::srv::ManageLaunch_Response";
}

template<>
inline const char * name<vizanti_msgs::srv::ManageLaunch_Response>()
{
  return "vizanti_msgs/srv/ManageLaunch_Response";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ManageLaunch_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::ManageLaunch_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vizanti_msgs::srv::ManageLaunch_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vizanti_msgs::srv::ManageLaunch>()
{
  return "vizanti_msgs::srv::ManageLaunch";
}

template<>
inline const char * name<vizanti_msgs::srv::ManageLaunch>()
{
  return "vizanti_msgs/srv/ManageLaunch";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::ManageLaunch>
  : std::integral_constant<
    bool,
    has_fixed_size<vizanti_msgs::srv::ManageLaunch_Request>::value &&
    has_fixed_size<vizanti_msgs::srv::ManageLaunch_Response>::value
  >
{
};

template<>
struct has_bounded_size<vizanti_msgs::srv::ManageLaunch>
  : std::integral_constant<
    bool,
    has_bounded_size<vizanti_msgs::srv::ManageLaunch_Request>::value &&
    has_bounded_size<vizanti_msgs::srv::ManageLaunch_Response>::value
  >
{
};

template<>
struct is_service<vizanti_msgs::srv::ManageLaunch>
  : std::true_type
{
};

template<>
struct is_service_request<vizanti_msgs::srv::ManageLaunch_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vizanti_msgs::srv::ManageLaunch_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VIZANTI_MSGS__SRV__DETAIL__MANAGE_LAUNCH__TRAITS_HPP_
