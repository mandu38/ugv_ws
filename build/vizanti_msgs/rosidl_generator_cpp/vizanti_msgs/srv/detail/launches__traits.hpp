// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vizanti_msgs:srv/Launches.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__TRAITS_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vizanti_msgs/srv/detail/launches__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Launches_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Launches_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Launches_Request & msg, bool use_flow_style = false)
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
  const vizanti_msgs::srv::Launches_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::Launches_Request & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::Launches_Request>()
{
  return "vizanti_msgs::srv::Launches_Request";
}

template<>
inline const char * name<vizanti_msgs::srv::Launches_Request>()
{
  return "vizanti_msgs/srv/Launches_Request";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::Launches_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::Launches_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vizanti_msgs::srv::Launches_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace vizanti_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Launches_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: launches
  {
    if (msg.launches.size() == 0) {
      out << "launches: []";
    } else {
      out << "launches: [";
      size_t pending_items = msg.launches.size();
      for (auto item : msg.launches) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Launches_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: launches
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.launches.size() == 0) {
      out << "launches: []\n";
    } else {
      out << "launches:\n";
      for (auto item : msg.launches) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Launches_Response & msg, bool use_flow_style = false)
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
  const vizanti_msgs::srv::Launches_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vizanti_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vizanti_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vizanti_msgs::srv::Launches_Response & msg)
{
  return vizanti_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vizanti_msgs::srv::Launches_Response>()
{
  return "vizanti_msgs::srv::Launches_Response";
}

template<>
inline const char * name<vizanti_msgs::srv::Launches_Response>()
{
  return "vizanti_msgs/srv/Launches_Response";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::Launches_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vizanti_msgs::srv::Launches_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vizanti_msgs::srv::Launches_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vizanti_msgs::srv::Launches>()
{
  return "vizanti_msgs::srv::Launches";
}

template<>
inline const char * name<vizanti_msgs::srv::Launches>()
{
  return "vizanti_msgs/srv/Launches";
}

template<>
struct has_fixed_size<vizanti_msgs::srv::Launches>
  : std::integral_constant<
    bool,
    has_fixed_size<vizanti_msgs::srv::Launches_Request>::value &&
    has_fixed_size<vizanti_msgs::srv::Launches_Response>::value
  >
{
};

template<>
struct has_bounded_size<vizanti_msgs::srv::Launches>
  : std::integral_constant<
    bool,
    has_bounded_size<vizanti_msgs::srv::Launches_Request>::value &&
    has_bounded_size<vizanti_msgs::srv::Launches_Response>::value
  >
{
};

template<>
struct is_service<vizanti_msgs::srv::Launches>
  : std::true_type
{
};

template<>
struct is_service_request<vizanti_msgs::srv::Launches_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vizanti_msgs::srv::Launches_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__TRAITS_HPP_
