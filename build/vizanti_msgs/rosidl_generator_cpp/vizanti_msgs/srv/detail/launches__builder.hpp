// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vizanti_msgs:srv/Launches.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__BUILDER_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vizanti_msgs/srv/detail/launches__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vizanti_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vizanti_msgs::srv::Launches_Request>()
{
  return ::vizanti_msgs::srv::Launches_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace vizanti_msgs


namespace vizanti_msgs
{

namespace srv
{

namespace builder
{

class Init_Launches_Response_launches
{
public:
  Init_Launches_Response_launches()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vizanti_msgs::srv::Launches_Response launches(::vizanti_msgs::srv::Launches_Response::_launches_type arg)
  {
    msg_.launches = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vizanti_msgs::srv::Launches_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vizanti_msgs::srv::Launches_Response>()
{
  return vizanti_msgs::srv::builder::Init_Launches_Response_launches();
}

}  // namespace vizanti_msgs

#endif  // VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__BUILDER_HPP_
