// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from apriltag_msgs:msg/AprilTagDetection.idl
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__DETAIL__APRIL_TAG_DETECTION__BUILDER_HPP_
#define APRILTAG_MSGS__MSG__DETAIL__APRIL_TAG_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "apriltag_msgs/msg/detail/april_tag_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace apriltag_msgs
{

namespace msg
{

namespace builder
{

class Init_AprilTagDetection_pose
{
public:
  explicit Init_AprilTagDetection_pose(::apriltag_msgs::msg::AprilTagDetection & msg)
  : msg_(msg)
  {}
  ::apriltag_msgs::msg::AprilTagDetection pose(::apriltag_msgs::msg::AprilTagDetection::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::apriltag_msgs::msg::AprilTagDetection msg_;
};

class Init_AprilTagDetection_size
{
public:
  explicit Init_AprilTagDetection_size(::apriltag_msgs::msg::AprilTagDetection & msg)
  : msg_(msg)
  {}
  Init_AprilTagDetection_pose size(::apriltag_msgs::msg::AprilTagDetection::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_AprilTagDetection_pose(msg_);
  }

private:
  ::apriltag_msgs::msg::AprilTagDetection msg_;
};

class Init_AprilTagDetection_id
{
public:
  Init_AprilTagDetection_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AprilTagDetection_size id(::apriltag_msgs::msg::AprilTagDetection::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_AprilTagDetection_size(msg_);
  }

private:
  ::apriltag_msgs::msg::AprilTagDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::apriltag_msgs::msg::AprilTagDetection>()
{
  return apriltag_msgs::msg::builder::Init_AprilTagDetection_id();
}

}  // namespace apriltag_msgs

#endif  // APRILTAG_MSGS__MSG__DETAIL__APRIL_TAG_DETECTION__BUILDER_HPP_
