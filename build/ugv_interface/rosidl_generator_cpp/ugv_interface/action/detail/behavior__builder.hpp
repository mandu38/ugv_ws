// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ugv_interface:action/Behavior.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__BUILDER_HPP_
#define UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ugv_interface/action/detail/behavior__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_Goal_command
{
public:
  Init_Behavior_Goal_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ugv_interface::action::Behavior_Goal command(::ugv_interface::action::Behavior_Goal::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_Goal>()
{
  return ugv_interface::action::builder::Init_Behavior_Goal_command();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_Result_result
{
public:
  Init_Behavior_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ugv_interface::action::Behavior_Result result(::ugv_interface::action::Behavior_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_Result>()
{
  return ugv_interface::action::builder::Init_Behavior_Result_result();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_Feedback_feedback
{
public:
  Init_Behavior_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ugv_interface::action::Behavior_Feedback feedback(::ugv_interface::action::Behavior_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_Feedback>()
{
  return ugv_interface::action::builder::Init_Behavior_Feedback_feedback();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_SendGoal_Request_goal
{
public:
  explicit Init_Behavior_SendGoal_Request_goal(::ugv_interface::action::Behavior_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::ugv_interface::action::Behavior_SendGoal_Request goal(::ugv_interface::action::Behavior_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_SendGoal_Request msg_;
};

class Init_Behavior_SendGoal_Request_goal_id
{
public:
  Init_Behavior_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Behavior_SendGoal_Request_goal goal_id(::ugv_interface::action::Behavior_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Behavior_SendGoal_Request_goal(msg_);
  }

private:
  ::ugv_interface::action::Behavior_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_SendGoal_Request>()
{
  return ugv_interface::action::builder::Init_Behavior_SendGoal_Request_goal_id();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_SendGoal_Response_stamp
{
public:
  explicit Init_Behavior_SendGoal_Response_stamp(::ugv_interface::action::Behavior_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::ugv_interface::action::Behavior_SendGoal_Response stamp(::ugv_interface::action::Behavior_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_SendGoal_Response msg_;
};

class Init_Behavior_SendGoal_Response_accepted
{
public:
  Init_Behavior_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Behavior_SendGoal_Response_stamp accepted(::ugv_interface::action::Behavior_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Behavior_SendGoal_Response_stamp(msg_);
  }

private:
  ::ugv_interface::action::Behavior_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_SendGoal_Response>()
{
  return ugv_interface::action::builder::Init_Behavior_SendGoal_Response_accepted();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_GetResult_Request_goal_id
{
public:
  Init_Behavior_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ugv_interface::action::Behavior_GetResult_Request goal_id(::ugv_interface::action::Behavior_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_GetResult_Request>()
{
  return ugv_interface::action::builder::Init_Behavior_GetResult_Request_goal_id();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_GetResult_Response_result
{
public:
  explicit Init_Behavior_GetResult_Response_result(::ugv_interface::action::Behavior_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::ugv_interface::action::Behavior_GetResult_Response result(::ugv_interface::action::Behavior_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_GetResult_Response msg_;
};

class Init_Behavior_GetResult_Response_status
{
public:
  Init_Behavior_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Behavior_GetResult_Response_result status(::ugv_interface::action::Behavior_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Behavior_GetResult_Response_result(msg_);
  }

private:
  ::ugv_interface::action::Behavior_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_GetResult_Response>()
{
  return ugv_interface::action::builder::Init_Behavior_GetResult_Response_status();
}

}  // namespace ugv_interface


namespace ugv_interface
{

namespace action
{

namespace builder
{

class Init_Behavior_FeedbackMessage_feedback
{
public:
  explicit Init_Behavior_FeedbackMessage_feedback(::ugv_interface::action::Behavior_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::ugv_interface::action::Behavior_FeedbackMessage feedback(::ugv_interface::action::Behavior_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ugv_interface::action::Behavior_FeedbackMessage msg_;
};

class Init_Behavior_FeedbackMessage_goal_id
{
public:
  Init_Behavior_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Behavior_FeedbackMessage_feedback goal_id(::ugv_interface::action::Behavior_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Behavior_FeedbackMessage_feedback(msg_);
  }

private:
  ::ugv_interface::action::Behavior_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ugv_interface::action::Behavior_FeedbackMessage>()
{
  return ugv_interface::action::builder::Init_Behavior_FeedbackMessage_goal_id();
}

}  // namespace ugv_interface

#endif  // UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__BUILDER_HPP_
