// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vizanti_msgs:srv/Launches.idl
// generated code does not contain a copyright notice

#ifndef VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__STRUCT_HPP_
#define VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vizanti_msgs__srv__Launches_Request __attribute__((deprecated))
#else
# define DEPRECATED__vizanti_msgs__srv__Launches_Request __declspec(deprecated)
#endif

namespace vizanti_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Launches_Request_
{
  using Type = Launches_Request_<ContainerAllocator>;

  explicit Launches_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Launches_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    vizanti_msgs::srv::Launches_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const vizanti_msgs::srv::Launches_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vizanti_msgs::srv::Launches_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vizanti_msgs::srv::Launches_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vizanti_msgs__srv__Launches_Request
    std::shared_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vizanti_msgs__srv__Launches_Request
    std::shared_ptr<vizanti_msgs::srv::Launches_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Launches_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Launches_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Launches_Request_

// alias to use template instance with default allocator
using Launches_Request =
  vizanti_msgs::srv::Launches_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vizanti_msgs


#ifndef _WIN32
# define DEPRECATED__vizanti_msgs__srv__Launches_Response __attribute__((deprecated))
#else
# define DEPRECATED__vizanti_msgs__srv__Launches_Response __declspec(deprecated)
#endif

namespace vizanti_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Launches_Response_
{
  using Type = Launches_Response_<ContainerAllocator>;

  explicit Launches_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Launches_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _launches_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _launches_type launches;

  // setters for named parameter idiom
  Type & set__launches(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->launches = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vizanti_msgs::srv::Launches_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const vizanti_msgs::srv::Launches_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vizanti_msgs::srv::Launches_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vizanti_msgs::srv::Launches_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vizanti_msgs__srv__Launches_Response
    std::shared_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vizanti_msgs__srv__Launches_Response
    std::shared_ptr<vizanti_msgs::srv::Launches_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Launches_Response_ & other) const
  {
    if (this->launches != other.launches) {
      return false;
    }
    return true;
  }
  bool operator!=(const Launches_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Launches_Response_

// alias to use template instance with default allocator
using Launches_Response =
  vizanti_msgs::srv::Launches_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace vizanti_msgs

namespace vizanti_msgs
{

namespace srv
{

struct Launches
{
  using Request = vizanti_msgs::srv::Launches_Request;
  using Response = vizanti_msgs::srv::Launches_Response;
};

}  // namespace srv

}  // namespace vizanti_msgs

#endif  // VIZANTI_MSGS__SRV__DETAIL__LAUNCHES__STRUCT_HPP_
