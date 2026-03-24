// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ugv_interface:srv/MapSave.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_HPP_
#define UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ugv_interface__srv__MapSave_Request __attribute__((deprecated))
#else
# define DEPRECATED__ugv_interface__srv__MapSave_Request __declspec(deprecated)
#endif

namespace ugv_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MapSave_Request_
{
  using Type = MapSave_Request_<ContainerAllocator>;

  explicit MapSave_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mapname = "";
    }
  }

  explicit MapSave_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mapname(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mapname = "";
    }
  }

  // field types and members
  using _mapname_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mapname_type mapname;

  // setters for named parameter idiom
  Type & set__mapname(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mapname = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ugv_interface::srv::MapSave_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ugv_interface::srv::MapSave_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ugv_interface::srv::MapSave_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ugv_interface::srv::MapSave_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ugv_interface__srv__MapSave_Request
    std::shared_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ugv_interface__srv__MapSave_Request
    std::shared_ptr<ugv_interface::srv::MapSave_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapSave_Request_ & other) const
  {
    if (this->mapname != other.mapname) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapSave_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapSave_Request_

// alias to use template instance with default allocator
using MapSave_Request =
  ugv_interface::srv::MapSave_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ugv_interface


#ifndef _WIN32
# define DEPRECATED__ugv_interface__srv__MapSave_Response __attribute__((deprecated))
#else
# define DEPRECATED__ugv_interface__srv__MapSave_Response __declspec(deprecated)
#endif

namespace ugv_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MapSave_Response_
{
  using Type = MapSave_Response_<ContainerAllocator>;

  explicit MapSave_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  explicit MapSave_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  // field types and members
  using _response_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ugv_interface::srv::MapSave_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ugv_interface::srv::MapSave_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ugv_interface::srv::MapSave_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ugv_interface::srv::MapSave_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ugv_interface__srv__MapSave_Response
    std::shared_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ugv_interface__srv__MapSave_Response
    std::shared_ptr<ugv_interface::srv::MapSave_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapSave_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapSave_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapSave_Response_

// alias to use template instance with default allocator
using MapSave_Response =
  ugv_interface::srv::MapSave_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ugv_interface

namespace ugv_interface
{

namespace srv
{

struct MapSave
{
  using Request = ugv_interface::srv::MapSave_Request;
  using Response = ugv_interface::srv::MapSave_Response;
};

}  // namespace srv

}  // namespace ugv_interface

#endif  // UGV_INTERFACE__SRV__DETAIL__MAP_SAVE__STRUCT_HPP_
