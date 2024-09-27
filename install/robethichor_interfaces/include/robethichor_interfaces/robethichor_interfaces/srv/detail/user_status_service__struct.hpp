// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robethichor_interfaces:srv/UserStatusService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_HPP_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robethichor_interfaces__srv__UserStatusService_Request __attribute__((deprecated))
#else
# define DEPRECATED__robethichor_interfaces__srv__UserStatusService_Request __declspec(deprecated)
#endif

namespace robethichor_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UserStatusService_Request_
{
  using Type = UserStatusService_Request_<ContainerAllocator>;

  explicit UserStatusService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit UserStatusService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robethichor_interfaces__srv__UserStatusService_Request
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robethichor_interfaces__srv__UserStatusService_Request
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UserStatusService_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const UserStatusService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UserStatusService_Request_

// alias to use template instance with default allocator
using UserStatusService_Request =
  robethichor_interfaces::srv::UserStatusService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robethichor_interfaces


#ifndef _WIN32
# define DEPRECATED__robethichor_interfaces__srv__UserStatusService_Response __attribute__((deprecated))
#else
# define DEPRECATED__robethichor_interfaces__srv__UserStatusService_Response __declspec(deprecated)
#endif

namespace robethichor_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UserStatusService_Response_
{
  using Type = UserStatusService_Response_<ContainerAllocator>;

  explicit UserStatusService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit UserStatusService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robethichor_interfaces__srv__UserStatusService_Response
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robethichor_interfaces__srv__UserStatusService_Response
    std::shared_ptr<robethichor_interfaces::srv::UserStatusService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UserStatusService_Response_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const UserStatusService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UserStatusService_Response_

// alias to use template instance with default allocator
using UserStatusService_Response =
  robethichor_interfaces::srv::UserStatusService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robethichor_interfaces

namespace robethichor_interfaces
{

namespace srv
{

struct UserStatusService
{
  using Request = robethichor_interfaces::srv::UserStatusService_Request;
  using Response = robethichor_interfaces::srv::UserStatusService_Response;
};

}  // namespace srv

}  // namespace robethichor_interfaces

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_HPP_
