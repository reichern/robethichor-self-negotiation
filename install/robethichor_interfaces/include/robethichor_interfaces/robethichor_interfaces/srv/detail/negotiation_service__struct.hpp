// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robethichor_interfaces:srv/NegotiationService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_HPP_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robethichor_interfaces__srv__NegotiationService_Request __attribute__((deprecated))
#else
# define DEPRECATED__robethichor_interfaces__srv__NegotiationService_Request __declspec(deprecated)
#endif

namespace robethichor_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NegotiationService_Request_
{
  using Type = NegotiationService_Request_<ContainerAllocator>;

  explicit NegotiationService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit NegotiationService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tasks_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _tasks_type tasks;

  // setters for named parameter idiom
  Type & set__tasks(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->tasks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robethichor_interfaces__srv__NegotiationService_Request
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robethichor_interfaces__srv__NegotiationService_Request
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NegotiationService_Request_ & other) const
  {
    if (this->tasks != other.tasks) {
      return false;
    }
    return true;
  }
  bool operator!=(const NegotiationService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NegotiationService_Request_

// alias to use template instance with default allocator
using NegotiationService_Request =
  robethichor_interfaces::srv::NegotiationService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robethichor_interfaces


#ifndef _WIN32
# define DEPRECATED__robethichor_interfaces__srv__NegotiationService_Response __attribute__((deprecated))
#else
# define DEPRECATED__robethichor_interfaces__srv__NegotiationService_Response __declspec(deprecated)
#endif

namespace robethichor_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct NegotiationService_Response_
{
  using Type = NegotiationService_Response_<ContainerAllocator>;

  explicit NegotiationService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = "";
    }
  }

  explicit NegotiationService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : outcome(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = "";
    }
  }

  // field types and members
  using _outcome_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _outcome_type outcome;

  // setters for named parameter idiom
  Type & set__outcome(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->outcome = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robethichor_interfaces__srv__NegotiationService_Response
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robethichor_interfaces__srv__NegotiationService_Response
    std::shared_ptr<robethichor_interfaces::srv::NegotiationService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NegotiationService_Response_ & other) const
  {
    if (this->outcome != other.outcome) {
      return false;
    }
    return true;
  }
  bool operator!=(const NegotiationService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NegotiationService_Response_

// alias to use template instance with default allocator
using NegotiationService_Response =
  robethichor_interfaces::srv::NegotiationService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robethichor_interfaces

namespace robethichor_interfaces
{

namespace srv
{

struct NegotiationService
{
  using Request = robethichor_interfaces::srv::NegotiationService_Request;
  using Response = robethichor_interfaces::srv::NegotiationService_Response;
};

}  // namespace srv

}  // namespace robethichor_interfaces

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_HPP_
