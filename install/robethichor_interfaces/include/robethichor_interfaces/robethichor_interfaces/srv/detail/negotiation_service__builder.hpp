// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robethichor_interfaces:srv/NegotiationService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__BUILDER_HPP_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robethichor_interfaces/srv/detail/negotiation_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robethichor_interfaces
{

namespace srv
{

namespace builder
{

class Init_NegotiationService_Request_tasks
{
public:
  Init_NegotiationService_Request_tasks()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robethichor_interfaces::srv::NegotiationService_Request tasks(::robethichor_interfaces::srv::NegotiationService_Request::_tasks_type arg)
  {
    msg_.tasks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robethichor_interfaces::srv::NegotiationService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robethichor_interfaces::srv::NegotiationService_Request>()
{
  return robethichor_interfaces::srv::builder::Init_NegotiationService_Request_tasks();
}

}  // namespace robethichor_interfaces


namespace robethichor_interfaces
{

namespace srv
{

namespace builder
{

class Init_NegotiationService_Response_outcome
{
public:
  Init_NegotiationService_Response_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robethichor_interfaces::srv::NegotiationService_Response outcome(::robethichor_interfaces::srv::NegotiationService_Response::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robethichor_interfaces::srv::NegotiationService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robethichor_interfaces::srv::NegotiationService_Response>()
{
  return robethichor_interfaces::srv::builder::Init_NegotiationService_Response_outcome();
}

}  // namespace robethichor_interfaces

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__BUILDER_HPP_
