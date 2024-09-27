// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robethichor_interfaces:srv/UserStatusService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__BUILDER_HPP_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robethichor_interfaces/srv/detail/user_status_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robethichor_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robethichor_interfaces::srv::UserStatusService_Request>()
{
  return ::robethichor_interfaces::srv::UserStatusService_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace robethichor_interfaces


namespace robethichor_interfaces
{

namespace srv
{

namespace builder
{

class Init_UserStatusService_Response_data
{
public:
  Init_UserStatusService_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robethichor_interfaces::srv::UserStatusService_Response data(::robethichor_interfaces::srv::UserStatusService_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robethichor_interfaces::srv::UserStatusService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robethichor_interfaces::srv::UserStatusService_Response>()
{
  return robethichor_interfaces::srv::builder::Init_UserStatusService_Response_data();
}

}  // namespace robethichor_interfaces

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__BUILDER_HPP_
