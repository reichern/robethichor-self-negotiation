// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robethichor_interfaces:srv/NegotiationService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__TRAITS_HPP_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robethichor_interfaces/srv/detail/negotiation_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robethichor_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const NegotiationService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: tasks
  {
    if (msg.tasks.size() == 0) {
      out << "tasks: []";
    } else {
      out << "tasks: [";
      size_t pending_items = msg.tasks.size();
      for (auto item : msg.tasks) {
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
  const NegotiationService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tasks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tasks.size() == 0) {
      out << "tasks: []\n";
    } else {
      out << "tasks:\n";
      for (auto item : msg.tasks) {
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

inline std::string to_yaml(const NegotiationService_Request & msg, bool use_flow_style = false)
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

}  // namespace robethichor_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robethichor_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robethichor_interfaces::srv::NegotiationService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robethichor_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robethichor_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robethichor_interfaces::srv::NegotiationService_Request & msg)
{
  return robethichor_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robethichor_interfaces::srv::NegotiationService_Request>()
{
  return "robethichor_interfaces::srv::NegotiationService_Request";
}

template<>
inline const char * name<robethichor_interfaces::srv::NegotiationService_Request>()
{
  return "robethichor_interfaces/srv/NegotiationService_Request";
}

template<>
struct has_fixed_size<robethichor_interfaces::srv::NegotiationService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robethichor_interfaces::srv::NegotiationService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robethichor_interfaces::srv::NegotiationService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robethichor_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const NegotiationService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: outcome
  {
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NegotiationService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: outcome
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NegotiationService_Response & msg, bool use_flow_style = false)
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

}  // namespace robethichor_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robethichor_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robethichor_interfaces::srv::NegotiationService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robethichor_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robethichor_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const robethichor_interfaces::srv::NegotiationService_Response & msg)
{
  return robethichor_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robethichor_interfaces::srv::NegotiationService_Response>()
{
  return "robethichor_interfaces::srv::NegotiationService_Response";
}

template<>
inline const char * name<robethichor_interfaces::srv::NegotiationService_Response>()
{
  return "robethichor_interfaces/srv/NegotiationService_Response";
}

template<>
struct has_fixed_size<robethichor_interfaces::srv::NegotiationService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robethichor_interfaces::srv::NegotiationService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robethichor_interfaces::srv::NegotiationService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robethichor_interfaces::srv::NegotiationService>()
{
  return "robethichor_interfaces::srv::NegotiationService";
}

template<>
inline const char * name<robethichor_interfaces::srv::NegotiationService>()
{
  return "robethichor_interfaces/srv/NegotiationService";
}

template<>
struct has_fixed_size<robethichor_interfaces::srv::NegotiationService>
  : std::integral_constant<
    bool,
    has_fixed_size<robethichor_interfaces::srv::NegotiationService_Request>::value &&
    has_fixed_size<robethichor_interfaces::srv::NegotiationService_Response>::value
  >
{
};

template<>
struct has_bounded_size<robethichor_interfaces::srv::NegotiationService>
  : std::integral_constant<
    bool,
    has_bounded_size<robethichor_interfaces::srv::NegotiationService_Request>::value &&
    has_bounded_size<robethichor_interfaces::srv::NegotiationService_Response>::value
  >
{
};

template<>
struct is_service<robethichor_interfaces::srv::NegotiationService>
  : std::true_type
{
};

template<>
struct is_service_request<robethichor_interfaces::srv::NegotiationService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robethichor_interfaces::srv::NegotiationService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__TRAITS_HPP_
