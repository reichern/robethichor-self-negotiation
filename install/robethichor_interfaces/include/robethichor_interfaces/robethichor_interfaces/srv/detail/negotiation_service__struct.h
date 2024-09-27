// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robethichor_interfaces:srv/NegotiationService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_H_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tasks'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NegotiationService in the package robethichor_interfaces.
typedef struct robethichor_interfaces__srv__NegotiationService_Request
{
  rosidl_runtime_c__String__Sequence tasks;
} robethichor_interfaces__srv__NegotiationService_Request;

// Struct for a sequence of robethichor_interfaces__srv__NegotiationService_Request.
typedef struct robethichor_interfaces__srv__NegotiationService_Request__Sequence
{
  robethichor_interfaces__srv__NegotiationService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robethichor_interfaces__srv__NegotiationService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'outcome'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/NegotiationService in the package robethichor_interfaces.
typedef struct robethichor_interfaces__srv__NegotiationService_Response
{
  rosidl_runtime_c__String outcome;
} robethichor_interfaces__srv__NegotiationService_Response;

// Struct for a sequence of robethichor_interfaces__srv__NegotiationService_Response.
typedef struct robethichor_interfaces__srv__NegotiationService_Response__Sequence
{
  robethichor_interfaces__srv__NegotiationService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robethichor_interfaces__srv__NegotiationService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__NEGOTIATION_SERVICE__STRUCT_H_
