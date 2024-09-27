// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robethichor_interfaces:srv/UserStatusService.idl
// generated code does not contain a copyright notice

#ifndef ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_H_
#define ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/UserStatusService in the package robethichor_interfaces.
typedef struct robethichor_interfaces__srv__UserStatusService_Request
{
  uint8_t structure_needs_at_least_one_member;
} robethichor_interfaces__srv__UserStatusService_Request;

// Struct for a sequence of robethichor_interfaces__srv__UserStatusService_Request.
typedef struct robethichor_interfaces__srv__UserStatusService_Request__Sequence
{
  robethichor_interfaces__srv__UserStatusService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robethichor_interfaces__srv__UserStatusService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UserStatusService in the package robethichor_interfaces.
typedef struct robethichor_interfaces__srv__UserStatusService_Response
{
  rosidl_runtime_c__String data;
} robethichor_interfaces__srv__UserStatusService_Response;

// Struct for a sequence of robethichor_interfaces__srv__UserStatusService_Response.
typedef struct robethichor_interfaces__srv__UserStatusService_Response__Sequence
{
  robethichor_interfaces__srv__UserStatusService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robethichor_interfaces__srv__UserStatusService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBETHICHOR_INTERFACES__SRV__DETAIL__USER_STATUS_SERVICE__STRUCT_H_
