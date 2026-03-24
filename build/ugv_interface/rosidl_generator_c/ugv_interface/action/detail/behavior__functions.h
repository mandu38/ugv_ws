// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ugv_interface:action/Behavior.idl
// generated code does not contain a copyright notice

#ifndef UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__FUNCTIONS_H_
#define UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ugv_interface/msg/rosidl_generator_c__visibility_control.h"

#include "ugv_interface/action/detail/behavior__struct.h"

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_Goal
 * )) before or use
 * ugv_interface__action__Behavior_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__init(ugv_interface__action__Behavior_Goal * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Goal__fini(ugv_interface__action__Behavior_Goal * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Goal *
ugv_interface__action__Behavior_Goal__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Goal__destroy(ugv_interface__action__Behavior_Goal * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__are_equal(const ugv_interface__action__Behavior_Goal * lhs, const ugv_interface__action__Behavior_Goal * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__copy(
  const ugv_interface__action__Behavior_Goal * input,
  ugv_interface__action__Behavior_Goal * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__Sequence__init(ugv_interface__action__Behavior_Goal__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Goal__Sequence__fini(ugv_interface__action__Behavior_Goal__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Goal__Sequence *
ugv_interface__action__Behavior_Goal__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Goal__Sequence__destroy(ugv_interface__action__Behavior_Goal__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__Sequence__are_equal(const ugv_interface__action__Behavior_Goal__Sequence * lhs, const ugv_interface__action__Behavior_Goal__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Goal__Sequence__copy(
  const ugv_interface__action__Behavior_Goal__Sequence * input,
  ugv_interface__action__Behavior_Goal__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_Result
 * )) before or use
 * ugv_interface__action__Behavior_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__init(ugv_interface__action__Behavior_Result * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Result__fini(ugv_interface__action__Behavior_Result * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Result *
ugv_interface__action__Behavior_Result__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Result__destroy(ugv_interface__action__Behavior_Result * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__are_equal(const ugv_interface__action__Behavior_Result * lhs, const ugv_interface__action__Behavior_Result * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__copy(
  const ugv_interface__action__Behavior_Result * input,
  ugv_interface__action__Behavior_Result * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__Sequence__init(ugv_interface__action__Behavior_Result__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Result__Sequence__fini(ugv_interface__action__Behavior_Result__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Result__Sequence *
ugv_interface__action__Behavior_Result__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Result__Sequence__destroy(ugv_interface__action__Behavior_Result__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__Sequence__are_equal(const ugv_interface__action__Behavior_Result__Sequence * lhs, const ugv_interface__action__Behavior_Result__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Result__Sequence__copy(
  const ugv_interface__action__Behavior_Result__Sequence * input,
  ugv_interface__action__Behavior_Result__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_Feedback
 * )) before or use
 * ugv_interface__action__Behavior_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__init(ugv_interface__action__Behavior_Feedback * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Feedback__fini(ugv_interface__action__Behavior_Feedback * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Feedback *
ugv_interface__action__Behavior_Feedback__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Feedback__destroy(ugv_interface__action__Behavior_Feedback * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__are_equal(const ugv_interface__action__Behavior_Feedback * lhs, const ugv_interface__action__Behavior_Feedback * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__copy(
  const ugv_interface__action__Behavior_Feedback * input,
  ugv_interface__action__Behavior_Feedback * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__Sequence__init(ugv_interface__action__Behavior_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Feedback__Sequence__fini(ugv_interface__action__Behavior_Feedback__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_Feedback__Sequence *
ugv_interface__action__Behavior_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_Feedback__Sequence__destroy(ugv_interface__action__Behavior_Feedback__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__Sequence__are_equal(const ugv_interface__action__Behavior_Feedback__Sequence * lhs, const ugv_interface__action__Behavior_Feedback__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_Feedback__Sequence__copy(
  const ugv_interface__action__Behavior_Feedback__Sequence * input,
  ugv_interface__action__Behavior_Feedback__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_SendGoal_Request
 * )) before or use
 * ugv_interface__action__Behavior_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__init(ugv_interface__action__Behavior_SendGoal_Request * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Request__fini(ugv_interface__action__Behavior_SendGoal_Request * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_SendGoal_Request *
ugv_interface__action__Behavior_SendGoal_Request__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Request__destroy(ugv_interface__action__Behavior_SendGoal_Request * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__are_equal(const ugv_interface__action__Behavior_SendGoal_Request * lhs, const ugv_interface__action__Behavior_SendGoal_Request * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__copy(
  const ugv_interface__action__Behavior_SendGoal_Request * input,
  ugv_interface__action__Behavior_SendGoal_Request * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__Sequence__init(ugv_interface__action__Behavior_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Request__Sequence__fini(ugv_interface__action__Behavior_SendGoal_Request__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_SendGoal_Request__Sequence *
ugv_interface__action__Behavior_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Request__Sequence__destroy(ugv_interface__action__Behavior_SendGoal_Request__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__Sequence__are_equal(const ugv_interface__action__Behavior_SendGoal_Request__Sequence * lhs, const ugv_interface__action__Behavior_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Request__Sequence__copy(
  const ugv_interface__action__Behavior_SendGoal_Request__Sequence * input,
  ugv_interface__action__Behavior_SendGoal_Request__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_SendGoal_Response
 * )) before or use
 * ugv_interface__action__Behavior_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__init(ugv_interface__action__Behavior_SendGoal_Response * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Response__fini(ugv_interface__action__Behavior_SendGoal_Response * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_SendGoal_Response *
ugv_interface__action__Behavior_SendGoal_Response__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Response__destroy(ugv_interface__action__Behavior_SendGoal_Response * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__are_equal(const ugv_interface__action__Behavior_SendGoal_Response * lhs, const ugv_interface__action__Behavior_SendGoal_Response * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__copy(
  const ugv_interface__action__Behavior_SendGoal_Response * input,
  ugv_interface__action__Behavior_SendGoal_Response * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__Sequence__init(ugv_interface__action__Behavior_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Response__Sequence__fini(ugv_interface__action__Behavior_SendGoal_Response__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_SendGoal_Response__Sequence *
ugv_interface__action__Behavior_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_SendGoal_Response__Sequence__destroy(ugv_interface__action__Behavior_SendGoal_Response__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__Sequence__are_equal(const ugv_interface__action__Behavior_SendGoal_Response__Sequence * lhs, const ugv_interface__action__Behavior_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_SendGoal_Response__Sequence__copy(
  const ugv_interface__action__Behavior_SendGoal_Response__Sequence * input,
  ugv_interface__action__Behavior_SendGoal_Response__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_GetResult_Request
 * )) before or use
 * ugv_interface__action__Behavior_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__init(ugv_interface__action__Behavior_GetResult_Request * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Request__fini(ugv_interface__action__Behavior_GetResult_Request * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_GetResult_Request *
ugv_interface__action__Behavior_GetResult_Request__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Request__destroy(ugv_interface__action__Behavior_GetResult_Request * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__are_equal(const ugv_interface__action__Behavior_GetResult_Request * lhs, const ugv_interface__action__Behavior_GetResult_Request * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__copy(
  const ugv_interface__action__Behavior_GetResult_Request * input,
  ugv_interface__action__Behavior_GetResult_Request * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__Sequence__init(ugv_interface__action__Behavior_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Request__Sequence__fini(ugv_interface__action__Behavior_GetResult_Request__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_GetResult_Request__Sequence *
ugv_interface__action__Behavior_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Request__Sequence__destroy(ugv_interface__action__Behavior_GetResult_Request__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__Sequence__are_equal(const ugv_interface__action__Behavior_GetResult_Request__Sequence * lhs, const ugv_interface__action__Behavior_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Request__Sequence__copy(
  const ugv_interface__action__Behavior_GetResult_Request__Sequence * input,
  ugv_interface__action__Behavior_GetResult_Request__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_GetResult_Response
 * )) before or use
 * ugv_interface__action__Behavior_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__init(ugv_interface__action__Behavior_GetResult_Response * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Response__fini(ugv_interface__action__Behavior_GetResult_Response * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_GetResult_Response *
ugv_interface__action__Behavior_GetResult_Response__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Response__destroy(ugv_interface__action__Behavior_GetResult_Response * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__are_equal(const ugv_interface__action__Behavior_GetResult_Response * lhs, const ugv_interface__action__Behavior_GetResult_Response * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__copy(
  const ugv_interface__action__Behavior_GetResult_Response * input,
  ugv_interface__action__Behavior_GetResult_Response * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__Sequence__init(ugv_interface__action__Behavior_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Response__Sequence__fini(ugv_interface__action__Behavior_GetResult_Response__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_GetResult_Response__Sequence *
ugv_interface__action__Behavior_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_GetResult_Response__Sequence__destroy(ugv_interface__action__Behavior_GetResult_Response__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__Sequence__are_equal(const ugv_interface__action__Behavior_GetResult_Response__Sequence * lhs, const ugv_interface__action__Behavior_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_GetResult_Response__Sequence__copy(
  const ugv_interface__action__Behavior_GetResult_Response__Sequence * input,
  ugv_interface__action__Behavior_GetResult_Response__Sequence * output);

/// Initialize action/Behavior message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ugv_interface__action__Behavior_FeedbackMessage
 * )) before or use
 * ugv_interface__action__Behavior_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__init(ugv_interface__action__Behavior_FeedbackMessage * msg);

/// Finalize action/Behavior message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_FeedbackMessage__fini(ugv_interface__action__Behavior_FeedbackMessage * msg);

/// Create action/Behavior message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ugv_interface__action__Behavior_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_FeedbackMessage *
ugv_interface__action__Behavior_FeedbackMessage__create();

/// Destroy action/Behavior message.
/**
 * It calls
 * ugv_interface__action__Behavior_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_FeedbackMessage__destroy(ugv_interface__action__Behavior_FeedbackMessage * msg);

/// Check for action/Behavior message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__are_equal(const ugv_interface__action__Behavior_FeedbackMessage * lhs, const ugv_interface__action__Behavior_FeedbackMessage * rhs);

/// Copy a action/Behavior message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__copy(
  const ugv_interface__action__Behavior_FeedbackMessage * input,
  ugv_interface__action__Behavior_FeedbackMessage * output);

/// Initialize array of action/Behavior messages.
/**
 * It allocates the memory for the number of elements and calls
 * ugv_interface__action__Behavior_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__Sequence__init(ugv_interface__action__Behavior_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_FeedbackMessage__Sequence__fini(ugv_interface__action__Behavior_FeedbackMessage__Sequence * array);

/// Create array of action/Behavior messages.
/**
 * It allocates the memory for the array and calls
 * ugv_interface__action__Behavior_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
ugv_interface__action__Behavior_FeedbackMessage__Sequence *
ugv_interface__action__Behavior_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Behavior messages.
/**
 * It calls
 * ugv_interface__action__Behavior_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
void
ugv_interface__action__Behavior_FeedbackMessage__Sequence__destroy(ugv_interface__action__Behavior_FeedbackMessage__Sequence * array);

/// Check for action/Behavior message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__Sequence__are_equal(const ugv_interface__action__Behavior_FeedbackMessage__Sequence * lhs, const ugv_interface__action__Behavior_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Behavior messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ugv_interface
bool
ugv_interface__action__Behavior_FeedbackMessage__Sequence__copy(
  const ugv_interface__action__Behavior_FeedbackMessage__Sequence * input,
  ugv_interface__action__Behavior_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UGV_INTERFACE__ACTION__DETAIL__BEHAVIOR__FUNCTIONS_H_
