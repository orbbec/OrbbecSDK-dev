/**
 * @file Error.h
 * @brief Functions for handling errors, mainly used for obtaining error messages.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get the error status.
 *
 * @param[in] error The error object.
 * @return The error status.
 */
OB_API ob_status ob_error_get_status(const ob_error *error);

/**
 * @brief Get the error message.
 *
 * @param[in] error The error object.
 * @return The error message.
 */
OB_API const char *ob_error_get_message(const ob_error *error);

/**
 * @brief Get the name of the API function that caused the error.
 *
 * @param[in] error The error object.
 * @return The name of the API function.
 */
OB_API const char *ob_error_get_function(const ob_error *error);

/**
 * @brief Get the error parameters.
 *
 * @param[in] error The error object.
 * @return The error parameters.
 */
OB_API const char *ob_error_get_args(const ob_error *error);

/**
 * @brief Get the type of exception that caused the error.
 *
 * @param[in] error The error object.
 * @return The type of exception.
 */
OB_API ob_exception_type ob_error_get_exception_type(const ob_error *error);

/**
 * @brief Delete the error object.
 *
 * @param[in] error The error object to delete.
 */
OB_API void ob_delete_error(ob_error *error);

#ifdef __cplusplus
}
#endif