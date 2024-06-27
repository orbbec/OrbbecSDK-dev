#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Transform a 3d point of a source coordinate system into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point3f Source 3d point value
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
bool ob_calibration_3d_to_3d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                             const ob_sensor_type target_sensor_type, ob_point3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_3d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                             const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point3f *target_point3f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 3d point of the target coordinate system.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point3f Target 3d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_3d_undistortion(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                                          const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point3f *target_point3f,
                                          ob_error **error);

/**
 * @brief Transform a 3d point of a source coordinate system into a 2d pixel coordinate of the target camera.
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point3f Source 3d point value
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
bool ob_calibration_3d_to_2d(const ob_calibration_param calibration_param, const ob_point3f source_point3f, const ob_sensor_type source_sensor_type,
                             const ob_sensor_type target_sensor_type, ob_point2f *target_point2f, ob_error **error);

/**
 * @brief Transform a 2d pixel coordinate with an associated depth value of the source camera into a 2d pixel coordinate of the target camera
 *
 * @param[in] calibration_param Device calibration param,see pipeline::getCalibrationParam
 * @param[in] source_point2f Source 2d point value
 * @param[in] source_depth_pixel_value The depth of sourcePoint2f in millimeters
 * @param[in] source_sensor_type Source sensor type
 * @param[in] target_sensor_type Target sensor type
 * @param[out] target_point2f Target 2d point value
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return bool Transform result
 */
bool ob_calibration_2d_to_2d(const ob_calibration_param calibration_param, const ob_point2f source_point2f, const float source_depth_pixel_value,
                             const ob_sensor_type source_sensor_type, const ob_sensor_type target_sensor_type, ob_point2f *target_point2f, ob_error **error);

#ifdef __cplusplus
}
#endif