﻿/**
 * @file StreamProfile.h
 * @brief The stream profile related type is used to get information such as the width, height, frame rate, and format of the stream.
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a stream profile object
 *
 * @param[out] error Log error messages
 * @return ob_stream_profile* return the stream profile object
 */
OB_API ob_stream_profile* ob_create_stream_profile(ob_stream_type type, ob_format format,ob_error **error);

/**
 * @brief Create a video stream profile object
 *
 * @param[in] type Stream type
 * @param[in] format Stream format
 * @param[in] width Stream width
 * @param[in] height Stream height
 * @param[in] fps Stream frame rate
 * @param[out] error Log error messages
 * @return ob_stream_profile* return the video stream profile object
 */
OB_API ob_stream_profile* ob_create_video_stream_profile(ob_stream_type type, ob_format format, uint32_t width, uint32_t height, uint32_t fps,ob_error **error);

/**
 * @brief Create a accel stream profile object
 *
 * @param[in] full_scale_range Accel full scale range
 * @param[in] sample_rate Accel sample rate
 * @param[out] error Log error messages
 * @return ob_stream_profile* return the accel stream profile object
 */
OB_API ob_stream_profile* ob_create_accel_stream_profile(ob_accel_full_scale_range full_scale_range, ob_accel_sample_rate sample_rate,ob_error **error);

/**
 * @brief Create a gyro stream profile object
 *
 * @param[in] full_scale_range Gyro full scale range
 * @param[in] sample_rate Gyro sample rate
 * @param[out] error Log error messages
 * @return ob_stream_profile* return the accel stream profile object
 */
OB_API ob_stream_profile* ob_create_gyro_stream_profile(ob_gyro_full_scale_range full_scale_range, ob_gyro_sample_rate sample_rate,ob_error **error);

/**
 * @brief Delete the stream configuration.
 *
 * @param[in] profile Stream profile object .
 * @param[out] error Log error messages.
 */
OB_API void ob_delete_stream_profile(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get stream profile format
 *
 * @param[in] profile  Stream profile object
 * @param[out] error  Log error messages
 * @return ob_format return the format of the stream
 */
OB_API ob_format ob_stream_profile_get_format(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get stream profile type
 *
 * @param[in] profile Stream profile object
 * @param[out] error Log error messages
 * @return ob_stream_type stream type
 */
OB_API ob_stream_type ob_stream_profile_get_type(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the extrinsic for source stream to target stream
 *
 * @param[in] source Source stream profile
 * @param[in]  target Target stream profile
 * @param[out] error Log error messages
 * @return ob_extrinsic The extrinsic
 */
OB_API ob_extrinsic ob_stream_profile_get_extrinsic_to(const ob_stream_profile *source, ob_stream_profile *target, ob_error **error);

/**
 * @brief Set the extrinsic for source stream to target stream
 *
 * @param[in] profile Stream profile object
 * @param[in] target  Target stream type
 * @param[in] extrinsic The extrinsic
 * @param[out] error   Log error messages
 */
OB_API void ob_stream_profile_set_extrinsic_to(ob_stream_profile *source, const ob_stream_profile *target, ob_extrinsic extrinsic, ob_error **error);

/**
 * @brief Get the frame rate of the video stream
 *
 * @param[in] profile Stream profile object
 * @param[out] error Log error messages
 * @return uint32_t return the frame rate of the stream
 */
OB_API uint32_t ob_video_stream_profile_get_fps(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the width of the video stream
 *
 * @param[in] profile Stream profile object , If the profile is not a video stream configuration, an error will be returned
 * @param[out] error Log error messages
 * @return uint32_t return the width of the stream
 */
OB_API uint32_t ob_video_stream_profile_get_width(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the height of the video stream
 *
 * @param[in] profile Stream profile object , If the profile is not a video stream configuration, an error will be returned
 * @param[out] error  Log error messages
 * @return uint32_t return the height of the stream
 */
OB_API uint32_t ob_video_stream_profile_get_height(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the intrinsic of the video stream
 *
 * @param[in]  profile Stream profile object
 * @param[out] error   Log error messages
 * @return ob_camera_intrinsic Return the intrinsic of the stream
 */
OB_API ob_camera_intrinsic ob_video_stream_get_intrinsic(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Set the intrinsic of the video stream
 *
 * @param[in] profile Stream profile object
 * @param[in]  intrinsic The intrinsic of the stream
 * @param[out] error   Log error messages
 */
OB_API void ob_video_stream_set_intrinsic(ob_stream_profile *profile, ob_camera_intrinsic intrinsic, ob_error **error);

/**
 * @brief Get the distortion of the video stream
 *
 * @param[in]  profile Stream profile object
 * @param[out] error   Log error messages
 * @return ob_camera_distortion Return the distortion of the stream
 */
OB_API ob_camera_distortion ob_video_stream_get_distortion(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Set the distortion of the video stream
 *
 * @param[in] profile Stream profile object
 * @param[in]  distortion The distortion of the stream
 * @param[out] error   Log error messages
 */
OB_API void ob_video_stream_set_distortion(ob_stream_profile *profile, ob_camera_distortion distortion, ob_error **error);

/**
 * @brief Get the full-scale range of the accelerometer stream.
 *
 * @param[in] profile Stream profile object. If the profile is not for the accelerometer stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The full-scale range of the accelerometer stream.
 */
OB_API ob_accel_full_scale_range ob_accel_stream_profile_get_full_scale_range(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the sampling frequency of the accelerometer frame.
 *
 * @param[in] profile Stream profile object. If the profile is not for the accelerometer stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The sampling frequency of the accelerometer frame.
 */
OB_API ob_accel_sample_rate ob_accel_stream_profile_get_sample_rate(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the intrinsic of the accelerometer stream.
 *
 * @param[in]  profile Stream profile object. If the profile is not for the accelerometer stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return ob_accel_intrinsic Return the intrinsic of the accelerometer stream.
 */
OB_API ob_accel_intrinsic ob_accel_stream_profile_get_intrinsic(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Set the intrinsic of the accelerometer stream.
 *
 * @param[in] profile Stream profile object. If the profile is not for the accelerometer stream, an error will be returned.
 * @param[in]  intrinsic The intrinsic of the accelerometer stream.
 * @param[out] error Log error messages.
 */
OB_API void ob_accel_stream_profile_set_intrinsic(ob_stream_profile *profile, ob_accel_intrinsic intrinsic, ob_error **error);

/**
 * @brief Get the full-scale range of the gyroscope stream.
 *
 * @param[in] profile Stream profile object. If the profile is not for the gyroscope stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The full-scale range of the gyroscope stream.
 */
OB_API ob_gyro_full_scale_range ob_gyro_stream_profile_get_full_scale_range(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the sampling frequency of the gyroscope stream.
 *
 * @param[in] profile Stream profile object. If the profile is not for the gyroscope stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return The sampling frequency of the gyroscope stream.
 */
OB_API ob_gyro_sample_rate ob_gyro_stream_profile_get_sample_rate(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Get the intrinsic of the gyroscope stream.
 *
 * @param[in]  profile Stream profile object. If the profile is not for the gyroscope stream, an error will be returned.
 * @param[out] error Log error messages.
 * @return ob_gyro_intrinsic Return the intrinsic of the gyroscope stream.
 */
OB_API ob_gyro_intrinsic ob_gyro_stream_get_intrinsic(const ob_stream_profile *profile, ob_error **error);

/**
 * @brief Set the intrinsic of the gyroscope stream.
 *
 * @param[in] profile Stream profile object. If the profile is not for the gyroscope stream, an error will be returned.
 * @param[in]  intrinsic The intrinsic of the gyroscope stream.
 * @param[out] error Log error messages.
 */
OB_API void ob_gyro_stream_set_intrinsic(ob_stream_profile *profile, ob_gyro_intrinsic intrinsic, ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @attention The stream profile returned by this function should be deleted by calling @ref ob_delete_stream_profile() when it is no longer needed.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] width Width. If you don't need to add matching conditions, you can pass OB_WIDTH_ANY.
 * @param[in] height Height. If you don't need to add matching conditions, you can pass OB_HEIGHT_ANY.
 * @param[in] format Format. If you don't need to add matching conditions, you can pass OB_FORMAT_ANY.
 * @param[in] fps Frame rate. If you don't need to add matching conditions, you can pass OB_FPS_ANY.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
OB_API const ob_stream_profile *ob_stream_profile_list_get_video_stream_profile(const ob_stream_profile_list *profile_list, int width, int height, ob_format format, int fps,
                                                                   ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @attention The stream profile returned by this function should be deleted by calling @ref ob_delete_stream_profile() when it is no longer needed.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] full_scale_range Full-scale range. If you don't need to add matching conditions, you can pass 0.
 * @param[in] sample_rate Sample rate. If you don't need to add matching conditions, you can pass 0.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
OB_API const ob_stream_profile *ob_stream_profile_list_get_accel_stream_profile(const ob_stream_profile_list *profile_list, ob_accel_full_scale_range full_scale_range,
                                                                   ob_accel_sample_rate sample_rate, ob_error **error);

/**
 * @brief Match the corresponding ob_stream_profile through the passed parameters. If there are multiple matches,
 * the first one in the list will be returned by default. If no matched profile is found, an error will be returned.
 *
 * @attention The stream profile returned by this function should be deleted by calling @ref ob_delete_stream_profile() when it is no longer needed.
 *
 * @param[in] profile_list Resolution list.
 * @param[in] full_scale_range Full-scale range. If you don't need to add matching conditions, you can pass 0.
 * @param[in] sample_rate Sample rate. If you don't need to add matching conditions, you can pass 0.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
OB_API const ob_stream_profile *ob_stream_profile_list_get_gyro_stream_profile(const ob_stream_profile_list *profile_list, ob_gyro_full_scale_range full_scale_range,
                                                                  ob_gyro_sample_rate sample_rate, ob_error **error);

/**
 * @brief Get the corresponding StreamProfile by subscripting.
 *
 * @attention The stream profile returned by this function should be deleted by calling @ref ob_delete_stream_profile() when it is no longer needed.
 *
 * @param[in] profile_list StreamProfile lists.
 * @param[in] index Index.
 * @param[out] error Log error messages.
 * @return The matching profile.
 */
OB_API const ob_stream_profile *ob_stream_profile_list_get_profile(const ob_stream_profile_list *profile_list, int index, ob_error **error);

/**
 * @brief Get the number of StreamProfile lists.
 *
 * @param[in] profile_list StreamProfile list.
 * @param[out] error Log error messages.
 * @return The number of StreamProfile lists.
 */
OB_API uint32_t ob_stream_profile_list_count(const ob_stream_profile_list *profile_list, ob_error **error);

/**
 * @brief Delete the stream profile list.
 *
 * @param[in] profile_list Stream configuration list.
 * @param[out] error Log error messages.
 */
OB_API void ob_delete_stream_profile_list(const ob_stream_profile_list *profile_list, ob_error **error);


#ifdef __cplusplus
}
#endif