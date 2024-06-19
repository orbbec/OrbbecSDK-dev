/**
 * @file Pipeline.h
 * @brief The SDK's advanced API can quickly implement functions such as switching streaming, frame synchronization, software filtering, etc., suitable for
 * applications, and the algorithm focuses on rgbd data stream scenarios. If you are on real-time or need to handle synchronization separately, align the scene.
 * Please use the interface of Device's Lower API.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Create a pipeline object
 *
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_pipeline* return the pipeline object
 */
OB_EXPORT ob_pipeline *ob_create_pipeline(ob_error **error);

/**
 * @brief Using device objects to create pipeline objects
 *
 * @param[in] dev Device object used to create pipeline
 * @param[out] error  Pointer to an error object that will be set if an error occurs.
 * @return ob_pipeline* return the pipeline object
 */
OB_EXPORT ob_pipeline *ob_create_pipeline_with_device(ob_device *dev, ob_error **error);

/**
 * @brief Delete pipeline objects
 *
 * @param[in] pipeline  The pipeline object to be deleted
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_pipeline(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Start the pipeline with default parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_start(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Start the pipeline with configuration parameters
 *
 * @param[in] pipeline  pipeline object
 * @param[in] config Parameters to be configured
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_start_with_config(ob_pipeline *pipeline, ob_config *config, ob_error **error);

/**
 * @brief Start the pipeline and set the frame collection data callback
 *
 * @param[in] pipeline pipeline object
 * @param[in] config  Parameters to be configured
 * @param[in] callback Trigger a callback when all frame data in the frameset arrives
 * @param[in] user_data  Pass in any user data and get it from the callback
 * @param[out] error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_start_with_callback(ob_pipeline *pipeline, ob_config *config, ob_frameset_callback callback, void *user_data, ob_error **error);

/**
 * @brief Stop pipeline
 *
 * @param[in] pipeline pipeline object
 * @param[out] error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_stop(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Get the configuration object associated with the pipeline
 * @brief Returns default configuration if the user has not configured
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_config* The configuration object
 */
OB_EXPORT ob_config *ob_pipeline_get_config(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Wait for a set of frames to be returned synchronously
 *
 * @param[in] pipeline The pipeline object
 * @param[in] timeout_ms The timeout for waiting (in milliseconds)
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_frame* The frameset that was waited for. A frameset is a special frame that can be used to obtain independent frames from the set.
 */
OB_EXPORT ob_frame *ob_pipeline_wait_for_frameset(ob_pipeline *pipeline, uint32_t timeout_ms, ob_error **error);

/**
 * @brief Get the device object associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_device* The device object
 */
OB_EXPORT ob_device *ob_pipeline_get_device(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Get the stream profile list associated with the pipeline
 *
 * @param[in] pipeline The pipeline object
 * @param[in] sensorType The sensor type. The supported sensor types can be obtained through the ob_device_get_sensor_list() interface.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_stream_profile_list* The stream profile list
 */
OB_EXPORT ob_stream_profile_list *ob_pipeline_get_stream_profile_list(ob_pipeline *pipeline, ob_sensor_type sensorType, ob_error **error);

/**
 * @brief Enable frame synchronization
 * @brief Synchronize the frames of different streams by using the timestamp information of the frames.
 * @brief Dynamically (when pipeline is started) enable/disable frame synchronization is allowed.
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_enable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Disable frame synchronization
 *
 * @param[in] pipeline The pipeline object
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_pipeline_disable_frame_sync(ob_pipeline *pipeline, ob_error **error);

/**
 * @brief Return a list of D2C-enabled depth sensor resolutions corresponding to the input color sensor resolution
 *
 * @param[in] pipeline The pipeline object
 * @param[in] color_profile The input profile of the color sensor
 * @param[in] align_mode The input align mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_stream_profile_list* The list of D2C-enabled depth sensor resolutions
 */
OB_EXPORT ob_stream_profile_list *ob_get_d2c_depth_profile_list(ob_pipeline *pipeline, ob_stream_profile *color_profile, ob_align_mode align_mode,
                                                                ob_error **error);

/**
 * @brief Create the pipeline configuration
 *
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_config* The configuration object
 */
OB_EXPORT ob_config *ob_create_config(ob_error **error);

/**
 * @brief Delete the pipeline configuration
 *
 * @param[in] config The configuration to be deleted
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_config(ob_config *config, ob_error **error);

/**
 * @brief Enable a stream accroding to the stream profile
 *
 * @param[in] config The pipeline configuration
 * @param[in] profile The stream profile to be enabled
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_enable_stream_with_stream_profile(ob_config *config, ob_stream_profile *profile, ob_error **error);

/**
 * @brief Enable a stream with default profile
 *
 * @param[in] config The pipeline configuration
 * @param[in] stream_type The type of the stream to be enabled
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_enable_stream(ob_config *config, ob_stream_type stream_type, ob_error **error);


OB_EXPORT void ob_config_enable_video_stream(ob_config *config, ob_stream_type stream_type, uint32_t width,  uint32_t height,  uint32_t fps,  ob_format format, ob_error **error);
OB_EXPORT void ob_config_enable_accel_stream(ob_config *config, ob_accel_full_scale_range full_scale_range, ob_accel_sample_rate sample_rate, ob_error **error);
OB_EXPORT void ob_config_enable_gyro_stream(ob_config *config, ob_gyro_full_scale_range full_scale_range, ob_gyro_sample_rate sample_rate, ob_error **error);

/**
 * @brief  Get the enabled stream profile list in the pipeline configuration
 *
 * @param config The pipeline configuration
 * @param error Pointer to an error object that will be set if an error occurs.
 * @return ob_stream_profile_list* The enabled stream profile list, should be released by @ref ob_delete_stream_profile_list after use
 */
OB_EXPORT ob_stream_profile_list *ob_config_get_enabled_stream_profile_list(ob_config *config, ob_error **error);

/**
 * @brief Disable a specific stream in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[in] type The type of stream to be disabled
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_disable_stream(ob_config *config, ob_stream_type type, ob_error **error);

/**
 * @brief Disable all streams in the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_disable_all_stream(ob_config *config, ob_error **error);

/**
 * @brief Set the alignment mode for the pipeline configuration
 *
 * @param[in] config The pipeline configuration
 * @param[in] mode The alignment mode to be set
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_set_align_mode(ob_config *config, ob_align_mode mode, ob_error **error);

/**
 * @brief Set whether depth scaling is required after enable depth to color alignment
 * @brief After enabling depth to color alignment, the depth image may need to be scaled to match the color image size.
 *
 * @param[in] config The pipeline configuration
 * @param[in] enable Whether scaling is required
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_config_set_depth_scale_after_align_require(ob_config *config, bool enable, ob_error **error);

/**
 * @brief Set the frame aggregation output mode for the pipeline configuration
 * @brief The processing strategy when the FrameSet generated by the frame aggregation function does not contain the frames of all opened streams (which
 * can be caused by different frame rates of each stream, or by the loss of frames of one stream): drop directly or output to the user.
 *
 * @param[in] config The pipeline configuration
 * @param[in] mode The frame aggregation output mode to be set (default mode is @ref OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE)
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 */
void ob_config_set_frame_aggregate_output_mode(ob_config *config, ob_frame_aggregate_output_mode mode, ob_error **error);

#ifdef __cplusplus
}
#endif
