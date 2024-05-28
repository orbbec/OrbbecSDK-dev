﻿/**
 * @file Frame.h
 * @brief Frame related function is mainly used to obtain frame data and frame information
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get the frame index
 *
 * @param[in] frame Frame object
 * @param[out] error Log wrong message
 * @return uint64_t return the frame index
 */
OB_API uint64_t ob_frame_get_index(ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame format
 *
 * @param[in] frame Frame object
 * @param[out] error  Log error messages
 * @return ob_format return the frame format
 */
OB_API ob_format ob_frame_get_format(ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame type
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return ob_frame_type return the frame type
 */
OB_API ob_frame_type ob_frame_get_type(ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame timestamp (also known as device timestamp, hardware timestamp) of the frame in milliseconds.
 * @brief The hardware timestamp is the time point when the frame was captured by the device (Typically in the mid-exposure, unless otherwise stated), on
 * device clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t return the frame hardware timestamp in milliseconds
 */
OB_API uint64_t ob_frame_get_timestamp_ms(ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame timestamp (also known as device timestamp, hardware timestamp)  of the frame in microseconds.
 * @brief The hardware timestamp is the time point when the frame was captured by the device (Typically in the mid-exposure, unless otherwise stated), on device
 * clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t return the frame hardware timestamp in microseconds
 */
OB_API uint64_t ob_frame_get_timestamp_us(ob_frame *frame, ob_error **error);

/**
 * @brief Set the frame timestamp (also known as the device timestamp, hardware timestamp) of a frame object.
 *
 * @param[in] frame Frame object to set the timestamp.
 * @param[in] timestamp_us frame timestamp to set in microseconds.
 * @param[out] error Log error messages.
 */
OB_API void ob_frame_set_timestamp_us(ob_frame *frame, uint64_t timestamp_us, ob_error **error);

/**
 * @brief Get the system timestamp of the frame in milliseconds.
 * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t return the frame system timestamp in milliseconds
 */
OB_API uint64_t ob_frame_get_system_timestamp_ms(ob_frame *frame, ob_error **error);

/**
 * @brief Get the system timestamp of the frame in microseconds.
 * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t return the frame system timestamp in microseconds
 */
OB_API uint64_t ob_frame_system_get_timestamp_us(ob_frame *frame, ob_error **error);

/**
 * @brief Set the system timestamp of a frame object.
 *
 * @param[in] frame Frame object to set the system timestamp for.
 * @param[in] timestamp_us System timestamp to set in microseconds.
 * @param[out] error Log error messages.
 */
OB_API void ob_frame_set_system_timestamp_us(ob_frame *frame, uint64_t timestamp_us, ob_error **error);

/**
 * @brief Get the global timestamp of the frame in microseconds.
 * @brief The global timestamp is the time point when the frame was was captured by the device, and has been converted to the host clock domain. The
 * conversion process base on the frame timestamp and can eliminate the timer drift of the device
 *
 * @attention Only some models of device support getting the global timestamp. If the device does not support it, this function will return 0. Check the device
 * support status by @ref ob_device_is_global_timestamp_supported() function.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint64_t The global timestamp of the frame in microseconds.
 */
OB_API uint64_t ob_frame_get_global_timestamp_us(ob_frame *frame, ob_error **error);

/**
 * @brief Get frame data
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return void* * return frame data pointer
 */
OB_API uint8_t *ob_frame_get_data(ob_frame *frame, ob_error **error);

/**
 * @brief Get the frame data size
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint32_t return the frame data size
 * If it is point cloud data, it return the number of bytes occupied by all point sets. If you need to find the number of points, you need to divide dataSize
 * by the structure size of the corresponding point type.
 */
OB_API uint32_t ob_frame_get_data_size(ob_frame *frame, ob_error **error);

/**
 * @brief Get the metadata of the frame
 *
 * @param[in] frame frame object
 * @param[out] error Log error messages
 * @return void* return the metadata pointer of the frame
 */
OB_API uint8_t *ob_frame_get_metadata(ob_frame *frame, ob_error **error);
#define ob_video_frame_metadata ob_frame_get_metadata  // for compatibility

/**
 * @brief Get the metadata size of the frame
 *
 * @param[in] frame frame object
 * @param[out] error Log error messages
 * @return uint32_t return the metadata size of the frame
 */
OB_API uint32_t ob_frame_metadata_size(ob_frame *frame, ob_error **error);
#define ob_video_frame_metadata_size ob_frame_metadata_size  // for compatibility

/**
 * @brief check if the frame contains the specified metadata
 *
 * @param[in] frame frame object
 * @param[in] type metadata type, refer to @ref ob_frame_metadata_type
 * @param[out] error Log error messages
 */
OB_API bool ob_frame_has_metadata(ob_frame *frame, ob_frame_metadata_type type, ob_error **error);

/**
 * @brief Get the metadata value of the frame
 *
 * @param[in] frame frame object
 * @param[in] type  metadata type, refer to @ref ob_frame_metadata_type
 * @param[out] error Log error messages
 * @return int64_t return the metadata value of the frame
 */
OB_API int64_t ob_frame_get_metadata_value(ob_frame *frame, ob_frame_metadata_type type, ob_error **error);

/**
 * @brief Get the stream profile of the frame
 *
 * @attention Require @ref ob_delete_stream_profile() to release the return stream profile.
 *
 * @param frame frame object
 * @param error Log error messages
 * @return ob_stream_profile* Return the stream profile of the frame, if the frame is not captured by a sensor stream, it will return NULL
 */
OB_API ob_stream_profile *ob_frame_get_stream_profile(ob_frame *frame, ob_error **error);

/**
 * @brief Set (override) the stream profile of the frame
 *
 * @param frame frame object
 * @param stream_profile The stream profile to set for the frame.
 * @param error Log error messages.
 */
OB_API void ob_frame_set_stream_profile(ob_frame *frame, ob_stream_profile *stream_profile, ob_error **error);

/**
 * @brief Get the sensor of the frame
 *
 * @attention Require @ref ob_delete_sensor() to release the return sensor.
 *
 * @param[in] frame frame object
 * @param[out] error Log error messages
 * @return ob_sensor* return the sensor of the frame, if the frame is not captured by a sensor or the sensor stream has been destroyed, it will return NULL
 */
OB_API ob_sensor *ob_frame_get_sensor(ob_frame *frame, ob_error **error);

/**
 * @brief Get the device of the frame
 *
 * @attention Require @ref ob_delete_device() to release the return device.
 *
 * @param frame frame object
 * @param[out] error Log error messages
 * @return ob_device* return the device of the frame, if the frame is not captured by a sensor stream or the device has been destroyed, it will return NULL
 */
OB_API ob_device *ob_frame_get_device(ob_frame *frame, ob_error **error);

/**
 * @brief Get video frame width
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return uint32_t return the frame width
 */
OB_API uint32_t ob_video_frame_get_width(ob_frame *frame, ob_error **error);

/**
 * @brief Get video frame height
 *
 * @param[in] frame  Frame object
 * @param[out] error Log error messages
 * @return uint32_t return the frame height
 */
OB_API uint32_t ob_video_frame_get_height(ob_frame *frame, ob_error **error);

/**
 * @brief  Get the effective number of pixels (such as Y16 format frame, but only the lower 10 bits are effective bits, and the upper 6 bits are filled with 0)
 * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format
 *
 * @param[in] frame video frame object
 * @param[out] error log error messages
 * @return uint8_t return the effective number of pixels in the pixel, or 0 if it is an unsupported format
 */
OB_API uint8_t ob_video_frame_pixel_available_bit_size(ob_frame *frame, ob_error **error);

/**
 * @brief Get the source sensor type of the ir frame (left or right for dual camera)
 *
 * @param frame Frame object
 * @param ob_error Log error messages
 * @return ob_sensor_type return the source sensor type of the ir frame
 */
OB_API ob_sensor_type ob_ir_frame_get_source_sensor_type(ob_frame *frame, ob_error **ob_error);

/**
 * @brief Get the value scale of the depth frame. The pixel value of the depth frame is multiplied by the scale to give a depth value in millimeters.
 * For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale = 10000*0.1=1000mm.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return float The value scale of the depth frame
 */
OB_API float ob_depth_frame_get_value_scale(ob_frame *frame, ob_error **error);

/**
 * @brief Get the point position value scale of the points frame. The point position value of the points frame is multiplied by the scale to give a position
 * value in millimeters. For example, if scale=0.1, the x-coordinate value of a point is x = 10000, which means that the actual x-coordinate value = x*scale =
 * 10000*0.1 = 1000mm.
 *
 * @param[in] frame Frame object
 * @param[out] error Log error messages
 * @return float The position value scale of the points frame
 */
OB_API float ob_points_frame_get_coordinate_value_scale(ob_frame *frame, ob_error **error);

/**
 * @brief Delete a frame object
 *
 * @param[in] frame The frame object to delete
 * @param[out] error Log error messages
 */
OB_API void ob_delete_frame(ob_frame *frame, ob_error **error);
/**
 * @brief Get the number of frames contained in the frameset
 *
 * @param[in] frameset frameset object
 * @param[out] error Log error messages
 * @return uint32_t return the number of frames
 */
OB_API uint32_t ob_frameset_get_frame_count(ob_frame *frameset, ob_error **error);
/**
 * @brief Get the depth frame from the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the depth frame.
 */
OB_API ob_frame *ob_frameset_get_depth_frame(ob_frame *frameset, ob_error **error);

/**
 * @brief Get the color frame from the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the color frame.
 */
OB_API ob_frame *ob_frameset_get_color_frame(ob_frame *frameset, ob_error **error);

/**
 * @brief Get the infrared frame from the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the infrared frame.
 */
OB_API ob_frame *ob_frameset_get_ir_frame(ob_frame *frameset, ob_error **error);

/**
 * @brief Get point cloud data from the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the point cloud frame.
 */
OB_API ob_frame *ob_frameset_get_points_frame(ob_frame *frameset, ob_error **error);

/**
 * @brief Get a frame of a specific type from the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[in] frame_type Frame type.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the frame of the specified type, or nullptr if it does not exist.
 */
OB_API ob_frame *ob_frameset_get_frame(ob_frame *frameset, ob_frame_type frame_type, ob_error **error);

/**
 * @brief Get a frame at a specific index from the FrameSet
 *
 * @param[in] frameset Frameset object.
 * @param[in] index The index of the frame.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the frame at the specified index, or nullptr if it does not exist.
 */
OB_API ob_frame *ob_frameset_get_frame_by_index(ob_frame *frameset, int index, ob_error **error);

/**
 * @brief Get accelerometer frame data.
 *
 * @param[in] frame Accelerometer frame.
 * @param[out] error Log error messages.
 * @return ob_accel_value Return the accelerometer data.
 */
OB_API ob_accel_value ob_accel_frame_get_value(ob_frame *frame, ob_error **error);

/**
 * @brief Get the temperature when acquiring the accelerometer frame.
 *
 * @param[in] frame Accelerometer frame.
 * @param[out] error Log error messages.
 * @return float Return the temperature value.
 */
OB_API float ob_accel_frame_get_temperature(ob_frame *frame, ob_error **error);

/**
 * @brief Get gyroscope frame data.
 *
 * @param[in] frame Gyroscope frame.
 * @param[out] error Log error messages.
 * @return ob_gyro_value Return the gyroscope data.
 */
OB_API ob_gyro_value ob_gyro_frame_get_value(ob_frame *frame, ob_error **error);

/**
 * @brief Get the temperature when acquiring the gyroscope frame.
 *
 * @param[in] frame Gyroscope frame.
 * @param[out] error Log error messages.
 * @return float Return the temperature value.
 */
OB_API float ob_gyro_frame_get_temperature(ob_frame *frame, ob_error **error);

/**
 * @brief Increase the reference count of a frame object.
 *
 * @param[in] frame Frame object to increase the reference count.
 * @param[out] error Log error messages.
 */
OB_API void ob_frame_add_ref(ob_frame *frame, ob_error **error);

/**
 * @brief Create an empty frame object based on the specified parameters.
 *
 * @param[in] frame_type Frame object type.
 * @param[in] frame_format Frame object format.
 * @param[in] width Frame object width.
 * @param[in] height Frame object height.
 * @param[in] stride_bytes Row span in bytes. If 0, the stride is calculated based on the width and format.
 * @param[out] error Log error messages.
 * @return ob_frame* Return an empty frame object.
 */
OB_API ob_frame *ob_create_video_frame(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes, ob_error **error);

/**
 * @brief Create a frame object based on an externally created buffer.
 *
 * @param[in] frame_type Frame object type.
 * @param[in] format Frame object format.
 * @param[in] width Frame object width.
 * @param[in] height Frame object height.
 * @param[in] stride_bytes Row span in bytes. If 0, the stride is calculated based on the width and format.
 * @param[in] buffer Frame object buffer.
 * @param[in] buffer_size Frame object buffer size.
 * @param[in] buffer_destroy_cb Destroy callback.
 * @param[in] buffer_destroy_context Destroy context.
 * @param[out] error Log error messages.
 * @return ob_frame* Return the frame object.
 */
OB_API ob_frame *ob_create_video_frame_from_buffer(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes, uint8_t *buffer,
                                            uint32_t buffer_size, ob_frame_destroy_callback *buffer_destroy_cb, void *buffer_destroy_context, ob_error **error);

/**
 * @brief Create a frame object based on stream profile
 * 
 * @param[in] profile Stream profile object
 * @param[out] error Log error messages.
 * @return ob_frame* Return the frame object.
 */
OB_API ob_frame *ob_create_frame_from_stream_profile(ob_stream_profile *profile, ob_error **error);

/**
 * @brief Create an empty frameset object.
 *
 * @param[out] error Log error messages.
 * @return ob_frame* Return the frameset object.
 */
OB_API ob_frame *ob_create_frameset(ob_error **error);

/**
 * @brief Add a frame of the specified type to the frameset.
 *
 * @param[in] frameset Frameset object.
 * @param[in] frame Frame object to add.
 * @param[out] error Log error messages.
 */
OB_API void ob_frameset_push_frame(ob_frame *frameset, ob_frame *frame, ob_error **error);

#ifdef __cplusplus
}
#endif
