/**
 * @file Pipeline.hpp
 * @brief The SDK's advanced API type can quickly implement switching streaming and frame synchronization
 * operations.
 */
#pragma once

#include "Types.hpp"
#include "Device.hpp"
#include "StreamProfile.hpp"
#include "Frame.hpp"
#include <functional>
#include <memory>
#include "openobsdk/h/Pipeline.h"

namespace ob {

/**
 * @brief Config class for configuring pipeline parameters
 *
 * The Config class provides an interface for configuring pipeline parameters.
 */
class Config {
private:
    ob_config_t *impl_;

public:
    /**
     * @brief Construct a new Config object
     */
    Config() {
        ob_error *error = nullptr;
        impl_           = ob_create_config(&error);
        Error::handle(&error, false);
    }

    explicit Config(ob_config_t *impl) : impl_(impl) {}

    /**
     * @brief Destroy the Config object
     */
    ~Config() noexcept {
        ob_error *error = nullptr;
        ob_delete_config(impl_, &error);
        Error::handle(&error, false);
    }

    ob_config_t *getImpl() const {
        return impl_;
    }

    /**
     * @brief Enable a stream to be used in the pipeline
     *
     * @param streamProfile The stream configuration to be enabled
     */
    void enableStream(std::shared_ptr<StreamProfile> streamProfile) {
        ob_error *error = nullptr;
        ob_config_enable_stream(impl_, streamProfile->type(), &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Enable a video stream to be used in the pipeline.
     *
     * This function allows users to enable a video stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default resolution settings.
     * Users who wish to set custom resolutions should refer to the product manual, as available resolutions vary by camera model.
     *
     * @param type The video stream type.
     * @param width The video stream width (default is OB_WIDTH_ANY, which selects the default resolution).
     * @param height The video stream height (default is OB_HEIGHT_ANY, which selects the default resolution).
     * @param fps The video stream frame rate (default is OB_FPS_ANY, which selects the default frame rate).
     * @param format The video stream format (default is OB_FORMAT_ANY, which selects the default format).
     */
    void enableVideoStream(ob_stream_type stream_type, int width = OB_WIDTH_ANY, int height = OB_HEIGHT_ANY, int fps = OB_FPS_ANY, OBFormat format = OB_FORMAT_ANY){
        ob_error *error = nullptr;
        ob_config_enable_video_stream(impl_, stream_type, width, height, fps, format, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Enable an accelerometer stream to be used in the pipeline.
     *
     * This function allows users to enable an accelerometer stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default settings.
     * Users who wish to set custom full-scale ranges or sample rates should refer to the product manual, as available settings vary by device model.
     *
     * @param fullScaleRange The full-scale range of the accelerometer (default is OB_ACCEL_FULL_SCALE_RANGE_ANY, which selects the default range).
     * @param sampleRate The sample rate of the accelerometer (default is OB_ACCEL_SAMPLE_RATE_ANY, which selects the default rate).
     */
    void enableAccelStream(ob_accel_full_scale_range fullScaleRange = OB_ACCEL_FULL_SCALE_RANGE_ANY,
                           ob_accel_sample_rate      sampleRate     = OB_ACCEL_SAMPLE_RATE_ANY){
        ob_error *error = nullptr;
        ob_config_enable_accel_stream(impl_, fullScaleRange, sampleRate, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Enable a gyroscope stream to be used in the pipeline.
     *
     * This function allows users to enable a gyroscope stream with customizable parameters.
     * If no parameters are specified, the stream will be enabled with default settings.
     * Users who wish to set custom full-scale ranges or sample rates should refer to the product manual, as available settings vary by device model.
     *
     * @param fullScaleRange The full-scale range of the gyroscope (default is OB_GYRO_FULL_SCALE_RANGE_ANY, which selects the default range).
     * @param sampleRate The sample rate of the gyroscope (default is OB_GYRO_SAMPLE_RATE_ANY, which selects the default rate).
     */
    void enableGyroStream(ob_gyro_full_scale_range fullScaleRange = OB_GYRO_FULL_SCALE_RANGE_ANY, ob_gyro_sample_rate sampleRate = OB_GYRO_SAMPLE_RATE_ANY){
        ob_error *error = nullptr;
        ob_config_enable_gyro_stream(impl_, fullScaleRange, sampleRate, &error);
        Error::handle(&error, false);
    }


    /**
     * @brief Disable a stream to be used in the pipeline
     *
     * @param streamType The stream configuration to be disabled
     */
    void disableStream(OBStreamType streamType) {
        ob_error *error = nullptr;
        ob_config_disable_stream(impl_, streamType, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Disable all streams to be used in the pipeline
     */
    void disableAllStream() {
        ob_error *error = nullptr;
        ob_config_disable_all_stream(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the Enabled Stream Profile List
     *
     * @return std::shared_ptr<StreamProfileList>
     */
    std::shared_ptr<StreamProfileList> getEnabledStreamProfileList() {
        ob_error *error = nullptr;
        auto      list  = ob_config_get_enabled_stream_profile_list(impl_, &error);
        Error::handle(&error, false);
        return std::make_shared<StreamProfileList>(list);
    }

    /**
     * @brief Set the alignment mode
     *
     * @param mode The alignment mode
     */
    void setAlignMode(OBAlignMode mode) {
        ob_error *error = nullptr;
        ob_config_set_align_mode(impl_, mode, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Set whether the depth needs to be scaled after setting D2C
     *
     * @param enable Whether scaling is required
     */
    void setDepthScaleRequire(bool enable) {
        ob_error *error = nullptr;
        ob_config_set_depth_scale_after_align_require(impl_, enable, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Set the frame aggregation output mode for the pipeline configuration
     * @brief The processing strategy when the FrameSet generated by the frame aggregation function does not contain the frames of all opened streams (which
     * can be caused by different frame rates of each stream, or by the loss of frames of one stream): drop directly or output to the user.
     *
     * @param mode The frame aggregation output mode to be set (default mode is @ref OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE)
     */
    void setFrameAggregateOutputMode(OBFrameAggregateOutputMode mode) {
        ob_error *error = nullptr;
        ob_config_set_frame_aggregate_output_mode(impl_, mode, &error);
        Error::handle(&error, false);
    }
};

typedef std::function<void(std::shared_ptr<FrameSet> frame)> FrameSetCallback;

class Pipeline {
private:
    ob_pipeline_t   *impl_;
    FrameSetCallback callback_;

public:
    /**
     * @brief Pipeline is a high-level interface for applications, algorithms related RGBD data streams. Pipeline can provide alignment inside and synchronized
     * FrameSet. Pipeline() no parameter version, which opens the first device in the list of devices connected to the OS by default. If the application has
     * obtained the device through the DeviceList, opening the Pipeline() at this time will throw an exception that the device has been created.
     */
    Pipeline() {
        ob_error *error = nullptr;
        impl_           = ob_create_pipeline(&error);
        Error::handle(&error, false);
    }

    /**
     * @brief
     * Pipeline(std::shared_ptr< Device > device ) Function for multi-device operations. Multiple devices need to be obtained through DeviceList, and the device
     * and pipeline are bound through this interface.
     */
    explicit Pipeline(std::shared_ptr<Device> device) {
        ob_error *error = nullptr;
        impl_           = ob_create_pipeline_with_device(device->getImpl(), &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Destroy the pipeline object
     */
    ~Pipeline() noexcept {
        ob_error *error = nullptr;
        ob_delete_pipeline(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Start the pipeline with configuration parameters
     *
     * @param config The parameter configuration of the pipeline
     */
    void start(std::shared_ptr<Config> config) {
        ob_error *error = nullptr;
        ob_pipeline_start_with_config(impl_, config->getImpl(), &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Start the pipeline and set the frameset data callback
     *
     * @param config The configuration of the pipeline
     * @param callback The callback to be triggered when all frame data in the frameset arrives
     */
    void start(std::shared_ptr<Config> config, FrameSetCallback callback) {
        callback_       = callback;
        ob_error *error = nullptr;
        ob_pipeline_start_with_callback(impl_, config->getImpl(), &Pipeline::frameSetCallback, this, &error);
        Error::handle(&error, false);
    }

    static void frameSetCallback(ob_frame_t *frameSet, void *userData) {
        auto pipeline = static_cast<Pipeline *>(userData);
        pipeline->callback_(std::make_shared<FrameSet>(frameSet));
    }

    /**
     * @brief Stop the pipeline
     */
    void stop() {
        ob_error *error = nullptr;
        ob_pipeline_stop(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the pipeline configuration parameters
     * @brief Returns the default configuration if the user has not configured it
     *
     * @return std::shared_ptr<Config> The configured parameters
     */
    std::shared_ptr<Config> getConfig() {
        ob_error *error  = nullptr;
        auto      config = ob_pipeline_get_config(impl_, &error);
        Error::handle(&error, false);
        return std::make_shared<Config>(config);
    }

    /**
     * @brief Wait for frameset data
     *
     * @param timeout_ms The waiting timeout in milliseconds
     * @return std::shared_ptr<FrameSet> The waiting frameset data
     */
    std::shared_ptr<FrameSet> waitForFrames(uint32_t timeout_ms) {
        ob_error *error    = nullptr;
        auto      frameSet = ob_pipeline_wait_for_frameset(impl_, timeout_ms, &error);
        if (frameSet == nullptr) {
            return nullptr;
        }
        Error::handle(&error, false);
        return std::make_shared<FrameSet>(frameSet);
    }

    /**
     * @brief Get the device object
     *
     * @return std::shared_ptr<Device> The device object
     */
    std::shared_ptr<Device> getDevice() {
        ob_error *error  = nullptr;
        auto      device = ob_pipeline_get_device(impl_, &error);
        Error::handle(&error, false);
        return std::make_shared<Device>(device);
    }

    /**
     * @brief Get the stream profile of the specified sensor
     *
     * @param sensorType The type of sensor
     * @return std::shared_ptr<StreamProfileList> The stream profile list
     */
    std::shared_ptr<StreamProfileList> getStreamProfileList(OBSensorType sensorType) {
        ob_error *error = nullptr;
        auto      list  = ob_pipeline_get_stream_profile_list(impl_, sensorType, &error);
        Error::handle(&error, false);
        return std::make_shared<StreamProfileList>(list);
    }

    /**
     * @brief Turn on frame synchronization
     */
    void enableFrameSync() {
        ob_error *error = nullptr;
        ob_pipeline_enable_frame_sync(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Turn off frame synchronization
     */
    void disableFrameSync() {
        ob_error *error = nullptr;
        ob_pipeline_disable_frame_sync(impl_, &error);
        Error::handle(&error, false);
    }
};

}  // namespace ob
