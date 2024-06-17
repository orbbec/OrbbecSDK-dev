/**
 * @file Sensor.hpp
 * @brief Defines types related to sensors, which are used to obtain stream configurations, open and close streams, and set and get sensor properties.
 */
#pragma once

#include "Types.hpp"
#include "openobsdk/hpp/Filter.hpp"
#include "openobsdk/h/Sensor.h"
#include "openobsdk/h/Filter.h"
#include "Error.hpp"
#include "StreamProfile.hpp"
#include "Device.hpp"
#include "Frame.hpp"
#include <functional>
#include <memory>

namespace ob {

/**
 * @brief Callback function for frame data.
 *
 * @param frame The frame data.
 */
using FrameCallback = std::function<void(std::shared_ptr<Frame> frame)>;

class Sensor {
protected:
    ob_sensor_t  *impl_ = nullptr;
    FrameCallback callback_;

public:
    explicit Sensor(ob_sensor_t *impl) : impl_(impl) {}

    Sensor(Sensor &&sensor) noexcept : impl_(sensor.impl_) {
        sensor.impl_ = nullptr;
    }
    Sensor &operator=(Sensor &&sensor) noexcept {
        if(this != &sensor) {
            ob_error *error = nullptr;
            ob_delete_sensor(impl_, &error);
            Error::handle(&error, false);
            impl_        = sensor.impl_;
            sensor.impl_ = nullptr;
        }
        return *this;
    }
    Sensor(const Sensor &sensor)            = delete;
    Sensor &operator=(const Sensor &sensor) = delete;

    virtual ~Sensor() noexcept {
        ob_error *error = nullptr;
        ob_delete_sensor(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the sensor type.
     *
     * @return OBSensorType The sensor type.
     */
    OBSensorType type() {
        ob_error *error = nullptr;
        auto      type  = ob_sensor_get_type(impl_, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get the list of stream profiles.
     *
     * @return std::shared_ptr<StreamProfileList> The stream profile list.
     */
    std::shared_ptr<StreamProfileList> getStreamProfileList() {
        ob_error *error = nullptr;
        auto      list  = ob_sensor_get_stream_profile_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<StreamProfileList>(list);
    }

    /**
     * @brief Request recommended filters
     * @return OBFilterList list of frame processing block
     */
    std::shared_ptr<OBFilterList> getRecommendedFilters() {
        ob_error *error = nullptr;
        auto      list  = ob_sensor_get_recommended_filter_list(impl_, &error);
        Error::handle(&error);
        return std::make_shared<OBFilterList>(list);
    }

    /**
     * @brief Open a frame data stream and set up a callback.
     *
     * @param streamProfile The stream configuration.
     * @param callback The callback to set when frame data arrives.
     */
    void start(std::shared_ptr<StreamProfile> streamProfile, FrameCallback callback) {
        ob_error *error = nullptr;
        callback_       = std::move(callback);
        ob_sensor_start(impl_, const_cast<ob_stream_profile_t *>(streamProfile->getImpl()), &Sensor::frameCallback, this, &error);
        Error::handle(&error);
    }

    static void frameCallback(const ob_frame *frame, void *userData) {
        auto sensor = static_cast<Sensor *>(userData);
        sensor->callback_(std::make_shared<Frame>(frame));
    }

    /**
     * @brief Stop the stream.
     */
    void stop() {
        ob_error *error = nullptr;
        ob_sensor_stop(impl_, &error);
        Error::handle(&error);
    }
};

class SensorList {
private:
    ob_sensor_list_t *impl_ = nullptr;

public:
    explicit SensorList(ob_sensor_list_t *impl) : impl_(impl) {}

    ~SensorList() noexcept {
        ob_error *error = nullptr;
        ob_delete_sensor_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of sensors.
     *
     * @return uint32_t The number of sensors.
     */
    uint32_t count() {
        ob_error *error = nullptr;
        auto      count = ob_sensor_list_get_sensor_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the type of the specified sensor.
     *
     * @param index The sensor index.
     * @return OBSensorType The sensor type.
     */
    OBSensorType type(uint32_t index) {
        ob_error *error = nullptr;
        auto      type  = ob_sensor_list_get_sensor_type(impl_, index, &error);
        Error::handle(&error);
        return type;
    }

    /**
     * @brief Get a sensor by index number.
     *
     * @param index The sensor index. The range is [0, count-1]. If the index exceeds the range, an exception will be thrown.
     * @return std::shared_ptr<Sensor> The sensor object.
     */
    std::shared_ptr<Sensor> getSensor(uint32_t index) {
        ob_error *error  = nullptr;
        auto      sensor = ob_sensor_list_get_sensor(impl_, index, &error);
        Error::handle(&error);
        return std::make_shared<Sensor>(sensor);
    }

    /**
     * @brief Get a sensor by sensor type.
     *
     * @param sensorType The sensor type to obtain.
     * @return std::shared_ptr<Sensor> A sensor object. If the specified sensor type does not exist, it will return empty.
     */
    std::shared_ptr<Sensor> getSensor(OBSensorType sensorType) {
        ob_error *error  = nullptr;
        auto      sensor = ob_sensor_list_get_sensor_by_type(impl_, sensorType, &error);
        Error::handle(&error);
        return std::make_shared<Sensor>(sensor);
    }
};

/**
 * @brief Class representing a list of FrameProcessingBlock
 */
class OBFilterList {
private:
    ob_filter_list_t *impl_ = nullptr;

public:
    explicit OBFilterList(ob_filter_list_t *impl) : impl_(impl) {}
    ~OBFilterList() noexcept {
        ob_error *error = nullptr;
        ob_delete_filter_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of OBDepthWorkMode FrameProcessingBlock in the list
     *
     * @return uint32_t the number of FrameProcessingBlock objects in the list
     */
    uint32_t count() {
        ob_error *error = nullptr;
        auto      count = ob_filter_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the Filter object at the specified index
     *
     * @param index the index of the target Filter object
     * @return the Filter object at the specified index
     */
    std::shared_ptr<Filter> getFilter(uint32_t index) {
        ob_error *error  = nullptr;
        auto      filter = ob_filter_list_get_filter(impl_, index, &error);
        Error::handle(&error);
        return std::make_shared<Filter>(filter);
    }
};

}  // namespace ob
