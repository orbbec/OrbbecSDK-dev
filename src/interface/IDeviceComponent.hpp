#pragma once
#include <mutex>
#include <memory>

#include "utils/Utils.hpp"
#include "exception/ObException.hpp"

namespace libobsensor {
class IDevice;

class IDeviceComponent {
public:
    virtual ~IDeviceComponent()       = default;
    virtual IDevice *getOwner() const = 0;
};

typedef std::unique_lock<std::recursive_timed_mutex> DeviceComponentLock;

template <typename T> class DeviceComponentPtr {
public:
    DeviceComponentPtr(std::shared_ptr<T> ptr, DeviceComponentLock &&lock) : ptr_(ptr), lock_(std::move(lock)) {}
    DeviceComponentPtr(std::shared_ptr<T> ptr) : ptr_(ptr), lock_() {}
    DeviceComponentPtr() : ptr_(), lock_() {}

    // copy constructor and assignment operator are deleted to avoid accidental copies of the lock
    DeviceComponentPtr(const DeviceComponentPtr &other)            = delete;
    DeviceComponentPtr &operator=(const DeviceComponentPtr &other) = delete;

    DeviceComponentPtr(DeviceComponentPtr &&other)            = default;
    DeviceComponentPtr &operator=(DeviceComponentPtr &&other) = default;

    T *operator->() const {
        if(ptr_ == nullptr) {
            throw std::runtime_error("DeviceComponentPtr is nullptr");
        }
        return ptr_.get();
    }

    operator bool() const {
        return ptr_ != nullptr;
    }

    void reset() {
        ptr_.reset();
        lock_ = DeviceComponentLock();
    }

    template <typename U> DeviceComponentPtr<U> as() {
        auto uPtr = std::dynamic_pointer_cast<U>(ptr_);
        if(uPtr == nullptr) {
            throw invalid_value_exception(utils::string::to_string() << "DeviceComponentPtr is not of type " << typeid(U).name());
        }
        ptr_ = nullptr;
        return DeviceComponentPtr<U>(uPtr, std::move(lock_));
    }

    std::shared_ptr<T> get() const {
        return ptr_;
    }

private:
    std::shared_ptr<T>  ptr_;
    DeviceComponentLock lock_;
};

#define OB_DEV_COMPONENT_PROP_ACCESSOR "PropertyAccessor"
#define OB_DEV_COMPONENT_DEPTH_SENSOR "DepthSensor"
#define OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR "DepthFrameProcessor"
#define OB_DEV_COMPONENT_IR_SENSOR "InfraredSensor"
#define OB_DEV_COMPONENT_IR_FRAME_PROCESSOR "InfraredFrameProcessor"
#define OB_DEV_COMPONENT_LEFT_IR_SENSOR "LeftInfraredSensor"
#define OB_DEV_COMPONENT_LEFT_IR_FRAME_PROCESSOR "LeftInfraredFrameProcessor"
#define OB_DEV_COMPONENT_RIGHT_IR_SENSOR "RightInfraredSensor"
#define OB_DEV_COMPONENT_RIGHT_IR_FRAME_PROCESSOR "RightInfraredFrameProcessor"
#define OB_DEV_COMPONENT_COLOR_SENSOR "ColorSensor"
#define OB_DEV_COMPONENT_COLOR_FRAME_PROCESSOR "ColorFrameProcessor"
#define OB_DEV_COMPONENT_GYRO_SENSOR "GyroSensor"
#define OB_DEV_COMPONENT_ACCEL_SENSOR "AccelSensor"
#define OB_DEV_COMPONENT_IMU_STREAMER "ImuStreamer"
#define OB_DEV_COMPONENT_SENSOR_STREAM_STRATEGY "SensorStreamStrategy"
#define OB_DEV_COMPONENT_GLOBAL_TIMESTAMP_FITTER "GlobalTimestampFitter"
#define OB_DEV_COMPONENT_PRESET_MANAGER "PresetManager"
#define OB_DEV_COMPONENT_ALG_PARAM_MANAGER "AlgorithmParameterManager"
#define OB_DEV_COMPONENT_DEPTH_ALG_MODE_MANAGER "DepthAlgorithmModeManager"
#define OB_DEV_COMPONENT_DEVICE_SYNC_CONFIGURATOR "DeviceSyncConfigurator"
#define OB_DEV_COMPONENT_DEVICE_MONITOR "DeviceMonitor"
#define OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY "FrameProcessorFactory"
#define OB_DEV_COMPONENT_COMMAND_PORT "CommandPort"

}  // namespace libobsensor