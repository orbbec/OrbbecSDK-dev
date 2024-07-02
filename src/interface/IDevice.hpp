#pragma once
#include "libobsensor/h/ObTypes.h"
#include "IProperty.hpp"
#include "ISensor.hpp"
#include "IFilter.hpp"
#include "IPresetManager.hpp"
#include "IDeviceComponent.hpp"

namespace libobsensor {

struct DeviceInfo {
    // identifier of the device
    int         pid_ = 0;
    int         vid_ = 0;
    std::string uid_;  // Unique identifier of the port the device is connected to (pal specific)
    std::string name_;

    std::string connectionType_;  // Device connection type
    uint16_t    type_ = 0;        // 0: Monocular disparity based; 1: Binocular disparity based; 2: tof
    std::string fwVersion_;
    std::string hwVersion_;
    std::string supportedSdkVersion_;
    std::string asicName_;
    std::string deviceSn_;
};

typedef std::function<void(OBDeviceState state, const char *message)>                    DeviceStateChangedCallback;
typedef std::function<void(OBFwUpdateState state, const char *message, uint8_t percent)> DeviceFwUpdateCallback;

class IDevice {
public:
    virtual ~IDevice() = default;

    virtual void init() = 0;

    virtual std::shared_ptr<const DeviceInfo> getInfo() const                              = 0;
    virtual const std::string                &getExtensionInfo(const std::string &infoKey) = 0;

    virtual bool                                  isComponentExists(const std::string &name) const                     = 0;
    virtual DeviceComponentPtr<IDeviceComponent>  getComponent(const std::string &name, bool throwExIfNotFound = true) = 0;
    virtual DeviceComponentPtr<IPropertyAccessor> getPropertyAccessor()                                                = 0;
    virtual DeviceComponentPtr<ISensor>           getSensor(OBSensorType type)                                         = 0;

    virtual std::vector<OBSensorType>             getSensorTypeList() const                                 = 0;
    virtual std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) = 0;

    virtual void enableHeadBeat(bool enable) = 0;

    virtual OBDeviceState getDeviceState()                                                       = 0;
    virtual int           registerDeviceStateChangeCallback(DeviceStateChangedCallback callback) = 0;
    virtual void          unregisterDeviceStateChangeCallback(int index)                         = 0;

    virtual void reboot()     = 0;
    virtual void deactivate() = 0;

    virtual void updateFirmware(const std::vector<uint8_t> &firmware, DeviceFwUpdateCallback updateCallback, bool async) = 0;

    // for debug and vendor specific purpose
    virtual const std::vector<uint8_t> &sendAndReceiveData(const std::vector<uint8_t> &data) = 0;
};

}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif
struct ob_device_t {
    std::shared_ptr<libobsensor::IDevice> device;
};

struct ob_device_info_t {
    std::shared_ptr<const libobsensor::DeviceInfo> info;
};

#ifdef __cplusplus
}
#endif