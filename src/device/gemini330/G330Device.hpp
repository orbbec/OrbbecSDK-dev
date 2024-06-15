#pragma once
#include "IDevice.hpp"
#include "IDeviceEnumerator.hpp"
#include <map>
#include <memory>

namespace libobsensor {

class G330Device : public IDevice, public std::enable_shared_from_this<IDevice> {
public:
    G330Device(const std::shared_ptr<const DeviceEnumInfo> &info);
    virtual ~G330Device() noexcept;

    std::shared_ptr<const DeviceInfo> getInfo() const override;
    const std::string                &getExtensionInfo(const std::string &infoKey) override;

    ResourcePtr<IPropertyAccessor> getPropertyAccessor() override;

    std::vector<OBSensorType>             getSensorTypeList() const override;
    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;
    ResourcePtr<ISensor>                  getSensor(OBSensorType type) override;

    void enableHeadBeat(bool enable) override;

    OBDeviceState getDeviceState() override;
    int           registerDeviceStateChangeCallback(DeviceStateChangedCallback callback) override;
    void          unregisterDeviceStateChangeCallback(int index) override;

    void reboot() override;
    void deactivate() override;

    void updateFirmware(const char *data, uint32_t dataSize, DeviceFwUpdateCallback updateCallback, bool async) override;

    const std::vector<uint8_t> &sendAndReceiveData(const std::vector<uint8_t> &data) override;

private:
    void initSensors();
    void initProperties();

    ResourceLock tryLockResource();

private:
    const std::shared_ptr<const DeviceEnumInfo> enumInfo_;
    std::shared_ptr<DeviceInfo>                 deviceInfo_;
    std::map<std::string, std::string>          extensionInfo_;
    std::shared_ptr<IPropertyAccessor>          propertyAccessor_;
    std::map<OBSensorType, SensorEntry>         sensors_;

    std::recursive_timed_mutex componentLock_;
};
/* #endregion ------------G330Device declare end---------------- */

}  // namespace libobsensor
