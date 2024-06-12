#pragma once
#include "core/device/bootloader/BootDevice.hpp"

namespace libobsensor {
namespace g330 {
class G330BootDevice : public AbstractDevice {
public:
    G330BootDevice(std::shared_ptr<ObPal> obPal, const std::shared_ptr<DeviceInfo> info);
    ~G330BootDevice() noexcept override;

    std::shared_ptr<DeviceInfo> getDeviceInfo() override;

    void deviceUpgrade(std::string filePath, DeviceUpgradeCallback upgradeCallback, bool async) override;
    void deviceUpgrade(const char *fileData, uint32_t fileSize, DeviceUpgradeCallback upgradeCallback, bool async) override;

private:
    void createSensor(OBSensorType sensorType) override {
        throw libobsensor::invalid_value_exception("Create sensor failed! Unsupported sensor type!");
    }
    void initSensorMap() override{};
    void createCommand() override{};
    void initPropertyList() override {}
    void initDepthProcessParam() override {}

private:
    std::thread upgradeThread_;

    uint16_t devModePid_;
};
}  // namespace g330
}  // namespace libobsensor