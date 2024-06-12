#pragma once
#include "core/device/component/DeviceSyncConfigurator.hpp"

namespace libobsensor {
namespace g330 {
class G330DeviceSyncConfigurator : public DeviceSyncConfigurator {
public:
    G330DeviceSyncConfigurator(std::shared_ptr<PropertyManager> propertyManager, std::vector<OBMultiDeviceSyncMode> supportedSyncModes)
        : DeviceSyncConfigurator(propertyManager, {}), supportedSyncModes_(supportedSyncModes) {}
    virtual ~G330DeviceSyncConfigurator() = default;

    virtual OBDeviceSyncConfig getSyncConfig() override;
    virtual void               setSyncConfig(const OBDeviceSyncConfig &deviceSyncConfig) override;

    virtual OBMultiDeviceSyncConfig getSyncConfigNew();
    virtual void                    setSyncConfigNew(const OBMultiDeviceSyncConfig &deviceSyncConfig) override;

    uint16_t getSupportedSyncModeBitmap() override;

private:
    OBMultiDeviceSyncConfig            currentMultiDevSyncConfig_;
    std::vector<OBMultiDeviceSyncMode> supportedSyncModes_;
};
}  // namespace g330
}  // namespace libobsensor