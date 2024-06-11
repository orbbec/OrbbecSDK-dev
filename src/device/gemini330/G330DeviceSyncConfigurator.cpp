#include "G330DeviceSyncConfigurator.hpp"
#include "common/exception/ObException.hpp"

namespace libobsensor {
namespace g2r {

OBDeviceSyncConfig G330DeviceSyncConfigurator::getSyncConfig() {
    throw libobsensor::unsupported_operation_exception("Current device does not support this operation!");
    return OBDeviceSyncConfig();
}
void G330DeviceSyncConfigurator::setSyncConfig(const OBDeviceSyncConfig &deviceSyncConfig) {
    throw libobsensor::unsupported_operation_exception("Current device does not support this operation!");
}

#pragma pack(1)
typedef struct {
    uint32_t syncMode;  // ob_multi_device_sync_mode
    int      depthDelayUs;
    int      colorDelayUs;
    int      trigger2ImageDelayUs;
    bool     triggerOutEnable;
    int      triggerOutDelayUs;
    int      framesPerTrigger;
} OBMultiDeviceSyncConfigInternal;
#pragma pack()

OBMultiDeviceSyncConfig G330DeviceSyncConfigurator::getSyncConfigNew() {
    if(isSyncConfigInit_) {
        return currentMultiDevSyncConfig_;
    }

    if(propertyManager_->isPropertySupported(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, OB_PERMISSION_READ)) {
        auto                 accessor = propertyManager_->getPropertyAccessor(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, OB_PERMISSION_READ);
        std::vector<uint8_t> recv;
        recv.resize(sizeof(OBMultiDeviceSyncConfigInternal));

        TRY_EXECUTE(accessor->getStructData([&](OBCmdVersion cmdVersion, uint8_t *data, uint32_t dataSize) {
            memcpy(recv.data(), data, sizeof(OBMultiDeviceSyncConfigInternal));  // datasize 会少一字节，因为传输时是 half word
        }));

        OBMultiDeviceSyncConfigInternal *configInternal = (OBMultiDeviceSyncConfigInternal *)recv.data();
        currentMultiDevSyncConfig_.syncMode             = (OBMultiDeviceSyncMode)configInternal->syncMode;
        if(currentMultiDevSyncConfig_.syncMode == OB_MULTI_DEVICE_SYNC_MODE_SECONDARY) {
            currentMultiDevSyncConfig_.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
        }
        currentMultiDevSyncConfig_.depthDelayUs         = configInternal->depthDelayUs;
        currentMultiDevSyncConfig_.colorDelayUs         = configInternal->colorDelayUs;
        currentMultiDevSyncConfig_.trigger2ImageDelayUs = configInternal->trigger2ImageDelayUs;
        currentMultiDevSyncConfig_.triggerOutEnable     = configInternal->triggerOutEnable;
        currentMultiDevSyncConfig_.triggerOutDelayUs    = configInternal->triggerOutDelayUs;
        currentMultiDevSyncConfig_.framesPerTrigger     = configInternal->framesPerTrigger;
    }

    isSyncConfigInit_ = true;
    return currentMultiDevSyncConfig_;
}

void G330DeviceSyncConfigurator::setSyncConfigNew(const OBMultiDeviceSyncConfig &deviceSyncConfig) {
    if(0 == memcmp(&currentMultiDevSyncConfig_, &deviceSyncConfig, sizeof(OBMultiDeviceSyncConfig))) {
        LOG_INFO("New sync config is same as current device sync config, the upgrade process would not execute!");
        return;
    }
    if(propertyManager_->isPropertySupported(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, OB_PERMISSION_WRITE)) {
        auto accessor           = propertyManager_->getPropertyAccessor(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, OB_PERMISSION_WRITE);
        auto internalConfig     = OBMultiDeviceSyncConfigInternal();
        internalConfig.syncMode = deviceSyncConfig.syncMode;
        if(deviceSyncConfig.syncMode == 0) {
            internalConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
        }
        else if(deviceSyncConfig.syncMode == OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED) {
            internalConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
        }
        internalConfig.depthDelayUs         = deviceSyncConfig.depthDelayUs;
        internalConfig.colorDelayUs         = deviceSyncConfig.colorDelayUs;
        internalConfig.trigger2ImageDelayUs = deviceSyncConfig.trigger2ImageDelayUs;
        internalConfig.triggerOutEnable     = deviceSyncConfig.triggerOutEnable;
        internalConfig.triggerOutDelayUs    = deviceSyncConfig.triggerOutDelayUs;
        internalConfig.framesPerTrigger     = deviceSyncConfig.framesPerTrigger;
        TRY_EXECUTE(accessor->setStructData<OBMultiDeviceSyncConfigInternal>(internalConfig));
    }
    currentMultiDevSyncConfig_ = deviceSyncConfig;
    isSyncConfigInit_          = true;
}

uint16_t G330DeviceSyncConfigurator::getSupportedSyncModeBitmap() {
    uint16_t supportedSyncModeBitmap = 0;
    for(auto &mode: supportedSyncModes_) {
        supportedSyncModeBitmap |= mode;
    }
    return supportedSyncModeBitmap;
}

}  // namespace g2r
}  // namespace libobsensor