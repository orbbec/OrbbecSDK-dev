#include "G330DeviceSyncConfigurator.hpp"
#include "exception/ObException.hpp"
#include "InternalTypes.hpp"

namespace libobsensor {

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

G330DeviceSyncConfigurator::G330DeviceSyncConfigurator(IDevice *owner, const std::vector<OBMultiDeviceSyncMode> &supportedSyncModes)
    : supportedSyncModes_(supportedSyncModes), DeviceComponentBase(owner), isSyncConfigInit_(false) {}

OBMultiDeviceSyncConfig G330DeviceSyncConfigurator::getSyncConfig() {
    if(isSyncConfigInit_) {
        return currentMultiDevSyncConfig_;
    }
    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();
    auto configInternal   = propertyAccessor->getStructureDataProtoV1_1_T<OBMultiDeviceSyncConfigInternal, 1>(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG);

    currentMultiDevSyncConfig_.syncMode = (OBMultiDeviceSyncMode)configInternal.syncMode;
    if(currentMultiDevSyncConfig_.syncMode == OB_MULTI_DEVICE_SYNC_MODE_SECONDARY) {
        currentMultiDevSyncConfig_.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
    }
    currentMultiDevSyncConfig_.depthDelayUs         = configInternal.depthDelayUs;
    currentMultiDevSyncConfig_.colorDelayUs         = configInternal.colorDelayUs;
    currentMultiDevSyncConfig_.trigger2ImageDelayUs = configInternal.trigger2ImageDelayUs;
    currentMultiDevSyncConfig_.triggerOutEnable     = configInternal.triggerOutEnable;
    currentMultiDevSyncConfig_.triggerOutDelayUs    = configInternal.triggerOutDelayUs;
    currentMultiDevSyncConfig_.framesPerTrigger     = configInternal.framesPerTrigger;

    isSyncConfigInit_ = true;
    return currentMultiDevSyncConfig_;
}

void G330DeviceSyncConfigurator::setSyncConfig(const OBMultiDeviceSyncConfig &deviceSyncConfig) {
    if(isSyncConfigInit_ && 0 == memcmp(&currentMultiDevSyncConfig_, &deviceSyncConfig, sizeof(OBMultiDeviceSyncConfig))) {
        LOG_DEBUG("New sync config is same as current device sync config, the upgrade process would not execute!");
        return;
    }

    OBMultiDeviceSyncConfigInternal internalConfig;
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

    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();
    propertyAccessor->setStructureDataProtoV1_1_T<OBMultiDeviceSyncConfigInternal, 1>(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, internalConfig);

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

void G330DeviceSyncConfigurator::triggerCapture() {
    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();
    propertyAccessor->setPropertyValueT(OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL, true);
}

void G330DeviceSyncConfigurator::setTimestampResetConfig(const OBDeviceTimestampResetConfig &timestampResetConfig) {
    if(isTimestampResetConfigInit_ && 0 == memcmp(&currentTimestampResetConfig_, &timestampResetConfig, sizeof(OBDeviceTimestampResetConfig))) {
        LOG_DEBUG("New timestamp reset config is same as current device timestamp reset config, the upgrade process would not execute!");
        return;
    }

    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();
    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_ENABLE_BOOL, PROP_OP_WRITE, PROP_ACCESS_INTERNAL)) {
        propertyAccessor->setPropertyValueT(OB_PROP_TIMER_RESET_ENABLE_BOOL, timestampResetConfig.enable);
    }
    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, PROP_OP_WRITE, PROP_ACCESS_INTERNAL)) {
        propertyAccessor->setPropertyValueT(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, timestampResetConfig.timestamp_reset_signal_output_enable);
    }
    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, PROP_OP_WRITE, PROP_ACCESS_INTERNAL)) {
        propertyAccessor->setPropertyValueT(OB_PROP_TIMER_RESET_DELAY_US_INT, timestampResetConfig.timestamp_reset_delay_us);
    }

    currentTimestampResetConfig_ = timestampResetConfig;
    isTimestampResetConfigInit_  = true;
}

OBDeviceTimestampResetConfig G330DeviceSyncConfigurator::getTimestampResetConfig() {
    if(isTimestampResetConfigInit_) {
        return currentTimestampResetConfig_;
    }

    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();

    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_ENABLE_BOOL, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
        currentTimestampResetConfig_.enable = propertyAccessor->getPropertyValueT<bool>(OB_PROP_TIMER_RESET_ENABLE_BOOL);
    }
    else {
        currentTimestampResetConfig_.enable = true;
    }

    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
        currentTimestampResetConfig_.timestamp_reset_signal_output_enable =
            propertyAccessor->getPropertyValueT<bool>(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL);
    }
    else {
        currentTimestampResetConfig_.timestamp_reset_signal_output_enable = true;
    }

    if(propertyAccessor->checkProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
        currentTimestampResetConfig_.timestamp_reset_delay_us = propertyAccessor->getPropertyValueT<int>(OB_PROP_TIMER_RESET_DELAY_US_INT);
    }

    isTimestampResetConfigInit_ = true;
    return currentTimestampResetConfig_;
}

void G330DeviceSyncConfigurator::timestampReset() {
    auto owner            = getOwner();
    auto propertyAccessor = owner->getPropertyAccessor();
    propertyAccessor->setPropertyValueT(OB_PROP_TIMER_RESET_ENABLE_BOOL, true);
}

void G330DeviceSyncConfigurator::timerSyncWithHost() {
    const uint32_t MINI_HOST_DEVICE_TIME_DIFF = 1;  // ms
    const uint32_t MAX_REPEAT_TIME            = 10;
    uint8_t        repeated                   = 0;
    uint64_t       rtt                        = 0;

    while(repeated < MAX_REPEAT_TIME) {
        {
            auto owner            = getOwner();
            auto propertyAccessor = owner->getPropertyAccessor();

            auto         tsp    = utils::getNowTimesMs();
            OBDeviceTime devTsp = { tsp, 0 };
            propertyAccessor->setStructureDataT<OBDeviceTime>(OB_STRUCT_DEVICE_TIME, devTsp);
            uint64_t after = utils::getNowTimesMs();
            rtt            = after - tsp;
        }

        {
            auto owner            = getOwner();
            auto propertyAccessor = owner->getPropertyAccessor();
            auto devTsp           = propertyAccessor->getStructureDataT<OBDeviceTime>(OB_STRUCT_DEVICE_TIME);
            auto now              = utils::getNowTimesMs();
            auto nowDev           = devTsp.time;
            auto diff             = now - nowDev - rtt / 2;
            if(diff > 0xFFFFFFFF) {
                int64_t adjustedDiff = (now & 0xFFFFFFFF) - (nowDev & 0xFFFFFFFF) - (rtt & 0xFFFFFFFF) / 2;
                diff                 = adjustedDiff > 0 ? adjustedDiff : -adjustedDiff;
            }
            if(diff <= ((double)MINI_HOST_DEVICE_TIME_DIFF)) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        repeated++;
    }

    if(repeated >= MAX_REPEAT_TIME) {
        throw io_exception(utils::string::to_string() << "syncDeviceTime failed after retry " << repeated << " times, rtt=" << rtt);
    }
}

}  // namespace libobsensor