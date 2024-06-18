#include "G330Device.hpp"
#include "ObPal.hpp"
#include "InternalTypes.hpp"
#include "property/VendorPropertyPort.hpp"
#include "property/UvcPropertyPort.hpp"
#include "property/PropertyAccessor.hpp"
#include "sensor/video/VideoSensor.hpp"
#include "sensor/motion/MotionStreamer.hpp"
#include "sensor/motion/AccelSensor.hpp"
#include "sensor/motion/GyroSensor.hpp"
#include "usb/uvc/UvcDevicePort.hpp"
#include "filter/FilterFactory.hpp"

#include "utils/Utils.hpp"

#include <algorithm>

namespace libobsensor {

constexpr uint8_t INTERFACE_COLOR = 4;
constexpr uint8_t INTERFACE_DEPTH = 0;

G330Device::G330Device(const std::shared_ptr<const DeviceEnumInfo> &info) : enumInfo_(info) {
    initSensors();
    initProperties();

    auto propAccessor                 = getPropertyAccessor();
    auto version                      = propAccessor->getStructureDataT<OBVersionInfo>(OB_STRUCT_VERSION);
    deviceInfo_                       = std::make_shared<DeviceInfo>();
    deviceInfo_->name_                = version.deviceName;
    deviceInfo_->fwVersion_           = version.firmwareVersion;
    deviceInfo_->deviceSn_            = version.serialNumber;
    deviceInfo_->asicName_            = version.depthChip;
    deviceInfo_->hwVersion_           = version.hardwareVersion;
    deviceInfo_->type_                = static_cast<uint16_t>(version.deviceType);
    deviceInfo_->supportedSdkVersion_ = version.sdkVersion;
    deviceInfo_->pid_                 = enumInfo_->getPid();
    deviceInfo_->vid_                 = enumInfo_->getVid();
    deviceInfo_->uid_                 = enumInfo_->getUid();
    deviceInfo_->connectionType_      = enumInfo_->getConnectionType();
}

G330Device::~G330Device() noexcept {}

void G330Device::initSensors() {
    auto        pal                = ObPal::getInstance();
    const auto &sourcePortInfoList = enumInfo_->getSourcePortInfoList();
    std::for_each(sourcePortInfoList.cbegin(), sourcePortInfoList.cend(), [this, &pal](const std::shared_ptr<const SourcePortInfo> &portInfo) {
        if(portInfo->portType == SOURCE_PORT_USB_UVC) {
            auto port        = pal->createSourcePort(portInfo);
            auto uvcPortInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo);
            if(uvcPortInfo->infIndex == INTERFACE_COLOR) {
                sensors_.insert({ OB_SENSOR_COLOR, { nullptr, port } });
            }
            else if(uvcPortInfo->infIndex == INTERFACE_DEPTH) {
                auto uvcDevicePort = std::dynamic_pointer_cast<UvcDevicePort>(port);
                uvcDevicePort->updateXuUnit(OB_G330_XU_UNIT);
                sensors_.insert({ OB_SENSOR_DEPTH, { nullptr, port } });
                sensors_.insert({ OB_SENSOR_IR_LEFT, { nullptr, port } });
                sensors_.insert({ OB_SENSOR_IR_RIGHT, { nullptr, port } });
            }
        }
        else if(portInfo->portType == SOURCE_PORT_USB_HID) {
            auto port = pal->createSourcePort(portInfo);
            sensors_.insert({ OB_SENSOR_ACCEL, { nullptr, port } });
            sensors_.insert({ OB_SENSOR_GYRO, { nullptr, port } });
        }
    });
}

void G330Device::initProperties() {
    propertyAccessor_ = std::make_shared<PropertyAccessor>();
    for(auto &sensor: sensors_) {
        auto &sourcePort = sensor.second.backend;
        // todo: lazy create source port
        if(sensor.first == OB_SENSOR_COLOR) {
            auto uvcPropertyPort = std::make_shared<UvcPropertyPort>(sourcePort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_EXPOSURE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_GAIN_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_SATURATION_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_BRIGHTNESS_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_SHARPNESS_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_CONTRAST_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_HUE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_GAMMA_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT, "rw", "rw", uvcPropertyPort);
        }
        else if(sensor.first == OB_SENSOR_DEPTH) {
            auto uvcPropertyPort = std::make_shared<UvcPropertyPort>(sourcePort);
            propertyAccessor_->registerProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEPTH_EXPOSURE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEPTH_GAIN_INT, "rw", "rw", uvcPropertyPort);
            // FIXME
            auto vendorPropertyPort = std::make_shared<VendorPropertyPort<0>>(sourcePort);
            propertyAccessor_->registerProperty(OB_PROP_LDP_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LASER_CONTROL_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LASER_ALWAYS_ON_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LASER_ON_OFF_PATTERN_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_TEMPERATURE_COMPENSATION_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LDP_STATUS_BOOL, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LDP_MEASURE_DISTANCE_INT, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL, "w", "w", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_CAPTURE_IMAGE_FRAME_NUMBER_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_VERSION, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEVICE_TEMPERATURE, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEVICE_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_ASIC_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_DEPTH_CALIB_PARAM, "", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_BASELINE_CALIBRATION_PARAM, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEPTH_HDR_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_COLOR_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEPTH_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_IMU_CALIB_PARAM, "", "rw", vendorPropertyPort);

            // todo: add these properties to the frame processor
            // propertyAccessor_->registerProperty(OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL, "rw", "rw", vendorPropertyPort);
            // propertyAccessor_->registerProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL, "rw", "rw", vendorPropertyPort);
            // propertyAccessor_->registerProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL, "rw", "rw", vendorPropertyPort);

            propertyAccessor_->registerProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_HEARTBEAT_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_GPM_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEVICE_TIME, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_GYRO_ODR_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_ACCEL_ODR_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_GYRO_FULL_SCALE_INT, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_ACCEL_FULL_SCALE_INT, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_IR_BRIGHTNESS_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEVICE_USB2_REPEAT_IDENTIFY_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_DEVICE_EXTENSION_INFORMATION, "", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_SLAVE_DEVICE_SYNC_STATUS_BOOL, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_PROP_DEVICE_RESET_BOOL, "", "w", vendorPropertyPort);
        }
    }
    propertyAccessor_->aliasProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    propertyAccessor_->aliasProperty(OB_PROP_IR_EXPOSURE_INT, OB_PROP_DEPTH_EXPOSURE_INT);
    propertyAccessor_->aliasProperty(OB_PROP_IR_GAIN_INT, OB_PROP_DEPTH_GAIN_INT);
}

IDevice::ResourceLock G330Device::tryLockResource() {
    ResourceLock resLock(componentLock_, std::defer_lock);
    if(!resLock.try_lock_for(std::chrono::milliseconds(10000))) {
        throw libobsensor::wrong_api_call_sequence_exception("Resource busy! You can try again later!");
    }
    return resLock;
}

std::shared_ptr<const DeviceInfo> G330Device::getInfo() const {
    return deviceInfo_;
}

const std::string &G330Device::getExtensionInfo(const std::string &infoKey) {
    // todo: implement this
    utils::unusedVar(infoKey);
    static std::string emptyStr;
    return emptyStr;
}

IDevice::ResourcePtr<IPropertyAccessor> G330Device::getPropertyAccessor() {
    auto resLock = tryLockResource();
    return ResourcePtr<IPropertyAccessor>(propertyAccessor_, std::move(resLock));
}

std::vector<OBSensorType> G330Device::getSensorTypeList() const {
    std::vector<OBSensorType> sensorTypes;
    for(auto &sensor: sensors_) {
        sensorTypes.push_back(sensor.first);
    }
    return sensorTypes;
}

std::vector<std::shared_ptr<IFilter>> G330Device::createRecommendedPostProcessingFilters(OBSensorType type) {
    utils::unusedVar(type);
    return {};
}

IDevice::ResourcePtr<ISensor> G330Device::getSensor(OBSensorType type) {
    auto resLock = tryLockResource();
    auto iter    = sensors_.find(type);
    if(iter == sensors_.end()) {
        throw invalid_value_exception("Sensor not supported!");
    }

    if(iter->second.sensor) {
        return ResourcePtr<ISensor>(iter->second.sensor, std::move(resLock));
    }
    // create
    if(type == OB_SENSOR_ACCEL || type == OB_SENSOR_GYRO) {
        auto dataStreamPort = std::dynamic_pointer_cast<IDataStreamPort>(iter->second.backend);
        auto motionStreamer = std::make_shared<MotionStreamer>(dataStreamPort, nullptr);  // todo: add data phaser

        auto accelIter           = sensors_.find(OB_SENSOR_ACCEL);
        auto accelSensor         = std::make_shared<AccelSensor>(shared_from_this(), accelIter->second.backend, motionStreamer);
        accelIter->second.sensor = accelSensor;

        auto gyroIter           = sensors_.find(OB_SENSOR_GYRO);
        auto gyroSensor         = std::make_shared<GyroSensor>(shared_from_this(), gyroIter->second.backend, motionStreamer);
        gyroIter->second.sensor = gyroSensor;
    }
    else {  // type == OB_SENSOR_COLOR || type == OB_SENSOR_DEPTH || type == OB_SENSOR_IR_LEFT || type == OB_SENSOR_IR_RIGHT
        auto videoSensor    = std::make_shared<VideoSensor>(shared_from_this(), type, iter->second.backend);
        iter->second.sensor = videoSensor;

        if(type == OB_SENSOR_DEPTH) {
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_Z16, OB_FORMAT_Y16, nullptr } });
        }
        else if(type == OB_SENSOR_IR_LEFT) {
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_NV12, OB_FORMAT_Y12, nullptr } });
            // add Y16 which unpack from NV12
            // todo: implement this
        }
        else if(type == OB_SENSOR_IR_RIGHT) {
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_YV12, OB_FORMAT_Y12, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_BA81, OB_FORMAT_Y8, nullptr } });
            // add Y16 which unpack from YV12
            // todo: implement this
        }
        else if(type == OB_SENSOR_COLOR) {
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_BYR2, OB_FORMAT_RW16, nullptr } });

            // add RGB, RGBA, BGR, BGRA, Y16, Y8 which convert from YUYV
            // todo: implement this
        }
    }

    auto profiles = iter->second.sensor->getStreamProfileList();
    // todo: bind params to stream profile

    // todo: printf streamProfile
    for(auto &profile: profiles) {
        utils::unusedVar(profile);
    }
    return ResourcePtr<ISensor>(iter->second.sensor, std::move(resLock));
}

void G330Device::enableHeadBeat(bool enable) {
    // todo:implement this
    utils::unusedVar(enable);
}

OBDeviceState G330Device::getDeviceState() {
    // todo: implement this
    return 0;
}
int G330Device::registerDeviceStateChangeCallback(DeviceStateChangedCallback callback) {
    // todo: implement this
    utils::unusedVar(callback);
    return 0;
}
void G330Device::unregisterDeviceStateChangeCallback(int index) {
    // todo: implement this
    utils::unusedVar(index);
}

void G330Device::reboot() {
    auto propAccessor = getPropertyAccessor();
    propAccessor->setPropertyValueT(OB_PROP_DEVICE_RESET_BOOL, true);
    deactivate();
}

void G330Device::deactivate() {
    // todo: implement this
}

void G330Device::updateFirmware(const char *data, uint32_t dataSize, DeviceFwUpdateCallback updateCallback, bool async) {
    // todo: implement this
    utils::unusedVar(data);
    utils::unusedVar(dataSize);
    utils::unusedVar(updateCallback);
    utils::unusedVar(async);
}

const std::vector<uint8_t> &G330Device::sendAndReceiveData(const std::vector<uint8_t> &data) {
    // todo: implement this
    utils::unusedVar(data);
    static std::vector<uint8_t> emptyData;
    return emptyData;
}
}  // namespace libobsensor