#include "G330Device.hpp"
#include "ObPal.hpp"
#include "InternalTypes.hpp"
#include "DevicePids.hpp"

#include "property/VendorPropertyPort.hpp"
#include "property/UvcPropertyPort.hpp"
#include "property/FilterPropertyPort.hpp"
#include "property/PropertyAccessor.hpp"
#include "sensor/video/VideoSensor.hpp"
#include "sensor/video/DisparityBasedSensor.hpp"
#include "sensor/motion/MotionStreamer.hpp"
#include "sensor/motion/AccelSensor.hpp"
#include "sensor/motion/GyroSensor.hpp"
#include "usb/uvc/UvcDevicePort.hpp"
#include "filter/FilterFactory.hpp"
#include "utils/Utils.hpp"
#include "metadata/FrameMetadataParserContainer.hpp"

#include "G330MetadataParser.hpp"
#include "G330MetadataTypes.hpp"
#include "G330TimestampCalculator.hpp"

#include <algorithm>

namespace libobsensor {

constexpr uint8_t INTERFACE_COLOR = 4;
constexpr uint8_t INTERFACE_DEPTH = 0;

G330Device::G330Device(const std::shared_ptr<const IDeviceEnumInfo> &info) : enumInfo_(info) {
    initSensors();
    initProperties();
    initFrameMetadataParserContainer();

    {
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

    DeviceResourceGetter<IPropertyAccessor> propertyAccessorGetter([this]() {
        auto propAccessor = getPropertyAccessor();
        return std::move(propAccessor);
    });
    algParamManager_       = std::make_shared<G330AlgParamManager>(deviceInfo_, propertyAccessorGetter);
    globalTimestampFitter_ = std::make_shared<GlobalTimestampFitter>(propertyAccessorGetter);

    auto iter = std::find(gG330LPids.begin(), gG330LPids.end(), deviceInfo_->pid_);
    if(iter != gG330LPids.end()) {
        videoFrameTimestampCalculator_ = std::make_shared<G330TimestampCalculator>(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, globalTimestampFitter_);
    }
    else {
        videoFrameTimestampCalculator_ = std::make_shared<G330TimestampCalculator>(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, globalTimestampFitter_);
    }
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
            auto vendorPropertyPort = std::make_shared<VendorPropertyPort>(sourcePort);
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
            propertyAccessor_->registerProperty(OB_STRUCT_DEVICE_TIME, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, "", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEVICE_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_ASIC_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_DEPTH_CALIB_PARAM, "", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_ALIGN_CALIB_PARAM, "", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST, "", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_BASELINE_CALIBRATION_PARAM, "r", "r", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEPTH_HDR_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_COLOR_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_STRUCT_DEPTH_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor_->registerProperty(OB_RAW_DATA_IMU_CALIB_PARAM, "", "rw", vendorPropertyPort);

            // todo: add these properties to the frame processor
            // propertyAccessor_->registerProperty(OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL, "rw", "rw", vendorPropertyPort);

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
        else if(sensor.first == OB_SENSOR_ACCEL) {
            auto imuCorrecterFilter       = getSpecifyFilter("IMUCorrecter");
            auto imuCorrecterPropertyPort = std::make_shared<FilterPropertyPort>(imuCorrecterFilter);
            propertyAccessor_->registerProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL, "rw", "rw", imuCorrecterPropertyPort);
        }
        else if(sensor.first == OB_SENSOR_GYRO) {
            auto imuCorrecterFilter       = getSpecifyFilter("IMUCorrecter");
            auto imuCorrecterPropertyPort = std::make_shared<FilterPropertyPort>(imuCorrecterFilter);
            propertyAccessor_->registerProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL, "rw", "rw", imuCorrecterPropertyPort);
        }
    }
    propertyAccessor_->aliasProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    propertyAccessor_->aliasProperty(OB_PROP_IR_EXPOSURE_INT, OB_PROP_DEPTH_EXPOSURE_INT);
    propertyAccessor_->aliasProperty(OB_PROP_IR_GAIN_INT, OB_PROP_DEPTH_GAIN_INT);
}

void G330Device::initFrameMetadataParserContainer() {
    // for depth and left/right ir sensor
    depthMdParserContainer_ = std::make_shared<FrameMetadataParserContainer>();
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_TIMESTAMP, std::make_shared<G330MetadataTimestampParser<G330DepthUvcMetadata>>());
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, std::make_shared<G330MetadataSensorTimestampParser>());
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_FRAME_NUMBER, makeStructureMetadataParser(&G330DepthUvcMetadata::frame_counter));
    // todo: calculate actual fps according exposure and frame rate
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE, makeStructureMetadataParser(&G330DepthUvcMetadata::actual_fps));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAIN, makeStructureMetadataParser(&G330DepthUvcMetadata::gain_level));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE,
                                            makeStructureMetadataParser(&G330CommonUvcMetadata::bitmap_union_0,
                                                                        [](const uint64_t &param) {  //
                                                                            return ((G330ColorUvcMetadata::bitmap_union_0_fields *)&param)->auto_exposure;
                                                                        }));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE, makeStructureMetadataParser(&G330CommonUvcMetadata::exposure));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY, makeStructureMetadataParser(&G330DepthUvcMetadata::exposure_priority));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_POWER, makeStructureMetadataParser(&G330DepthUvcMetadata::laser_power));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_POWER_LEVEL, makeStructureMetadataParser(&G330DepthUvcMetadata::laser_power_level));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_STATUS, makeStructureMetadataParser(&G330DepthUvcMetadata::laser_status));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_LEFT, makeStructureMetadataParser(&G330DepthUvcMetadata::exposure_roi_left));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_TOP, makeStructureMetadataParser(&G330DepthUvcMetadata::exposure_roi_top));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT, makeStructureMetadataParser(&G330DepthUvcMetadata::exposure_roi_right));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM, makeStructureMetadataParser(&G330DepthUvcMetadata::exposure_roi_bottom));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA, makeStructureMetadataParser(&G330DepthUvcMetadata::gpio_input_data));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME, makeStructureMetadataParser(&G330DepthUvcMetadata::sequence_name));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE, makeStructureMetadataParser(&G330DepthUvcMetadata::sequence_size));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX, makeStructureMetadataParser(&G330DepthUvcMetadata::sequence_id));

    // for color sensor
    colorMdParserContainer_ = std::make_shared<FrameMetadataParserContainer>();
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_TIMESTAMP, std::make_shared<G330MetadataTimestampParser<G330ColorUvcMetadata>>());
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP,
                                            std::make_shared<G330ColorMetadataSensorTimestampParser>([](const int64_t &param) { return param * 100; }));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_FRAME_NUMBER, makeStructureMetadataParser(&G330CommonUvcMetadata::frame_counter));
    // todo: calculate actual fps according exposure and frame rate
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE, makeStructureMetadataParser(&G330ColorUvcMetadata::actual_fps));

    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE,
                                            makeStructureMetadataParser(&G330CommonUvcMetadata::bitmap_union_0,
                                                                        [](const int64_t &param) {  //
                                                                            return ((G330ColorUvcMetadata::bitmap_union_0_fields *)&param)->auto_exposure;
                                                                        }));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE, makeStructureMetadataParser(&G330CommonUvcMetadata::exposure));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAIN, makeStructureMetadataParser(&G330ColorUvcMetadata::gain_level));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE, makeStructureMetadataParser(&G330ColorUvcMetadata::auto_white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_WHITE_BALANCE, makeStructureMetadataParser(&G330ColorUvcMetadata::white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE, makeStructureMetadataParser(&G330ColorUvcMetadata::white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_BRIGHTNESS, makeStructureMetadataParser(&G330ColorUvcMetadata::brightness));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_CONTRAST, makeStructureMetadataParser(&G330ColorUvcMetadata::contrast));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SATURATION, makeStructureMetadataParser(&G330ColorUvcMetadata::saturation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SHARPNESS, makeStructureMetadataParser(&G330ColorUvcMetadata::sharpness));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION,
                                            makeStructureMetadataParser(&G330ColorUvcMetadata::backlight_compensation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAMMA, makeStructureMetadataParser(&G330ColorUvcMetadata::gamma));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HUE, makeStructureMetadataParser(&G330ColorUvcMetadata::hue));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY,
                                            makeStructureMetadataParser(&G330ColorUvcMetadata::power_line_frequency));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION,
                                            makeStructureMetadataParser(&G330ColorUvcMetadata::low_light_compensation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_LEFT, makeStructureMetadataParser(&G330ColorUvcMetadata::exposure_roi_left));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_TOP, makeStructureMetadataParser(&G330ColorUvcMetadata::exposure_roi_top));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT, makeStructureMetadataParser(&G330ColorUvcMetadata::exposure_roi_right));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM, makeStructureMetadataParser(&G330ColorUvcMetadata::exposure_roi_bottom));
}

DeviceResourceLock G330Device::tryLockResource() {
    DeviceResourceLock resLock(componentLock_, std::defer_lock);
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

DeviceResourcePtr<IPropertyAccessor> G330Device::getPropertyAccessor() {
    auto resLock = tryLockResource();
    return DeviceResourcePtr<IPropertyAccessor>(propertyAccessor_, std::move(resLock));
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

DeviceResourcePtr<ISensor> G330Device::getSensor(OBSensorType type) {
    auto resLock = tryLockResource();
    auto iter    = sensors_.find(type);
    if(iter == sensors_.end()) {
        throw invalid_value_exception("Sensor not supported!");
    }

    if(iter->second.sensor) {
        return DeviceResourcePtr<ISensor>(iter->second.sensor, std::move(resLock));
    }

    // create
    if(type == OB_SENSOR_ACCEL || type == OB_SENSOR_GYRO) {
        auto                            dataStreamPort     = std::dynamic_pointer_cast<IDataStreamPort>(iter->second.backend);
        std::shared_ptr<MotionStreamer> motionStreamer     = nullptr;
        auto                            imuCorrecterFilter = getSpecifyFilter("IMUCorrecter");
        if(imuCorrecterFilter) {
            motionStreamer = std::make_shared<MotionStreamer>(dataStreamPort, imuCorrecterFilter);  // todo: add data phaser
        }

        auto accelIter           = sensors_.find(OB_SENSOR_ACCEL);
        auto accelSensor         = std::make_shared<AccelSensor>(shared_from_this(), accelIter->second.backend, motionStreamer);
        accelIter->second.sensor = accelSensor;

        auto gyroIter           = sensors_.find(OB_SENSOR_GYRO);
        auto gyroSensor         = std::make_shared<GyroSensor>(shared_from_this(), gyroIter->second.backend, motionStreamer);
        gyroIter->second.sensor = gyroSensor;
    }
    else {  // type == OB_SENSOR_COLOR || type == OB_SENSOR_DEPTH || type == OB_SENSOR_IR_LEFT || type == OB_SENSOR_IR_RIGHT
        std::shared_ptr<VideoSensor> videoSensor;

        if(type == OB_SENSOR_DEPTH) {
            videoSensor         = std::make_shared<DisparityBasedSensor>(shared_from_this(), type, iter->second.backend);
            iter->second.sensor = videoSensor;

            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_Z16, OB_FORMAT_Y16, nullptr } });

            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);
        }
        else if(type == OB_SENSOR_IR_LEFT) {
            videoSensor         = std::make_shared<VideoSensor>(shared_from_this(), type, iter->second.backend);
            iter->second.sensor = videoSensor;
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_NV12, OB_FORMAT_Y12, nullptr } });
            // add Y16 which unpack from NV12
            // todo: implement this
            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);
        }
        else if(type == OB_SENSOR_IR_RIGHT) {
            videoSensor         = std::make_shared<VideoSensor>(shared_from_this(), type, iter->second.backend);
            iter->second.sensor = videoSensor;
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_YV12, OB_FORMAT_Y12, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_BA81, OB_FORMAT_Y8, nullptr } });
            // add Y16 which unpack from YV12
            // todo: implement this
            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);
        }
        else if(type == OB_SENSOR_COLOR) {
            videoSensor         = std::make_shared<VideoSensor>(shared_from_this(), type, iter->second.backend);
            iter->second.sensor = videoSensor;
            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_BYR2, OB_FORMAT_RW16, nullptr } });

            // add RGB, RGBA, BGR, BGRA, Y16, Y8 which convert from YUYV
            // todo: implement this
            videoSensor->setFrameMetadataParserContainer(colorMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);
        }

        // frame preprocessor
        std::shared_ptr<FrameProcessor> frameProcessor = nullptr;
        if(!frameProcessorFactory_) {
            frameProcessorFactory_ = std::make_shared<FrameProcessorFactory>(shared_from_this());
        }
        frameProcessor = frameProcessorFactory_->createFrameProcessor(type);
        if(frameProcessor) {
            videoSensor->setFrameProcessor(frameProcessor);
        }
    }

    // bind params: extrinsics, intrinsics, etc.
    auto profiles = iter->second.sensor->getStreamProfileList();
    algParamManager_->bindStreamProfileParams(profiles);

    // todo: printf streamProfile
    for(auto &profile: profiles) {
        utils::unusedVar(profile);
    }

    return DeviceResourcePtr<ISensor>(iter->second.sensor, std::move(resLock));
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

std::shared_ptr<IFilter> G330Device::getSpecifyFilter(const std::string &name, bool createIfNotExist) {
    auto filterIter = std::find_if(filters_.begin(), filters_.end(), [name](const std::shared_ptr<IFilter> &filter) { return filter->getName() == name; });

    if(filterIter != filters_.end()) {
        return *filterIter;
    }
    else if(!createIfNotExist) {
        return nullptr;
    }

    auto filterFactory = FilterFactory::getInstance();
    auto filter        = filterFactory->createFilter(name);
    filters_.push_back(filter);
    return filter;
}
}  // namespace libobsensor