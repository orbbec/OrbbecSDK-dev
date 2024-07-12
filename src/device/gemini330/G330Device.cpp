#include "G330Device.hpp"
#include "ObPal.hpp"
#include "InternalTypes.hpp"
#include "DevicePids.hpp"
#include "utils/Utils.hpp"

#include "environment/EnvConfig.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "sensor/video/VideoSensor.hpp"
#include "sensor/video/DisparityBasedSensor.hpp"
#include "sensor/motion/MotionStreamer.hpp"
#include "sensor/motion/AccelSensor.hpp"
#include "sensor/motion/GyroSensor.hpp"
#include "usb/uvc/UvcDevicePort.hpp"
#include "filter/FilterFactory.hpp"

#include "filter/public_filters/FormatConverterProcess.hpp"
#include "filter/public_filters/FrameIMUCorrectProcess.hpp"

#include "component/metadata/FrameMetadataParserContainer.hpp"
#include "component/timestamp/GlobalTimestampFitter.hpp"
#include "component/property/VendorPropertyPort.hpp"
#include "component/property/UvcPropertyPort.hpp"
#include "component/property/PropertyAccessor.hpp"
#include "component/property/CommonPropertyPorts.hpp"
#include "component/monitor/DeviceMonitor.hpp"
#include "G330MetadataParser.hpp"
#include "G330MetadataTypes.hpp"
#include "G330TimestampCalculator.hpp"
#include "G330DeviceSyncConfigurator.hpp"
#include "G330AlgParamManager.hpp"
#include "G330PresetManager.hpp"
#include "G330DepthAlgModeManager.hpp"
#include "G330SensorStreamStrategy.hpp"

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

        // add prefix "Orbbec " to device name if it doesn't have it
        if(deviceInfo_->name_.find("Orbbec") == std::string::npos) {
            deviceInfo_->name_ = "Orbbec " + deviceInfo_->name_;
        }
    }

    auto globalTimestampFitter = std::make_shared<GlobalTimestampFitter>(this);
    registerComponent(OB_DEV_COMPONENT_GLOBAL_TIMESTAMP_FITTER, globalTimestampFitter);

    // todo: make timestamp calculator as a component
    auto iter = std::find(gG330LPids.begin(), gG330LPids.end(), deviceInfo_->pid_);
    if(iter != gG330LPids.end()) {
        videoFrameTimestampCalculator_ = std::make_shared<G330TimestampCalculator>(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, globalTimestampFitter);
    }
    else {
        videoFrameTimestampCalculator_ = std::make_shared<G330TimestampCalculator>(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, globalTimestampFitter);
    }

    auto algParamManager = std::make_shared<G330AlgParamManager>(this);
    registerComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER, algParamManager);

    auto depthAlgModeManager = std::make_shared<G330DepthAlgModeManager>(this);
    registerComponent(OB_DEV_COMPONENT_DEPTH_ALG_MODE_MANAGER, depthAlgModeManager);

    auto presetManager = std::make_shared<G330PresetManager>(this);
    registerComponent(OB_DEV_COMPONENT_PRESET_MANAGER, presetManager);

    auto sensorStreamStrategy = std::make_shared<G330SensorStreamStrategy>(this);
    registerComponent(OB_DEV_COMPONENT_SENSOR_STREAM_STRATEGY, sensorStreamStrategy);

    static const std::vector<OBMultiDeviceSyncMode> supportedSyncModes = {
        OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN,         OB_MULTI_DEVICE_SYNC_MODE_STANDALONE,          OB_MULTI_DEVICE_SYNC_MODE_PRIMARY,
        OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED, OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING, OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING
    };
    auto deviceSyncConfigurator = std::make_shared<G330DeviceSyncConfigurator>(this, supportedSyncModes);
    registerComponent(OB_DEV_COMPONENT_DEVICE_SYNC_CONFIGURATOR, deviceSyncConfigurator);

    // lazy create components
    registerComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY, [this]() {
        std::shared_ptr<FrameProcessorFactory> factory;
        TRY_EXECUTE({ factory = std::make_shared<FrameProcessorFactory>(this); })
        return factory;
    });

    registerComponent(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR, [this]() {
        auto comp           = getComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);
        auto factory        = comp.as<FrameProcessorFactory>();
        auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_DEPTH);
        return frameProcessor;
    });

    registerComponent(OB_DEV_COMPONENT_COLOR_FRAME_PROCESSOR, [this]() {
        auto comp           = getComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);
        auto factory        = comp.as<FrameProcessorFactory>();
        auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_COLOR);
        return frameProcessor;
    });

    registerComponent(OB_DEV_COMPONENT_LEFT_IR_FRAME_PROCESSOR, [this]() {
        auto comp           = getComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);
        auto factory        = comp.as<FrameProcessorFactory>();
        auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_IR_LEFT);
        return frameProcessor;
    });

    registerComponent(OB_DEV_COMPONENT_RIGHT_IR_FRAME_PROCESSOR, [this]() {
        auto comp           = getComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);
        auto factory        = comp.as<FrameProcessorFactory>();
        auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_IR_RIGHT);
        return frameProcessor;
    });
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
    auto propertyAccessor = std::make_shared<PropertyAccessor>(this);
    for(auto &sensor: sensors_) {
        auto &sourcePort = sensor.second.backend;
        // todo: lazy create source port
        if(sensor.first == OB_SENSOR_COLOR) {
            auto uvcPropertyPort = std::make_shared<UvcPropertyPort>(sourcePort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_EXPOSURE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_GAIN_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_SATURATION_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_BRIGHTNESS_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_SHARPNESS_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_CONTRAST_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_HUE_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_GAMMA_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, "rw", "rw", uvcPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT, "rw", "rw", uvcPropertyPort);
        }
        else if(sensor.first == OB_SENSOR_DEPTH) {
            auto uvcPropertyPort = std::make_shared<UvcPropertyPort>(sourcePort);
            propertyAccessor->registerProperty(OB_PROP_DEPTH_GAIN_INT, "rw", "rw", uvcPropertyPort);

            auto vendorPropertyPort = std::make_shared<VendorPropertyPort>(sourcePort);
            propertyAccessor->registerProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DEPTH_EXPOSURE_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LDP_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LASER_CONTROL_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LASER_ALWAYS_ON_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LASER_ON_OFF_PATTERN_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_TEMPERATURE_COMPENSATION_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LDP_STATUS_BOOL, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LDP_MEASURE_DISTANCE_INT, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_TIMER_RESET_DELAY_US_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL, "w", "w", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_CAPTURE_IMAGE_FRAME_NUMBER_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_VERSION, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEVICE_TEMPERATURE, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEVICE_TIME, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEVICE_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_ASIC_SERIAL_NUMBER, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_DEPTH_CALIB_PARAM, "", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_ALIGN_CALIB_PARAM, "", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST, "", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_BASELINE_CALIBRATION_PARAM, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEPTH_HDR_CONFIG, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_COLOR_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEPTH_AE_ROI, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_IMU_CALIB_PARAM, "", "rw", vendorPropertyPort);

            // todo: add these properties to the frame processor
            // propertyAccessor->registerProperty(OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL, "rw", "rw", vendorPropertyPort);

            propertyAccessor->registerProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_HEARTBEAT_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_GPM_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_DEVICE_TIME, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_GYRO_ODR_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_ACCEL_ODR_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_ACCEL_SWITCH_BOOL, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_GYRO_SWITCH_BOOL, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_GYRO_FULL_SCALE_INT, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_ACCEL_FULL_SCALE_INT, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_GET_ACCEL_PRESETS_ODR_LIST, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_GET_ACCEL_PRESETS_FULL_SCALE_LIST, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_GET_GYRO_PRESETS_ODR_LIST, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_GET_GYRO_PRESETS_FULL_SCALE_LIST, "", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_IR_BRIGHTNESS_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DEVICE_USB2_REPEAT_IDENTIFY_BOOL, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_DEVICE_EXTENSION_INFORMATION, "", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, "rw", "rw", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_SLAVE_DEVICE_SYNC_STATUS_BOOL, "r", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_PROP_DEVICE_RESET_BOOL, "", "w", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_RAW_DATA_DEPTH_ALG_MODE_LIST, "", "r", vendorPropertyPort);
            propertyAccessor->registerProperty(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, "", "rw", vendorPropertyPort);

            auto devMonitor = std::make_shared<DeviceMonitor>(this, sourcePort);
            registerComponent(OB_DEV_COMPONENT_DEVICE_MONITOR, devMonitor);
        }
        else if(sensor.first == OB_SENSOR_ACCEL) {
            auto imuCorrecterFilter = getSensorFrameFilter("IMUCorrecter", sensor.first);
            if(imuCorrecterFilter) {
                auto filter = std::dynamic_pointer_cast<IMUCorrecter>(imuCorrecterFilter);
                propertyAccessor->registerProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL, "rw", "rw", filter);
            }
        }
        else if(sensor.first == OB_SENSOR_GYRO) {
            auto imuCorrecterFilter = getSensorFrameFilter("IMUCorrecter", sensor.first);
            if(imuCorrecterFilter) {
                auto filter = std::dynamic_pointer_cast<IMUCorrecter>(imuCorrecterFilter);
                propertyAccessor->registerProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL, "rw", "rw", filter);
            }
        }
    }

    propertyAccessor->aliasProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    propertyAccessor->aliasProperty(OB_PROP_IR_EXPOSURE_INT, OB_PROP_DEPTH_EXPOSURE_INT);
    propertyAccessor->aliasProperty(OB_PROP_IR_GAIN_INT, OB_PROP_DEPTH_GAIN_INT);

    auto depthFrameProcessorPortWrapper = std::make_shared<DeviceComponentPropertyPortWrapper>(this, OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
    propertyAccessor->registerProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, "rw", "rw", depthFrameProcessorPortWrapper);

    registerComponent(OB_DEV_COMPONENT_PROP_ACCESSOR, propertyAccessor, true);
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

std::shared_ptr<const DeviceInfo> G330Device::getInfo() const {
    return deviceInfo_;
}

const std::string &G330Device::getExtensionInfo(const std::string &infoKey) {
    // todo: implement this
    utils::unusedVar(infoKey);
    static std::string emptyStr;
    return emptyStr;
}

std::vector<OBSensorType> G330Device::getSensorTypeList() const {
    std::vector<OBSensorType> sensorTypes;
    for(auto &sensor: sensors_) {
        sensorTypes.push_back(sensor.first);
    }
    return sensorTypes;
}

bool G330Device::hasAnySensorStreamActivated() const {
    for(auto &sensor: sensors_) {
        if(sensor.second.sensor && sensor.second.sensor->isStreamActivated()) {
            return true;
        }
    }
    return false;
}

std::vector<std::shared_ptr<IFilter>> G330Device::createRecommendedPostProcessingFilters(OBSensorType type) {
    auto filterFactory = FilterFactory::getInstance();
    if(type == OB_SENSOR_DEPTH) {
        std::vector<std::shared_ptr<IFilter>> depthFilterList;

        if(filterFactory->isFilterCreatorExists("DecimationFilter")) {
            auto decimationFilter = filterFactory->createFilter("DecimationFilter");
            depthFilterList.push_back(decimationFilter);
        }

        if(filterFactory->isFilterCreatorExists("HdrMerge")) {
            auto hdrMergeFilter = filterFactory->createFilter("HdrMerge");
            depthFilterList.push_back(hdrMergeFilter);
        }

        if(filterFactory->isFilterCreatorExists("SequenceIdFilter")) {
            auto sequenceIdFilter = filterFactory->createFilter("SequenceIdFilter");
            depthFilterList.push_back(sequenceIdFilter);
        }

        if(filterFactory->isFilterCreatorExists("PixelValueCutOff")) {
            auto pixelValueCutOffFilter = filterFactory->createFilter("PixelValueCutOff");
            depthFilterList.push_back(pixelValueCutOffFilter);
        }

        if(filterFactory->isFilterCreatorExists("NoiseRemovalFilter")) {
            auto noiseFilter = filterFactory->createFilter("NoiseRemovalFilter");
            // max_size, min_diff, width, height
            std::vector<std::string> params = { "80", "256", "848", "480" };
            noiseFilter->updateConfig(params);
            depthFilterList.push_back(noiseFilter);
        }

        if(filterFactory->isFilterCreatorExists("SpatialAdvancedFilter")) {
            auto spatFilter = filterFactory->createFilter("SpatialAdvancedFilter");
            // magnitude, alpha, disp_diff, radius
            std::vector<std::string> params = { "1", "0.5", "160", "1" };
            spatFilter->updateConfig(params);
            depthFilterList.push_back(spatFilter);
        }

        if(filterFactory->isFilterCreatorExists("TemporalFilter")) {
            auto tempFilter = filterFactory->createFilter("TemporalFilter");
            // diff_scale, weight
            std::vector<std::string> params = { "0.1", "0.4" };
            tempFilter->updateConfig(params);
            depthFilterList.push_back(tempFilter);
        }

        if(filterFactory->isFilterCreatorExists("HoleFillingFilter")) {
            auto hfFilter = filterFactory->createFilter("HoleFillingFilter");
            depthFilterList.push_back(hfFilter);
        }

        if(filterFactory->isFilterCreatorExists("DisparityTransform")) {
            auto dtFilter = filterFactory->createFilter("DisparityTransform");
            depthFilterList.push_back(dtFilter);
        }

        for(size_t i = 0; i < depthFilterList.size(); i++) {
            auto filter = depthFilterList[i];
            if(filter->getName() != "DisparityTransform") {
                filter->enable(false);
            }
        }
        return depthFilterList;
    }
    else if(type == OB_SENSOR_COLOR) {
        std::vector<std::shared_ptr<IFilter>> colorFilterList;
        if(filterFactory->isFilterCreatorExists("DecimationFilter")) {
            auto decimationFilter = filterFactory->createFilter("DecimationFilter");
            decimationFilter->enable(false);
            colorFilterList.push_back(decimationFilter);
        }
        return colorFilterList;
    }

    return {};
}

DeviceComponentPtr<ISensor> G330Device::getSensor(OBSensorType sensorType) {
    auto resLock = tryLockResource();
    auto iter    = sensors_.find(sensorType);
    if(iter == sensors_.end()) {
        throw invalid_value_exception("Sensor not supported!");
    }

    if(iter->second.sensor) {
        return DeviceComponentPtr<ISensor>(iter->second.sensor, std::move(resLock));
    }

    // create
    if(sensorType == OB_SENSOR_ACCEL || sensorType == OB_SENSOR_GYRO) {
        auto                            dataStreamPort     = std::dynamic_pointer_cast<IDataStreamPort>(iter->second.backend);
        std::shared_ptr<MotionStreamer> motionStreamer     = nullptr;
        auto                            imuCorrecterFilter = getSensorFrameFilter("IMUCorrecter", sensorType);
        if(imuCorrecterFilter) {
            motionStreamer = std::make_shared<MotionStreamer>(dataStreamPort, imuCorrecterFilter);
        }

        auto accelIter           = sensors_.find(OB_SENSOR_ACCEL);
        auto accelSensor         = std::make_shared<AccelSensor>(this, accelIter->second.backend, motionStreamer);
        accelIter->second.sensor = accelSensor;

        // bind params: extrinsics, intrinsics, etc.
        auto accelProfiles = accelSensor->getStreamProfileList();
        {
            auto algParamManager = getComponentT<G330AlgParamManager>(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
            algParamManager->bindStreamProfileParams(accelProfiles);
        }

        LOG_INFO("Sensor {} created! Found {} stream profiles.", OB_SENSOR_ACCEL, accelProfiles.size());
        for(auto &profile: accelProfiles) {
            LOG_INFO(" - {}", profile);
        }

        auto gyroIter           = sensors_.find(OB_SENSOR_GYRO);
        auto gyroSensor         = std::make_shared<GyroSensor>(this, gyroIter->second.backend, motionStreamer);
        gyroIter->second.sensor = gyroSensor;

        // bind params: extrinsics, intrinsics, etc.
        auto gyroProfiles = gyroSensor->getStreamProfileList();
        {
            auto algParamManager = getComponentT<G330AlgParamManager>(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
            algParamManager->bindStreamProfileParams(gyroProfiles);
        }

        LOG_INFO("Sensor {} created! Found {} stream profiles.", OB_SENSOR_GYRO, gyroProfiles.size());
        for(auto &profile: gyroProfiles) {
            LOG_INFO(" - {}", profile);
        }
    }
    else {  // type == OB_SENSOR_COLOR || type == OB_SENSOR_DEPTH || type == OB_SENSOR_IR_LEFT || type == OB_SENSOR_IR_RIGHT
        std::shared_ptr<VideoSensor> videoSensor;
        if(sensorType == OB_SENSOR_DEPTH) {
            videoSensor = std::make_shared<DisparityBasedSensor>(this, sensorType, iter->second.backend);

            videoSensor->updateFormatFilterConfig({ { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },
                                                    { FormatFilterPolicy::REPLACE, OB_FORMAT_Z16, OB_FORMAT_Y16, nullptr } });

            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

            auto comp = getComponent(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR, false);
            if(comp) {
                auto frameProcessor = comp.as<FrameProcessor>();
                videoSensor->setFrameProcessor(frameProcessor.get());
            }
        }
        else if(sensorType == OB_SENSOR_IR_LEFT) {
            videoSensor = std::make_shared<VideoSensor>(this, sensorType, iter->second.backend);

            std::vector<FormatFilterConfig> formatFilterConfigs = {
                { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },  //
                { FormatFilterPolicy::REMOVE, OB_FORMAT_BA81, OB_FORMAT_ANY, nullptr },
                { FormatFilterPolicy::REMOVE, OB_FORMAT_YV12, OB_FORMAT_ANY, nullptr },
                { FormatFilterPolicy::REPLACE, OB_FORMAT_NV12, OB_FORMAT_Y12, nullptr },
            };
            auto formatConverter = getSensorFrameFilter("FrameUnpacker", sensorType, false);
            if(formatConverter) {
                formatFilterConfigs.push_back({ FormatFilterPolicy::REPLACE, OB_FORMAT_NV12, OB_FORMAT_Y16, formatConverter });
            }

            videoSensor->updateFormatFilterConfig(formatFilterConfigs);
            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

            auto comp = getComponent(OB_DEV_COMPONENT_LEFT_IR_FRAME_PROCESSOR, false);
            if(comp) {
                auto frameProcessor = comp.as<FrameProcessor>();
                videoSensor->setFrameProcessor(frameProcessor.get());
            }
        }
        else if(sensorType == OB_SENSOR_IR_RIGHT) {
            videoSensor = std::make_shared<VideoSensor>(this, sensorType, iter->second.backend);

            std::vector<FormatFilterConfig> formatFilterConfigs = {
                { FormatFilterPolicy::REMOVE, OB_FORMAT_Z16, OB_FORMAT_ANY, nullptr },   //
                { FormatFilterPolicy::REMOVE, OB_FORMAT_Y8, OB_FORMAT_ANY, nullptr },    //
                { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },  //
                { FormatFilterPolicy::REMOVE, OB_FORMAT_UYVY, OB_FORMAT_ANY, nullptr },  //
                { FormatFilterPolicy::REPLACE, OB_FORMAT_BA81, OB_FORMAT_Y8, nullptr },  //
                { FormatFilterPolicy::REPLACE, OB_FORMAT_YV12, OB_FORMAT_Y12, nullptr },
            };

            auto formatConverter = getSensorFrameFilter("FrameUnpacker", sensorType, false);
            if(formatConverter) {
                formatFilterConfigs.push_back({ FormatFilterPolicy::REPLACE, OB_FORMAT_YV12, OB_FORMAT_Y16, formatConverter });
            }

            videoSensor->updateFormatFilterConfig(formatFilterConfigs);
            videoSensor->setFrameMetadataParserContainer(depthMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

            auto comp = getComponent(OB_DEV_COMPONENT_RIGHT_IR_FRAME_PROCESSOR, false);
            if(comp) {
                auto frameProcessor = comp.as<FrameProcessor>();
                videoSensor->setFrameProcessor(frameProcessor.get());
            }
        }
        else if(sensorType == OB_SENSOR_COLOR) {
            videoSensor = std::make_shared<VideoSensor>(this, sensorType, iter->second.backend);

            std::vector<FormatFilterConfig> formatFilterConfigs = {
                { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                { FormatFilterPolicy::REPLACE, OB_FORMAT_BYR2, OB_FORMAT_RW16, nullptr },
            };

            auto formatConverter = getSensorFrameFilter("FormatConverter", sensorType, false);
            if(formatConverter) {
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_RGB, formatConverter });
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_RGBA, formatConverter });
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_BGR, formatConverter });
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_BGRA, formatConverter });
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_Y16, formatConverter });
                formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_Y8, formatConverter });
            }

            videoSensor->updateFormatFilterConfig(formatFilterConfigs);
            videoSensor->setFrameMetadataParserContainer(colorMdParserContainer_);
            videoSensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

            auto comp = getComponent(OB_DEV_COMPONENT_COLOR_FRAME_PROCESSOR, false);
            if(comp) {
                auto frameProcessor = comp.as<FrameProcessor>();
                videoSensor->setFrameProcessor(frameProcessor.get());
            }
        }

        auto defaultStreamProfile = StreamProfileFactory::getDefaultStreamProfileFormEnvConfig(deviceInfo_->name_, sensorType);
        if(defaultStreamProfile) {
            videoSensor->updateDefaultStreamProfile(defaultStreamProfile);
        }

        iter->second.sensor = videoSensor;  // store

        // bind params: extrinsics, intrinsics, etc.
        auto profiles = iter->second.sensor->getStreamProfileList();
        {
            auto algParamManager = getComponentT<G330AlgParamManager>(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
            algParamManager->bindStreamProfileParams(profiles);
        }

        LOG_INFO("Sensor {} created! Found {} stream profiles.", sensorType, profiles.size());
        for(auto &profile: profiles) {
            LOG_INFO(" - {}", profile);
        }
    }
    iter->second.sensor->registerStreamStateChangedCallback([this](OBStreamState state, const std::shared_ptr<const StreamProfile> &sp) {
        auto streamStrategy = getComponentT<ISensorStreamStrategy>(OB_DEV_COMPONENT_SENSOR_STREAM_STRATEGY);
        if(state == STREAM_STATE_STARTING) {
            streamStrategy->markStreamStarted(sp);
        }
        else if(state == STREAM_STATE_STOPED) {
            streamStrategy->markStreamStopped(sp);
        }
    });

    return DeviceComponentPtr<ISensor>(iter->second.sensor, std::move(resLock));
}

void G330Device::reboot() {
    auto propAccessor = getPropertyAccessor();
    propAccessor->setPropertyValueT(OB_PROP_DEVICE_RESET_BOOL, true);
    deactivate();
}

void G330Device::deactivate() {
    sensors_.clear();
    DeviceBase::deactivate();
}

void G330Device::updateFirmware(const std::vector<uint8_t> &firmware, DeviceFwUpdateCallback updateCallback, bool async) {
    // todo: implement this
    utils::unusedVar(firmware);
    utils::unusedVar(updateCallback);
    utils::unusedVar(async);
    throw not_implemented_exception("Not implemented!");
}

}  // namespace libobsensor