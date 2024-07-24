#include "FemtoBoltDevice.hpp"
#include "ObPal.hpp"
// #include "FemtoBoltPropertyPort.hpp"
#include "environment/EnvConfig.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "sensor/video/VideoSensor.hpp"
#include "sensor/video/DisparityBasedSensor.hpp"
#include "sensor/imu/ImuStreamer.hpp"
#include "sensor/imu/AccelSensor.hpp"
#include "sensor/imu/GyroSensor.hpp"
#include "usb/uvc/UvcDevicePort.hpp"
#include "FilterFactory.hpp"

#include "component/metadata/FrameMetadataParserContainer.hpp"
#include "component/timestamp/GlobalTimestampFitter.hpp"
#include "component/property/VendorPropertyAccessor.hpp"
#include "component/property/UvcPropertyAccessor.hpp"
#include "component/property/PropertyServer.hpp"
#include "component/property/CommonPropertyAccessors.hpp"
#include "component/property/FilterPropertyAccessors.hpp"
#include "component/monitor/DeviceMonitor.hpp"

#include "publicfilters/FormatConverterProcess.hpp"
#include "publicfilters/IMUCorrector.hpp"

#include "FemtoBoltAlgParamManager.hpp"
#include "gemini330/G330DeviceSyncConfigurator.hpp"
//  #include "G330SensorStreamStrategy.hpp"

namespace libobsensor {
FemtoBoltDevice::FemtoBoltDevice(const std::shared_ptr<const IDeviceEnumInfo> &info) : DeviceBase(info) {
    init();
}

FemtoBoltDevice::~FemtoBoltDevice() noexcept {}

void FemtoBoltDevice::init() {
    initSensorList();
    initProperties();

    fetchDeviceInfo();

    auto algParamManager = std::make_shared<FemtoBoltAlgParamManager>(this);
    registerComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER, algParamManager);

    static const std::vector<OBMultiDeviceSyncMode> supportedSyncModes     = { OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN, OB_MULTI_DEVICE_SYNC_MODE_STANDALONE,
                                                                               OB_MULTI_DEVICE_SYNC_MODE_PRIMARY, OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED };
    auto                                            deviceSyncConfigurator = std::make_shared<G330DeviceSyncConfigurator>(this, supportedSyncModes);
    registerComponent(OB_DEV_COMPONENT_DEVICE_SYNC_CONFIGURATOR, deviceSyncConfigurator);
}

void FemtoBoltDevice::fetchDeviceInfo() {
    auto propServer = getPropertyServer();

    // auto version                      = propServer->getStructureDataT<OBVersionInfo>(OB_STRUCT_VERSION);
    deviceInfo_ = std::make_shared<DeviceInfo>();
    // deviceInfo_->name_                = version.deviceName;
    // deviceInfo_->fwVersion_           = version.firmwareVersion;
    // deviceInfo_->deviceSn_            = version.serialNumber;
    // deviceInfo_->asicName_            = version.depthChip;
    // deviceInfo_->hwVersion_           = version.hardwareVersion;
    // deviceInfo_->type_                = static_cast<uint16_t>(version.deviceType);
    // deviceInfo_->supportedSdkVersion_ = version.sdkVersion;
    deviceInfo_->pid_            = enumInfo_->getPid();
    deviceInfo_->vid_            = enumInfo_->getVid();
    deviceInfo_->uid_            = enumInfo_->getUid();
    deviceInfo_->connectionType_ = enumInfo_->getConnectionType();

    if(deviceInfo_->name_.find("Orbbec") == std::string::npos) {
        deviceInfo_->name_ = "Orbbec " + deviceInfo_->name_;
    }
}

void FemtoBoltDevice::initSensorStreamProfile(std::shared_ptr<ISensor> sensor) {
    auto streamProfile = StreamProfileFactory::getDefaultStreamProfileFromEnvConfig(deviceInfo_->name_, sensor->getSensorType());
    if(streamProfile) {
        sensor->updateDefaultStreamProfile(streamProfile);
    }

    // // bind params: extrinsics, intrinsics, etc.
    auto profiles = sensor->getStreamProfileList();
    {
        auto algParamManager = getComponentT<FemtoBoltAlgParamManager>(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
        algParamManager->bindStreamProfileParams(profiles);
    }

    auto sensorType = sensor->getSensorType();
    LOG_INFO("Sensor {} created! Found {} stream profiles.", sensorType, profiles.size());
    for(auto &profile: profiles) {
        LOG_INFO(" - {}", profile);
    }

    // sensor->registerStreamStateChangedCallback([this](OBStreamState state, const std::shared_ptr<const StreamProfile> &sp) {
    //     auto streamStrategy = getComponentT<ISensorStreamStrategy>(OB_DEV_COMPONENT_SENSOR_STREAM_STRATEGY);
    //     if(state == STREAM_STATE_STARTING) {
    //         streamStrategy->markStreamStarted(sp);
    //     }
    //     else if(state == STREAM_STATE_STOPED) {
    //         streamStrategy->markStreamStopped(sp);
    //     }
    // });
}

void FemtoBoltDevice::initSensorList() {
    registerComponent(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY, [this]() {
        std::shared_ptr<FrameProcessorFactory> factory;
        TRY_EXECUTE({ factory = std::make_shared<FrameProcessorFactory>(this); })
        return factory;
    });
    auto        pal                = ObPal::getInstance();
    const auto &sourcePortInfoList = enumInfo_->getSourcePortInfoList();
    auto depthPortInfoIter = std::find_if(sourcePortInfoList.begin(), sourcePortInfoList.end(), [](const std::shared_ptr<const SourcePortInfo> &portInfo) {
        return portInfo->portType == SOURCE_PORT_USB_UVC && std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->infIndex == 2;
    });

    if(depthPortInfoIter != sourcePortInfoList.end()) {
        auto depthPortInfo = *depthPortInfoIter;
        registerComponent(
            OB_DEV_COMPONENT_DEPTH_SENSOR,
            [this, depthPortInfo]() {
                auto pal    = ObPal::getInstance();
                auto port   = pal->getSourcePort(depthPortInfo);
                auto sensor = std::make_shared<DisparityBasedSensor>(this, OB_SENSOR_DEPTH, port);

                sensor->setFrameMetadataParserContainer(depthMdParserContainer_);
                sensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

                auto frameProcessor = getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR, false);
                if(frameProcessor) {
                    sensor->setFrameProcessor(frameProcessor.get());
                }

                initSensorStreamProfile(sensor);
                return sensor;
            },
            true);

        registerSensorPortInfo(OB_SENSOR_DEPTH, depthPortInfo);

        registerComponent(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR, [this]() {
            auto factory = getComponentT<FrameProcessorFactory>(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);

            auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_DEPTH);
            return frameProcessor;
        });
    }

    auto colorPortInfoIter = std::find_if(sourcePortInfoList.begin(), sourcePortInfoList.end(), [](const std::shared_ptr<const SourcePortInfo> &portInfo) {
        return portInfo->portType == SOURCE_PORT_USB_UVC && std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->infIndex == 0;
    });

    if(colorPortInfoIter != sourcePortInfoList.end()) {
        auto colorPortInfo = *colorPortInfoIter;
        registerComponent(
            OB_DEV_COMPONENT_COLOR_SENSOR,
            [this, colorPortInfo]() {
                auto pal    = ObPal::getInstance();
                auto port   = pal->getSourcePort(colorPortInfo);
                auto sensor = std::make_shared<VideoSensor>(this, OB_SENSOR_COLOR, port);

                std::vector<FormatFilterConfig> formatFilterConfigs = {
                    { FormatFilterPolicy::REMOVE, OB_FORMAT_NV12, OB_FORMAT_ANY, nullptr },
                    { FormatFilterPolicy::REPLACE, OB_FORMAT_BYR2, OB_FORMAT_RW16, nullptr },
                };

                auto formatConverter = getSensorFrameFilter("FormatConverter", OB_SENSOR_COLOR, false);
                if(formatConverter) {
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_RGB, formatConverter });
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_RGBA, formatConverter });
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_BGR, formatConverter });
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_BGRA, formatConverter });
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_Y16, formatConverter });
                    formatFilterConfigs.push_back({ FormatFilterPolicy::ADD, OB_FORMAT_YUYV, OB_FORMAT_Y8, formatConverter });
                }

                sensor->updateFormatFilterConfig(formatFilterConfigs);
                sensor->setFrameMetadataParserContainer(colorMdParserContainer_);
                sensor->setFrameTimestampCalculator(videoFrameTimestampCalculator_);

                auto frameProcessor = getComponentT<FrameProcessor>(OB_DEV_COMPONENT_COLOR_FRAME_PROCESSOR, false);
                if(frameProcessor) {
                    sensor->setFrameProcessor(frameProcessor.get());
                }

                initSensorStreamProfile(sensor);

                return sensor;
            },
            true);
        registerSensorPortInfo(OB_SENSOR_COLOR, colorPortInfo);

        registerComponent(OB_DEV_COMPONENT_COLOR_FRAME_PROCESSOR, [this]() {
            auto factory        = getComponentT<FrameProcessorFactory>(OB_DEV_COMPONENT_FRAME_PROCESSOR_FACTORY);
            auto frameProcessor = factory->createFrameProcessor(OB_SENSOR_COLOR);
            return frameProcessor;
        });
    }

    auto imuPortInfoIter = std::find_if(sourcePortInfoList.begin(), sourcePortInfoList.end(),
                                        [](const std::shared_ptr<const SourcePortInfo> &portInfo) { return portInfo->portType == SOURCE_PORT_USB_HID; });

    if(imuPortInfoIter != sourcePortInfoList.end()) {
        auto imuPortInfo = *imuPortInfoIter;
        registerComponent(OB_DEV_COMPONENT_IMU_STREAMER, [this, imuPortInfo]() {
            // the gyro and accel are both on the same port and share the same filter
            auto pal                = ObPal::getInstance();
            auto port               = pal->getSourcePort(imuPortInfo);
            auto imuCorrectorFilter = getSensorFrameFilter("IMUCorrector", OB_SENSOR_ACCEL);
            if(!imuCorrectorFilter) {
                throw not_implemented_exception("Cannot find IMU correcter filter!");
            }
            auto dataStreamPort = std::dynamic_pointer_cast<IDataStreamPort>(port);
            auto imuStreamer    = std::make_shared<ImuStreamer>(this, dataStreamPort, imuCorrectorFilter);
            return imuStreamer;
        });

        registerComponent(
            OB_DEV_COMPONENT_ACCEL_SENSOR,
            [this, imuPortInfo]() {
                auto pal                  = ObPal::getInstance();
                auto port                 = pal->getSourcePort(imuPortInfo);
                auto imuStreamer          = getComponentT<ImuStreamer>(OB_DEV_COMPONENT_IMU_STREAMER);
                auto imuStreamerSharedPtr = imuStreamer.get();
                auto sensor               = std::make_shared<AccelSensor>(this, port, imuStreamerSharedPtr);

                initSensorStreamProfile(sensor);

                return sensor;
            },
            true);
        registerSensorPortInfo(OB_SENSOR_ACCEL, imuPortInfo);

        registerComponent(
            OB_DEV_COMPONENT_GYRO_SENSOR,
            [this, imuPortInfo]() {
                auto pal                  = ObPal::getInstance();
                auto port                 = pal->getSourcePort(imuPortInfo);
                auto imuStreamer          = getComponentT<ImuStreamer>(OB_DEV_COMPONENT_IMU_STREAMER);
                auto imuStreamerSharedPtr = imuStreamer.get();
                auto sensor               = std::make_shared<GyroSensor>(this, port, imuStreamerSharedPtr);

                initSensorStreamProfile(sensor);

                return sensor;
            },
            true);
        registerSensorPortInfo(OB_SENSOR_GYRO, imuPortInfo);
    }
}

void FemtoBoltDevice::initProperties() {
    auto propertyServer = std::make_shared<PropertyServer>(this);
    //  auto femtoBoltPropertyAccessor = std::make_shared<FemtoBoltPropertyPort>(this);

    auto sensors = getSensorTypeList();
    for(auto &sensor: sensors) {
        auto pal            = ObPal::getInstance();
        auto sourcePortInfo = getSensorPortInfo(sensor);
        if(sensor == OB_SENSOR_COLOR) {
            auto uvcPropertyAccessor = std::make_shared<LazyPropertyAccessor>([this, &sourcePortInfo]() {
                auto pal      = ObPal::getInstance();
                auto port     = pal->getSourcePort(sourcePortInfo);
                auto accessor = std::make_shared<UvcPropertyAccessor>(port);
                return accessor;
            });

            propertyServer->registerProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_EXPOSURE_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_GAIN_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_SATURATION_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_BRIGHTNESS_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_SHARPNESS_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_CONTRAST_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, "rw", "rw", uvcPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_COLOR_HDR_BOOL, "rw", "rw", uvcPropertyAccessor);
        }
        else if(sensor == OB_SENSOR_DEPTH) {
            auto vendorPropertyAccessor = std::make_shared<LazyPropertyExtensionAccessor>([this, &sourcePortInfo]() {
                auto pal      = ObPal::getInstance();
                auto port     = pal->getSourcePort(sourcePortInfo);
                auto accessor = std::make_shared<VendorPropertyAccessor>(this, port);
                return accessor;
            });

            propertyServer->registerProperty(OB_PROP_TIMESTAMP_OFFSET_INT, "rw", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_INDICATOR_LIGHT_BOOL, "rw", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_HEARTBEAT_BOOL, "rw", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_USB_POWER_STATE_INT, "r", "r", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_DC_POWER_STATE_INT, "r", "r", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_BOOT_INTO_RECOVERY_MODE_BOOL, "w", "w", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_TIMER_RESET_ENABLE_BOOL, "rw", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, "w", "w", vendorPropertyAccessor);
            //  propertyServer->registerProperty(OB_PROP_DEVICE_IQ_DEBUG_BOOL, "", "rw", vendorPropertyPort);
            propertyServer->registerProperty(OB_STRUCT_DEVICE_TIME, "rw", "rw", vendorPropertyAccessor);
            //  propertyServer->registerProperty(OB_STRUCT_LED_CONTROL, "", "w", vendorPropertyPort);
            propertyServer->registerProperty(OB_RAW_DATA_CAMERA_CALIB_JSON_FILE, "r", "r", vendorPropertyAccessor);
            //  propertyServer->registerProperty(OB_RAW_DATA_MCU_UPGRADE_FILE, "rw", "rw", vendorPropertyPort);
            //  propertyServer->registerProperty(OB_RAW_DATA_HARDWARE_ALIGN_PARAM, "rw", "rw", vendorPropertyPort);

            propertyServer->registerProperty(OB_PROP_DEPTH_EXPOSURE_INT, "r", "r", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_PROP_SWITCH_IR_MODE_INT, "rw", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_RAW_DATA_ALIGN_CALIB_PARAM, "", "r", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST, "", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_RAW_DATA_IMU_CALIB_PARAM, "", "rw", vendorPropertyAccessor);
            propertyServer->registerProperty(OB_STRUCT_VERSION, "", "r", vendorPropertyAccessor);
        }
        else if(sensor == OB_SENSOR_ACCEL) {
            auto imuCorrectorFilter = getSensorFrameFilter("IMUCorrector", sensor);
            if(imuCorrectorFilter) {
                auto filterStateProperty = std::make_shared<FilterStatePropertyAccessor>(imuCorrectorFilter);
                propertyServer->registerProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL, "rw", "rw", filterStateProperty);
            }
        }
        else if(sensor == OB_SENSOR_GYRO) {
            auto imuCorrectorFilter = getSensorFrameFilter("IMUCorrector", sensor);
            if(imuCorrectorFilter) {
                auto filterStateProperty = std::make_shared<FilterStatePropertyAccessor>(imuCorrectorFilter);
                propertyServer->registerProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL, "rw", "rw", filterStateProperty);
            }
        }
    }

    // propertyServer->aliasProperty(OB_PROP_IR_EXPOSURE_INT, OB_PROP_DEPTH_EXPOSURE_INT);

    registerComponent(OB_DEV_COMPONENT_PROPERTY_SERVER, propertyServer, true);
}

std::vector<std::shared_ptr<IFilter>> FemtoBoltDevice::createRecommendedPostProcessingFilters(OBSensorType type) {
    std::vector<std::shared_ptr<IFilter>> filters;
    if(type == OB_SENSOR_COLOR) {
        return {};
    }
    return filters;
}
}  // namespace libobsensor