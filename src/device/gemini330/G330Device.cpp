#include "G330Device.hpp"
#include "G330Firmware.hpp"
#include "G330MetadataParser.hpp"
#include "G330MetadataTypes.hpp"
#include "G330TimestampConverter.hpp"
#include "G330DeviceSyncConfigurator.hpp"
#include "command/G330HostProtocol.hpp"
#include "core/sensor/motion/GyroSensor.hpp"
#include "core/sensor/motion/AccelSensor.hpp"

#include "common/utility/StringUtils.hpp"
#include "common/logger/Logger.hpp"
#include "common/utility/FileUtils.hpp"

#include "platform/usb/uvc/UvcDevicePort.hpp"

#include "core/Context.hpp"
#include "core/CoreType.hpp"
#include "core/sensor/video/VideoSensor.hpp"
#include "core/command/MX6600VendorCommand.hpp"
#include "core/frame/process/IMUFrameTransformer.hpp"
#include "core/frame/process/HdrMerge.hpp"
#include "core/frame/process/SequenceIdFilter.hpp"
#include "core/frame/process/ThresholdFilter.hpp"
#include "core/frame/process/DisparityTransform.hpp"
#include "core/frame/process/SpatialAdvancedFilter.hpp"
#include "core/frame/process/TemporalFilter.hpp"
#include "core/frame/process/HoleFillingFilter.hpp"
#include "core/frame/process/NoiseRemovalFilter.hpp"
#include "core/frame/process/DecimationFilter.hpp"

#include <stdexcept>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <chrono>
#include <json/json.h>
#include <map>
namespace libobsensor {
namespace g2r {

static const uint8_t INTERFACE_COLOR = 4;
static const uint8_t INTERFACE_DEPTH = 0;

G330Device::G330Device(std::shared_ptr<ObPal> obPal, const std::shared_ptr<DeviceInfo> info) : AbstractDevice(obPal, info) {
    LOG_DEBUG("Gemini2RDevice default constructor ...");
    initPropertyList();
    createCommand();
    initDepthAlgMode();
    initPreset(devEventBus_);
    initSensorMap();
    loadDefaultConfig();
    initDepthProcessParam();
    initFrameMetadataParserContainer();
    initHeartBeatEventListener();

    // init sync configurator
    if(command_ != nullptr) {
        syncConfigurator_ = std::make_shared<G330DeviceSyncConfigurator>(
            propertyManager_,
            std::vector<OBMultiDeviceSyncMode>({ OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN, OB_MULTI_DEVICE_SYNC_MODE_STANDALONE, OB_MULTI_DEVICE_SYNC_MODE_PRIMARY,
                                                 OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED, OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING,
                                                 OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING }));

        if(globalTimestampFitterEnable_) {
            globalTspFitter_ = std::make_shared<GlobalTimestampFitter>(command_);
        }
    }

    updateHeartBeatByXmlConfig();
    auto iter = std::find(gG330LPids.begin(), gG330LPids.end(), deviceInfo_->pid_);
    if(iter != gG330LPids.end()) {
        frameTimeStampMetadataType_ = OB_FRAME_METADATA_TYPE_TIMESTAMP;
    }
    else {
        frameTimeStampMetadataType_ = OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP;
    }

    LOG_INFO("G330Device created! PID:{}, SN:{}, depthMode: {}", deviceInfo_->pid_, deviceInfo_->deviceSn_, currentDepthAlgMode_);
}

G330Device::~G330Device() noexcept {
    if(upgradeThread_.joinable()) {
        upgradeThread_.join();
    }
    LOG_INFO("G330Device destroyed! PID:{}, SN:{}, depthMode: {}", deviceInfo_->pid_, deviceInfo_->deviceSn_, currentDepthAlgMode_);
}

std::shared_ptr<DeviceInfo> G330Device::getDeviceInfo() {
    AbstractDevice::getDeviceInfo();
    if(deviceInfo_->name_.find("Orbbec") == std::string::npos) {
        deviceInfo_->name_ = "Orbbec " + deviceInfo_->name_;
    }
    return deviceInfo_;
}

void G330Device::initPropertyList() {
    OBPropertyPermissionMap supportedPropertyList = {
        PP_PAIR(OB_PROP_LDP_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_LASER_CONTROL_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_LASER_ALWAYS_ON_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_LASER_ON_OFF_PATTERN_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_TEMPERATURE_COMPENSATION_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_MIN_DEPTH_INT, "rw", "rw"),
        // PP_PAIR(OB_PROP_MAX_DEPTH_INT, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_HOLEFILTER_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_LDP_STATUS_BOOL, "r", "r"),
        PP_PAIR(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT, "rw", "rw"),
        PP_PAIR(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_LDP_MEASURE_DISTANCE_INT, "r", "r"),
        PP_PAIR(OB_PROP_LDP_CALIBRATION_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_POWER_VOLTAGE_FLOAT, "", "r"),
        PP_PAIR(OB_PROP_LDP_STATUS_LEVEL_INT, "r", "r"),
        // PP_PAIR(OB_PROP_DEPTH_SOFT_FILTER_BOOL, "rw", "rw"),      // 软件滤波开关
        // PP_PAIR(OB_PROP_DEPTH_MAX_DIFF_INT, "rw", "rw"),          // soft filter maxdiff param
        // PP_PAIR(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, "rw", "rw"),  // soft filter maxSpeckleSize
        // PP_PAIR(OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT, "", "rw"),
        PP_PAIR(OB_PROP_TIMER_RESET_SIGNAL_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_TIMER_RESET_DELAY_US_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL, "w", "w"),
        // PP_PAIR(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL, "", "r"),
        PP_PAIR(OB_PROP_IR_RIGHT_MIRROR_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_CAPTURE_IMAGE_FRAME_NUMBER_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_PID_INT, "", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_EDGE_NOISE_REMOVAL_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_SPATIAL_FAST_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_SPATIAL_MODERATE_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_SPATIAL_ADVANCED_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_HOLE_FILLING_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_TEMPORAL_FILTER_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_FILTER_DUMP_DIFF_LUT_INT, "", "rw"),
        PP_PAIR(OB_STRUCT_VERSION, "", "r"),
        PP_PAIR(OB_STRUCT_DEVICE_TEMPERATURE, "r", "r"),
        // PP_PAIR(OB_STRUCT_TEMP_COMPENSATE_PARAM, "", "rw"),  // 温补系数
        PP_PAIR(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, "rw", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_PRECISION_SUPPORT_LIST, "r", "r"),
        // PP_PAIR(OB_PROP_DEPTH_RECTIFY_MASK_FILTER_CONFIG, "", "rw"),
        PP_PAIR(OB_STRUCT_DEVICE_SERIAL_NUMBER, "r", "rw"),
        PP_PAIR(OB_STRUCT_ASIC_SERIAL_NUMBER, "r", "rw"),
        PP_PAIR(OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG, "rw", "rw"),
        // PP_PAIR(OB_STRUCT_IR_AE_PARAM, "", "rw"),
        // PP_PAIR(OB_STRUCT_PERIPHERAL_ID_INFO, "", "r"),
        PP_PAIR(OB_RAW_DATA_DEPTH_CALIB_PARAM, "", "r"),
        PP_PAIR(OB_STRUCT_BASELINE_CALIBRATION_PARAM, "r", "r"),
        PP_PAIR(OB_STRUCT_LDP_MEASURE_EXTENSION_INFO, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_DDO_CONFIG, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_DDO_CONFIG_DEFAULT, "", "r"),
        // PP_PAIR(OB_STRUCT_DEPTH_NOISE_REMOVAL_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_EDGE_NOISE_REMOVAL_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_SPATIAL_FAST_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_SPATIAL_MODERATE_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_SPATIAL_ADVANCED_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_HOLE_FILLING_FILTER_PARAMS, "", "rw"),
        // PP_PAIR(OB_STRUCT_DEPTH_TEMPORAL_FILTER_PARAMS, "", "rw"),
        PP_PAIR(OB_STRUCT_DEPTH_HDR_CONFIG, "rw", "rw"),
        PP_PAIR(OB_STRUCT_COLOR_AE_ROI, "rw", "rw"),
        PP_PAIR(OB_STRUCT_DEPTH_AE_ROI, "rw", "rw"),
        // PP_PAIR(OB_RAW_DATA_EFFECTIVE_VIDEO_STREAM_PROFILE_LIST, "", "r"),
        // PP_PAIR(OB_RAW_DATA_DEPTH_ALG_MODE_LIST, "r", "r"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_REF_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_RECTIFY_KK_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_IR_RGB_PRE_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_RECTIFY_LUT_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_D2D_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_IMU_PARAM, "", "rw"),
        // PP_PAIR(OB_RAW_DATA_UPDATE_MX6600_FLASH_DATA, "", "rw"),
        PP_PAIR(OB_RAW_DATA_IMU_CALIB_PARAM, "", "rw"),
        PP_PAIR(OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL, "", "rw"),
        PP_PAIR(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_MIRROR_BOOL, "rw", "rw"),
        // PP_PAIR(OB_PROP_IR_CHANNEL_DATA_SOURCE_INT, "rw", "rw"),
        // PP_PAIR(OB_PROP_DEPTH_RM_FILTER_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_DISPARITY_TO_DEPTH_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_DEPTH_MIRROR_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_COLOR_MIRROR_BOOL, "rw", "rw"),
        //        PP_PAIR(OB_PROP_WATCHDOG_BOOL, "rw", "rw"),               // 看门狗功能开关，0: Disable， 1: Enable
        PP_PAIR(OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL, "rw", "rw"),  // 外部信号触发重启功能开关，0: Disable， 1: Enable
        PP_PAIR(OB_PROP_HEARTBEAT_BOOL, "rw", "rw"),              // 心跳监测功能开关，0: Disable， 1: Enable
        PP_PAIR(OB_PROP_GPM_BOOL, "", "rw"),
        PP_PAIR(OB_PROP_COLOR_FLIP_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_COLOR_ROTATE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_FLIP_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_ROTATE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_RIGHT_FLIP_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_RIGHT_ROTATE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_DEPTH_FLIP_BOOL, "rw", "rw"),
        PP_PAIR(OB_PROP_DEPTH_ROTATE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT, "r", "r"),
        PP_PAIR(OB_STRUCT_DEVICE_TIME, "rw", "rw"),
        PP_PAIR(OB_RAW_DATA_DE_IR_RECTIFY_PARAMS, "", "r"),
        PP_PAIR(OB_PROP_DEBUG_LOG_SEVERITY_LEVEL_INT, "", "rw"),  // 设备日志等级
        PP_PAIR_SENSOR(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL),         // 自动曝光
        PP_PAIR_SENSOR(OB_PROP_COLOR_EXPOSURE_INT),               // 曝光调节
        PP_PAIR_SENSOR(OB_PROP_COLOR_GAIN_INT),                   // 增益调节
        PP_PAIR_SENSOR(OB_PROP_COLOR_SATURATION_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL),
        PP_PAIR_SENSOR(OB_PROP_COLOR_WHITE_BALANCE_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_BRIGHTNESS_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_SHARPNESS_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_CONTRAST_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_HUE_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_GAMMA_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT),
        PP_PAIR_SENSOR(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT),
        PP_PAIR_SENSOR(OB_PROP_IR_AUTO_EXPOSURE_BOOL),     // 自动曝光
        PP_PAIR_SENSOR(OB_PROP_IR_EXPOSURE_INT),           // 曝光调节
        PP_PAIR_SENSOR(OB_PROP_IR_GAIN_INT),               // 增益调节
        PP_PAIR_SENSOR(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL),  // 自动曝光
        PP_PAIR_SENSOR(OB_PROP_DEPTH_EXPOSURE_INT),        // 曝光调节
        PP_PAIR_SENSOR(OB_PROP_DEPTH_GAIN_INT),            // 增益调节
        PP_PAIR(OB_PROP_GYRO_ODR_INT, "", "rw"),
        PP_PAIR(OB_PROP_ACCEL_ODR_INT, "", "rw"),
        PP_PAIR(OB_PROP_GYRO_FULL_SCALE_INT, "", "rw"),
        PP_PAIR(OB_PROP_ACCEL_FULL_SCALE_INT, "", "rw"),
        PP_PAIR(OB_PROP_IR_BRIGHTNESS_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_DEVICE_USB2_REPEAT_IDENTIFY_BOOL, "rw", "rw"),
        PP_PAIR(OB_RAW_DATA_DEVICE_EXTENSION_INFORMATION, "r", "r"),  // 扩展信息读取 Json格式
     //   PP_PAIR(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_IR_AE_MAX_EXPOSURE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, "rw", "rw"),
        PP_PAIR(OB_PROP_SLAVE_DEVICE_SYNC_STATUS_BOOL, "", "r"),
    };

    propertyManager_ = std::make_shared<PropertyManager>(supportedPropertyList, [&](OBPropertyID propertyId, OBPermissionType permission) {  //
        return getPropertyAccessor(propertyId, permission);
    });
}

void G330Device::createCommand() {
    LOG_DEBUG("Create command start!");
    for(auto &item: deviceInfo_->sourcePortInfoList_) {
        auto portInfo = std::dynamic_pointer_cast<USBSourcePortInfo>(item);
        if(portInfo->portType == SOURCE_PORT_USB_UVC && portInfo->infIndex == INTERFACE_DEPTH) {
            auto port = obPal_->createSourcePort(portInfo);
            if(port) {
                auto uvcDevicePort = std::dynamic_pointer_cast<UvcDevicePort>(port);
                uvcDevicePort->updateXuUnit(OB_G330_XU_UNIT);
                auto vendorDataPort = std::dynamic_pointer_cast<VendorDataPort>(uvcDevicePort);
                auto hostProtocol   = std::make_shared<G330HostProtocol>(vendorDataPort);
                command_            = std::make_shared<MX6600VendorCommand>(hostProtocol);
#ifdef __ANDROID__
                deviceInfo_->connectionType_ = uvcDevicePort->getUsbConnectType();
#endif

                OBVersionInfo ver;
                uint32_t      len;
                BEGIN_TRY_EXECUTE({
                    auto accessor = getPropertyAccessorForce(OB_STRUCT_VERSION);
                    accessor->getFirmwareData(&ver, &len);
                })
                CATCH_EXCEPTION_AND_EXECUTE(command_ = nullptr)

                uint8_t *data     = nullptr;
                uint16_t dataSize = 0;
                BEGIN_TRY_EXECUTE(auto accessor = getPropertyAccessor(OB_RAW_DATA_DEVICE_EXTENSION_INFORMATION, OB_PERMISSION_READ); accessor->getRawData(
                    [&](OBDataTranState state, OBDataChunk *dataChunk) {
                        if(state == DATA_TRAN_STAT_TRANSFERRING) {
                            if(data == nullptr) {
                                dataSize = dataChunk->fullDataSize;
                                data     = new uint8_t[dataSize];
                            }
                            memcpy(data + dataChunk->offset, dataChunk->data, dataChunk->size);
                        }
                    },
                    false))
                CATCH_EXCEPTION_AND_EXECUTE({ LOG_ERROR("Get extension info failed!"); })

                if(data != nullptr && dataSize > 0) {
                    // 老版本linux开发板固件有bug：data不带'\0'
                    if(data[dataSize - 1] != 0) {
                        uint8_t *newData = new uint8_t[dataSize + 1];
                        memcpy(newData, data, dataSize + 1);
                        delete[] data;
                        data           = newData;
                        data[dataSize] = 0;
                    }
                    std::string extensionInfo((char *)data);
                    deviceInfo_->extensionInfo_ = extensionInfo;
                    LOG_INFO("Json Content: {},dataSize:{}", extensionInfo, dataSize);
                    // parse extensioninfo
                    Json::Value  root;
                    Json::Reader reader;
                    bool         parseResult = reader.parse(extensionInfo, root);
                    if(!parseResult) {
                        LOG_ERROR("parse extensioninfo failed");
                    }
                    else {
                        std::map<std::string, std::string> dataMap;
                        Json::Value                        extensionInfos = root["ExtensionInfo"];
                        for(auto const &key: extensionInfos.getMemberNames()) {
                            dataMap[key] = extensionInfos[key].asString();
                            LOG_INFO("key: {},value:{}", key, extensionInfos[key].asString());
                            if(key == "IspFwVer") {
                                ispVersion_ = extensionInfos[key].asString();
                            }
                        }

                        for(const auto &pair: dataMap) {
                            std::cout << pair.first << " : " << pair.second << std::endl;
                        }
                    }
                }
                else {
                    LOG_ERROR("Get ExtensionInfo Data is Null!");
                }
                delete[] data;
                data = nullptr;
            }
            break;
        }
    }

    if(command_ == nullptr) {
        throw std::runtime_error("Create vendor command failed! Device no response or bad connection.");
    }
    LOG_DEBUG("Create command done!");
}

void G330Device::initHeartBeatEventListener() {
    const uint32_t HEART_BEAT_PERIOD = 3000;  // ms
    if(!stateChangedCallback_) {
        stateChangedCallback_ = [&](OBDeviceState state, const char *msg) {
            if(destroy_) {
                throw libobsensor::libobsensor_exception("object is destory", OB_EXCEPTION_TYPE_INVALID_VALUE);
            }
            std::lock_guard<std::mutex> lock(deviceStateChangedCBListMutex_);
            for(auto &cb: deviceStateChangedCBList_) {
                cb.second(state, msg);
            }
        };
    }

    std::function<void(OBPropertyID, OBPropertyValue, OBPermissionType)> onPropertyValueUpdate =
        [&, HEART_BEAT_PERIOD](OBPropertyID propertyId, OBPropertyValue value, OBPermissionType permissionType) {
            switch(propertyId) {
            case OB_PROP_HEARTBEAT_BOOL: {
                if(value.intValue == 0) {
                    command_->disableHeartBeat();
                }
                else if(value.intValue == 1) {
                    command_->enableHeartBeat(HEART_BEAT_PERIOD, stateChangedCallback_);
                }
            } break;

            default:
                break;
            }
        };
    devEventBus_->listenEvent("PropertyValueUpdate", onPropertyValueUpdate);

    if(command_->getHeartBeatEnable()) {
        command_->enableHeartBeat(HEART_BEAT_PERIOD, stateChangedCallback_);
    }

    LOG_DEBUG("Create command done!");
}

void G330Device::initFrameMetadataParserContainer() {
    // for depth and left/right ir sensor
    depthMdParserContainer_ = std::make_shared<IMetadataParserContainer>();
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_TIMESTAMP, std::make_shared<G330MetadataTimestampParser<g2r_depth_uvc_metadata_t>>());
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP, std::make_shared<G330MetadataSensorTimestampParser>());
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_FRAME_NUMBER, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::frame_counter));
    // todo: calculate actual fps according exposure and frame rate
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::actual_fps));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAIN, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::gain_level));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE,
                                            makeStructureMetadataParser(&g2r_common_uvc_metadata_t::bitmap_union_0,
                                                                        [](const uint64_t &param) {  //
                                                                            return ((g2r_color_uvc_metadata_t::bitmap_union_0_fields *)&param)->auto_exposure;
                                                                        }));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE, makeStructureMetadataParser(&g2r_common_uvc_metadata_t::exposure));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY,
                                            makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::exposure_priority));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_POWER, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::laser_power));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_POWER_LEVEL,
                                            makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::laser_power_level));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LASER_STATUS, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::laser_status));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_LEFT, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::exposure_roi_left));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_TOP, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::exposure_roi_top));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::exposure_roi_right));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::exposure_roi_bottom));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::gpio_input_data));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::sequence_name));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::sequence_size));
    depthMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX, makeStructureMetadataParser(&g2r_depth_uvc_metadata_t::sequence_id));

    // for color sensor
    colorMdParserContainer_ = std::make_shared<IMetadataParserContainer>();
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_TIMESTAMP, std::make_shared<G330MetadataTimestampParser<g2r_color_uvc_metadata_t>>());
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP,
                                            std::make_shared<G330ColorMetadataSensorTimestampParser>([](const int64_t &param) { return param * 100; }));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_FRAME_NUMBER, makeStructureMetadataParser(&g2r_common_uvc_metadata_t::frame_counter));
    // todo: calculate actual fps according exposure and frame rate
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::actual_fps));

    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE,
                                            makeStructureMetadataParser(&g2r_common_uvc_metadata_t::bitmap_union_0,
                                                                        [](const int64_t &param) {  //
                                                                            return ((g2r_color_uvc_metadata_t::bitmap_union_0_fields *)&param)->auto_exposure;
                                                                        }));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_EXPOSURE, makeStructureMetadataParser(&g2r_common_uvc_metadata_t::exposure));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAIN, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::gain_level));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE,
                                            makeStructureMetadataParser(&g2r_color_uvc_metadata_t::auto_white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_WHITE_BALANCE, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::white_balance));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_BRIGHTNESS, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::brightness));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_CONTRAST, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::contrast));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SATURATION, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::saturation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_SHARPNESS, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::sharpness));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION,
                                            makeStructureMetadataParser(&g2r_color_uvc_metadata_t::backlight_compensation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_GAMMA, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::gamma));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_HUE, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::hue));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY,
                                            makeStructureMetadataParser(&g2r_color_uvc_metadata_t::power_line_frequency));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION,
                                            makeStructureMetadataParser(&g2r_color_uvc_metadata_t::low_light_compensation));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_LEFT, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::exposure_roi_left));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_TOP, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::exposure_roi_top));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::exposure_roi_right));
    colorMdParserContainer_->registerParser(OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM, makeStructureMetadataParser(&g2r_color_uvc_metadata_t::exposure_roi_bottom));
}

void libobsensor::g2r::G330Device::updateHeartBeatByXmlConfig() {
    try {
        const int       defHeartbeat = 0;
        OBPropertyValue value;
        value.intValue = defHeartbeat;
        auto xmlConfig = Context::getInstance()->getXmlConfig();
        if(xmlConfig) {
            bool        ret           = false;
            std::string deviceNameTxt = deviceInfo_->name_;
            // remove space
            deviceNameTxt.erase(std::remove_if(deviceNameTxt.begin(), deviceNameTxt.end(), isspace), deviceNameTxt.end());
            std::string heartBeatFieldTxt = "Device." + deviceNameTxt + ".DefaultHeartBeat";
            ret                           = xmlConfig->getIntValue(heartBeatFieldTxt, value.intValue);
        }

        auto propertyAccessor = getPropertyAccessorForce(OB_PROP_HEARTBEAT_BOOL);
        if(propertyAccessor) {
            propertyAccessor->setPropertyValue(value);
        }
    }
    catch(libobsensor_exception &error) {
        LOG_ERROR("enable heart beat failed. {0}, exception type: {1}", error.get_message(), error.get_exception_type());
    }
}

void G330Device::initDepthAlgMode() {
    memset(&currentDepthAlgMode_, 0, sizeof(currentDepthAlgMode_));
    currentDepthAlgMode_ = requestCurrentDepthAglMode();
}

void G330Device::initSensorMap() {
    LOG_DEBUG("init sensor map start!");

    for(const auto &item: deviceInfo_->sourcePortInfoList_) {
        if(item->portType == SOURCE_PORT_USB_UVC) {
            auto portInfo = std::dynamic_pointer_cast<USBSourcePortInfo>(item);
            if(portInfo->infIndex == INTERFACE_COLOR) {
                sensorEntryList_.insert({ OB_SENSOR_COLOR, SensorEntry(portInfo) });
            }
            else if(portInfo->infIndex == INTERFACE_DEPTH) {
                sensorEntryList_.insert({ OB_SENSOR_DEPTH, SensorEntry(portInfo) });
                sensorEntryList_.insert({ OB_SENSOR_IR_LEFT, SensorEntry(portInfo) });
                sensorEntryList_.insert({ OB_SENSOR_IR_RIGHT, SensorEntry(portInfo) });
            }
        }
    }

    for(const auto &portInfo: deviceInfo_->sourcePortInfoList_) {
        if(portInfo->portType == SOURCE_PORT_USB_HID) {
            sensorEntryList_.insert({ OB_SENSOR_ACCEL, SensorEntry(portInfo) });
            sensorEntryList_.insert({ OB_SENSOR_GYRO, SensorEntry(portInfo) });
        }
    }
    LOG_DEBUG("init sensor map done!");
}

void G330Device::createSensor(OBSensorType sensorType) {
    switch(sensorType) {
    case OB_SENSOR_DEPTH:
        createDepthSensor();
        break;
    case OB_SENSOR_IR_LEFT:
        createLeftIrSensor();
        break;
    case OB_SENSOR_IR_RIGHT:
        createRightIrSensor();
        break;
    case OB_SENSOR_COLOR:
        createColorSensor();
        break;
    case OB_SENSOR_ACCEL:
        createAccelSensor();
        break;
    case OB_SENSOR_GYRO:
        createGyroSensor();
        break;
    default:
        throw libobsensor::invalid_value_exception("Create sensor failed! Unsupported sensor type!");
        break;
    }
}

void G330Device::createDepthSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_DEPTH);
    if(sensorIter != sensorEntryList_.end() && !sensorIter->second.sensor) {
        auto &sensorEntry = sensorIter->second;
        auto  port        = obPal_->createSourcePort(sensorEntry.sourcePortInfo);

        std::vector<FrameProcessingBlockConfig> processBlockTypeList;
        processBlockTypeList = {
            { "Disparity2DepthConverter", !hwD2DEnable_ },  //
            //{ "NoiseRemovalFilter", false },
            //{ "SpatialAdvancedFilter", false },
            //{ "TemporalFilter", false },
            //{ "HoleFillingFilter", false },
            //{ "ThresholdFilter", false },
            { "FrameMirror", false },
            { "FrameFlip", false },
            { "FrameRotate", false },
            { "D2CFilter", false },
            { "PostProcessFilter", false },
        };

        if(!sensorStartStrategy_) {
            sensorStartStrategy_ = std::make_shared<G330SensorStartStrategy>(shared_from_this());
        }

        auto frameProcessor                         = std::make_shared<DepthFrameProcessor>(processBlockTypeList, deviceDepthParam_, devEventBus_, nullptr);
        sensorEntry.backend.frameProcessor          = frameProcessor;
        sensorEntry.backend.devCmd                  = command_;
        sensorEntry.backend.eventBus                = devEventBus_;
        sensorEntry.backend.sourcePort              = port;
        sensorEntry.backend.deviceSyncConfigurator  = syncConfigurator_;
        sensorEntry.backend.frameTimestampConverter = std::make_shared<G330TimestampConverter>(frameTimeStampMetadataType_, globalTspFitter_);
        sensorEntry.backend.frameMetadataParserContainer = depthMdParserContainer_;
        sensorEntry.backend.algParamManager              = algParamManager_;
        sensorEntry.backend.sensorStartStrategy          = sensorStartStrategy_;

        sensorEntry.param.originalStreamFormatFilterList = { OB_FORMAT_Y8, OB_FORMAT_NV12, OB_FORMAT_BA81, OB_FORMAT_YV12, OB_FORMAT_UYVY };
        sensorEntry.param.streamFormatReplaceMap         = { { OB_FORMAT_Z16, OB_FORMAT_Y16 } };
        sensorEntry.param.currentDepthAlgMode            = currentDepthAlgMode_;

        auto sensor = std::make_shared<VideoSensor>(shared_from_this(), OB_SENSOR_DEPTH, sensorEntry.backend, sensorEntry.param);
        sensor->setFramePreProcessFunc([&](std::shared_ptr<Frame> frame) {
            frame->as<DepthFrame>()->setValueScale(deviceDepthParam_.depthUnit);
            if(!hwD2DEnable_ && !swD2DEnable_) {
                frame->as<DepthFrame>()->setFormat(OB_FORMAT_DISP16);
            }
        });

        std::function<void(OBSensorType, OBStreamState)> onDepthStreamStateChanged = [&](OBSensorType sensorType, OBStreamState state) {
            // TODO try-catch only avoid crash,the actual issue is pending confirm.
            try {
                if(sensorType == OB_SENSOR_DEPTH && state == STREAM_STATE_STREAMING) {
                    auto resLock              = tryLockResource();
                    auto depthSensorEntry     = getSensorEntry(resLock, OB_SENSOR_DEPTH);
                    auto currentStreamProfile = depthSensorEntry.sensor->getCurrentStreamProfile();

                    auto disparityParam = algParamManager_->getCurrentDisparityProcessParam();
                    algParamManager_->registerDisparityProcessParam(currentStreamProfile, disparityParam);

                    if(depthSensorEntry.backend.frameProcessor) {
                        auto depthFrameProcessor = std::dynamic_pointer_cast<DepthFrameProcessor>(depthSensorEntry.backend.frameProcessor);
                        depthFrameProcessor->updateDisparityParam(disparityParam);
                    }
                }
#ifdef __linux__
                // Restart any remaining depth/IR sensors if any depth/IR sensor stops to prevent stream interruption.
                int fwVersion = 0;
                getFirmwareVersionInt(fwVersion);
                if(fwVersion <= 10220 && state == STREAM_STATE_STOPED
                   && (sensorType == OB_SENSOR_DEPTH || sensorType == OB_SENSOR_IR_LEFT || sensorType == OB_SENSOR_IR_RIGHT)) {
                    auto                                  resLock = tryLockResource();
                    std::vector<std::shared_ptr<ISensor>> sensorList;
                    for(auto &item: sensorEntryList_) {
                        auto sensor = item.second.sensor;
                        if(!sensor || !sensor->isStreamStarted()
                           || (item.first != OB_SENSOR_DEPTH && item.first != OB_SENSOR_IR_LEFT && item.first != OB_SENSOR_IR_RIGHT)) {
                            continue;
                        }
                        sensorList.push_back(sensor);
                        sensor->stop();
                    }

                    for(auto &sensor: sensorList) {
                        auto sp = sensor->getCurrentStreamProfile();
                        sensor->start(sp);
                    }
                }
#endif
            }
            catch(const std::exception &e) {
                LOG_ERROR("Exception caught:{}", e.what());
            }
        };

        devEventBus_->listenEvent("SensorStreamStateChanged", onDepthStreamStateChanged, std::to_string((long)this) + "_DepthStreamStateChanged");

        sensorIter->second.sensor = sensor;
        LOG_INFO("Depth sensor has been created!");
    }
}

void G330Device::createColorSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_COLOR);
    if(sensorIter != sensorEntryList_.end() && !sensorIter->second.sensor) {
        auto &sensorEntry = sensorIter->second;
        auto  port        = obPal_->createSourcePort(sensorEntry.sourcePortInfo);

        std::vector<FrameProcessingBlockConfig> blockConfigList = {
            { "FormatConverter", false },
            { "FrameMirror", false },
            { "FrameFlip", false },
            { "FrameRotate", false },
        };
        auto frameProcessor                              = std::make_shared<ColorFrameProcessor>(blockConfigList);
        sensorEntry.backend.frameProcessor               = frameProcessor;
        sensorEntry.backend.devCmd                       = command_;
        sensorEntry.backend.eventBus                     = devEventBus_;
        sensorEntry.backend.sourcePort                   = port;
        sensorEntry.backend.deviceSyncConfigurator       = syncConfigurator_;
        sensorEntry.backend.frameTimestampConverter      = std::make_shared<G330TimestampConverter>(frameTimeStampMetadataType_, globalTspFitter_);
        sensorEntry.backend.frameMetadataParserContainer = colorMdParserContainer_;
        sensorEntry.backend.algParamManager              = algParamManager_;
        sensorEntry.backend.sensorStartStrategy          = sensorStartStrategy_;

        sensorEntry.param.originalStreamFormatFilterList = { OB_FORMAT_NV12 };
        sensorEntry.param.virtualStreamFormatList        = {
            { OB_FORMAT_YUYV, OB_FORMAT_RGB },  { OB_FORMAT_YUYV, OB_FORMAT_RGBA }, { OB_FORMAT_YUYV, OB_FORMAT_BGR },
            { OB_FORMAT_YUYV, OB_FORMAT_BGRA }, { OB_FORMAT_YUYV, OB_FORMAT_Y16 },  { OB_FORMAT_YUYV, OB_FORMAT_Y8 },
        };
        sensorEntry.param.streamFormatReplaceMap = { { OB_FORMAT_BYR2, OB_FORMAT_RW16 } };
        sensorEntry.param.currentDepthAlgMode    = currentDepthAlgMode_;

        auto sensor               = std::make_shared<VideoSensor>(shared_from_this(), OB_SENSOR_COLOR, sensorEntry.backend, sensorEntry.param);
        sensorIter->second.sensor = sensor;

        LOG_INFO("Color sensor has been created!");
    }
}

void G330Device::createLeftIrSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_IR_LEFT);
    if(sensorIter != sensorEntryList_.end()) {
        auto &sensorEntry = sensorIter->second;
        auto  port        = obPal_->createSourcePort(sensorEntry.sourcePortInfo);

        std::vector<FrameProcessingBlockConfig> blockTypeList = {
            { "FrameUnpacker", true },
            { "FrameMirror", false },
            { "FrameFlip", false },
            { "FrameRotate", false },
        };

        if(!sensorStartStrategy_) {
            sensorStartStrategy_ = std::make_shared<G330SensorStartStrategy>(shared_from_this());
        }

        auto frameProcessor                              = std::make_shared<FrameProcessor>(blockTypeList);
        sensorEntry.backend.frameProcessor               = frameProcessor;
        sensorEntry.backend.devCmd                       = command_;
        sensorEntry.backend.eventBus                     = devEventBus_;
        sensorEntry.backend.sourcePort                   = port;
        sensorEntry.backend.deviceSyncConfigurator       = syncConfigurator_;
        sensorEntry.backend.frameTimestampConverter      = std::make_shared<G330TimestampConverter>(frameTimeStampMetadataType_, globalTspFitter_);
        sensorEntry.backend.frameMetadataParserContainer = depthMdParserContainer_;
        sensorEntry.backend.algParamManager              = algParamManager_;
        sensorEntry.backend.sensorStartStrategy          = sensorStartStrategy_;

        sensorEntry.param.originalStreamFormatFilterList = { OB_FORMAT_Z16, OB_FORMAT_BA81, OB_FORMAT_YV12 };
        sensorEntry.param.streamFormatReplaceMap         = { { OB_FORMAT_NV12, OB_FORMAT_Y12 } };
        sensorEntry.param.virtualStreamFormatList        = { { OB_FORMAT_Y12, OB_FORMAT_Y16 } };
        sensorEntry.param.currentDepthAlgMode            = currentDepthAlgMode_;

        auto sensor               = std::make_shared<VideoSensor>(shared_from_this(), OB_SENSOR_IR_LEFT, sensorEntry.backend, sensorEntry.param);
        sensorIter->second.sensor = sensor;

        LOG_INFO("Left IR sensor has been created!");
    }
}

void G330Device::createRightIrSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_IR_RIGHT);
    if(sensorIter != sensorEntryList_.end()) {
        auto                                   &sensorEntry   = sensorIter->second;
        auto                                    port          = obPal_->createSourcePort(sensorEntry.sourcePortInfo);
        std::vector<FrameProcessingBlockConfig> blockTypeList = {
            { "FrameUnpacker", true },
            { "FrameMirror", false },
            { "FrameFlip", false },
            { "FrameRotate", false },
        };

        if(!sensorStartStrategy_) {
            sensorStartStrategy_ = std::make_shared<G330SensorStartStrategy>(shared_from_this());
        }

        auto frameProcessor                              = std::make_shared<FrameProcessor>(blockTypeList);
        sensorEntry.backend.frameProcessor               = frameProcessor;
        sensorEntry.backend.devCmd                       = command_;
        sensorEntry.backend.eventBus                     = devEventBus_;
        sensorEntry.backend.sourcePort                   = port;
        sensorEntry.backend.deviceSyncConfigurator       = syncConfigurator_;
        sensorEntry.backend.frameTimestampConverter      = std::make_shared<G330TimestampConverter>(frameTimeStampMetadataType_, globalTspFitter_);
        sensorEntry.backend.frameMetadataParserContainer = depthMdParserContainer_;
        sensorEntry.backend.algParamManager              = algParamManager_;
        sensorEntry.backend.sensorStartStrategy          = sensorStartStrategy_;

        sensorEntry.param.originalStreamFormatFilterList = { OB_FORMAT_Z16, OB_FORMAT_Y8, OB_FORMAT_NV12, OB_FORMAT_UYVY };
        sensorEntry.param.streamFormatReplaceMap         = { { OB_FORMAT_YV12, OB_FORMAT_Y12 }, { OB_FORMAT_BA81, OB_FORMAT_Y8 } };
        sensorEntry.param.virtualStreamFormatList        = { { OB_FORMAT_Y12, OB_FORMAT_Y16 } };
        sensorEntry.param.currentDepthAlgMode            = currentDepthAlgMode_;

        auto sensor               = std::make_shared<VideoSensor>(shared_from_this(), OB_SENSOR_IR_RIGHT, sensorEntry.backend, sensorEntry.param);
        sensorIter->second.sensor = sensor;

        LOG_INFO("Right IR sensor has been created!");
    }
}

void G330Device::createAccelSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_ACCEL);
    if(sensorIter != sensorEntryList_.end() && !sensorIter->second.sensor) {
        auto &sensorEntry = sensorIter->second;
        auto  port        = obPal_->createSourcePort(sensorEntry.sourcePortInfo);

        std::shared_ptr<AccelSensor>            accelSensor   = nullptr;
        std::vector<FrameProcessingBlockConfig> blockTypeList = {
            { "IMUFrameTransformer", true },
        };
        auto frameProcessor = std::make_shared<FrameProcessor>(blockTypeList);
        auto block          = std::dynamic_pointer_cast<IMUFrameTransformer>(frameProcessor->getBlock("IMUFrameTransformer"));
        block->setIMUCorrectionMode(TK_CORRECTION_MODE);
        block->setIMUCalibrationParam(imuCalibParam_);
        block->setProcessFuncEnable(true);

        sensorEntry.backend.frameProcessor         = frameProcessor;
        sensorEntry.backend.devCmd                 = command_;
        sensorEntry.backend.eventBus               = devEventBus_;
        sensorEntry.backend.sourcePort             = port;
        sensorEntry.backend.deviceSyncConfigurator = syncConfigurator_;
        sensorEntry.backend.algParamManager        = algParamManager_;

        uint64_t tspClockFrq                        = 1000000;
        uint64_t devClockFrq                        = 1000;
        sensorEntry.backend.frameTimestampConverter = std::make_shared<FrameTimestampConverter>(globalTspFitter_, tspClockFrq, devClockFrq);

        accelSensor        = std::make_shared<AccelSensor>(shared_from_this(), sensorEntry.backend, sensorEntry.param);
        sensorEntry.sensor = accelSensor;
        if(sensorEntry.sensor) {
            LOG_INFO("Accel sensor has been created!");
        }
    }
}

void G330Device::createGyroSensor() {
    auto sensorIter = sensorEntryList_.find(OB_SENSOR_GYRO);
    if(sensorIter != sensorEntryList_.end() && !sensorIter->second.sensor) {
        auto &sensorEntry = sensorIter->second;
        auto  port        = obPal_->createSourcePort(sensorEntry.sourcePortInfo);

        std::shared_ptr<GyroSensor>             gyroSensor    = nullptr;
        std::vector<FrameProcessingBlockConfig> blockTypeList = {
            { "IMUFrameTransformer", true },
        };
        auto frameProcessor = std::make_shared<FrameProcessor>(blockTypeList);
        auto block          = std::dynamic_pointer_cast<IMUFrameTransformer>(frameProcessor->getBlock("IMUFrameTransformer"));
        block->setIMUCorrectionMode(TK_CORRECTION_MODE);
        block->setIMUCalibrationParam(imuCalibParam_);
        block->setProcessFuncEnable(true);

        sensorEntry.backend.frameProcessor         = frameProcessor;
        sensorEntry.backend.devCmd                 = command_;
        sensorEntry.backend.eventBus               = devEventBus_;
        sensorEntry.backend.sourcePort             = port;
        sensorEntry.backend.deviceSyncConfigurator = syncConfigurator_;
        sensorEntry.backend.algParamManager        = algParamManager_;

        uint64_t tspClockFrq                        = 1000000;
        uint64_t devClockFrq                        = 1000;
        sensorEntry.backend.frameTimestampConverter = std::make_shared<FrameTimestampConverter>(globalTspFitter_, tspClockFrq, devClockFrq);
        gyroSensor                                  = std::make_shared<GyroSensor>(shared_from_this(), sensorEntry.backend, sensorEntry.param);
        sensorEntry.sensor                          = gyroSensor;
        if(sensorEntry.sensor) {
            LOG_INFO("Gyro sensor has been created!");
        }
    }
}

void G330Device::loadDefaultConfig() {
    AbstractDevice::loadDefaultConfig();
    loadDefaultStreamProfile();
}

void G330Device::loadDefaultStreamProfile() {
    auto ctx       = Context::getInstance();
    auto xmlConfig = ctx->getXmlConfig();
    auto resLock   = tryLockResource();

    getDeviceInfo();  // ensure deviceInfo_ is valid
    auto deviceName    = StringUtils::remove(deviceInfo_->name_, " ");
    deviceName         = StringUtils::replace(deviceName, "+", "Adv");
    auto depthModeName = StringUtils::remove(currentDepthAlgMode_.name, " ");

    // USB2.0 默认配置
    if(xmlConfig->isLoadConfigFileSuccessful() && (deviceInfo_->connectionType_ == "USB2.1")) {
        for(auto &sensorEntryIter: sensorEntryList_) {
            auto  sensorType  = sensorEntryIter.first;
            auto &sensorEntry = sensorEntryIter.second;

            OBStreamType defStreamType = OB_STREAM_UNKNOWN;
            int          defFps        = 10;
            int          defWidth      = 848;
            int          defHeight     = 480;
            OBFormat     defFormat     = OB_FORMAT_Y16;

            switch(sensorType) {
            case OB_SENSOR_DEPTH:  // follow
                defStreamType = OB_STREAM_DEPTH;
                break;
            case OB_SENSOR_IR_LEFT:  // follow
                defFormat     = OB_FORMAT_Y8;
                defStreamType = OB_STREAM_IR_LEFT;
                break;
            case OB_SENSOR_IR_RIGHT:  // follow
                defFormat     = OB_FORMAT_Y8;
                defStreamType = OB_STREAM_IR_RIGHT;
                break;
            case OB_SENSOR_IR:  // follow
                defFormat     = OB_FORMAT_Y8;
                defStreamType = OB_STREAM_IR;
                break;
            case OB_SENSOR_COLOR: {
                defFormat     = OB_FORMAT_MJPG;
                defStreamType = OB_STREAM_COLOR;
                defWidth      = 1280;
                defHeight     = 720;

            } break;
            default:
                break;
            }

            if(defStreamType != OB_STREAM_UNKNOWN) {
                sensorEntry.param.defaultStreamProfile =
                    std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), defStreamType, defFormat, defWidth, defHeight, defFps);
            }
        }
    }
    // 读取特定相机深度工作模式下的播放配置信息
    if(xmlConfig->isLoadConfigFileSuccessful() && xmlConfig->isNodeContained("Device." + deviceName + "." + depthModeName)) {
        for(auto &sensorEntryIter: sensorEntryList_) {
            auto  sensorType  = sensorEntryIter.first;
            auto &sensorEntry = sensorEntryIter.second;
            switch(sensorType) {
            case OB_SENSOR_DEPTH:     // follow
            case OB_SENSOR_IR_LEFT:   // follow
            case OB_SENSOR_IR_RIGHT:  // follow
            case OB_SENSOR_IR:        // follow
            case OB_SENSOR_COLOR: {
                int         width, height, fps;
                std::string formatStr;
                auto        sensorName     = mapSensorTypeToString(sensorType);
                auto        sensorNodePath = std::string("Device.") + deviceName + "." + depthModeName + "." + sensorName;

                bool ret = xmlConfig->getIntValue(sensorNodePath + ".Width", width) && xmlConfig->getIntValue(sensorNodePath + ".Height", height)
                           && xmlConfig->getIntValue(sensorNodePath + ".FPS", fps) && xmlConfig->getStringValue(sensorNodePath + ".Format", formatStr);
                if(ret) {
                    auto format     = mapFormatStrToFormat(formatStr);
                    auto streamType = mapSensorTypeToStreamType(sensorType);
                    sensorEntry.param.defaultStreamProfile =
                        std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), streamType, format, width, height, fps);
                }
            } break;
            default:
                break;
            }
        }
    }
}

void G330Device::initDepthProcessParam() {
    LOG_DEBUG("Init depth process param start!");
    algParamManager_ = std::make_shared<G330AlgParamManager>(command_, deviceInfo_->pid_);

    deviceDepthParam_                = { 0 };
    deviceDepthParam_.minDepthValue  = 100;
    deviceDepthParam_.maxDepthValue  = 10000;
    deviceDepthParam_.maxDepthLimit  = 15000;
    deviceDepthParam_.minDepthLimit  = 15000;
    deviceDepthParam_.disparityParam = algParamManager_->getCurrentDisparityProcessParam();
    deviceDepthParam_.isDualCamera   = algParamManager_->isBinocularCamera();
    deviceDepthParam_.depthUnit      = 1.0f;
    BEGIN_TRY_EXECUTE({
        OBPropertyValue propertyValue{ 0 };
        auto            accessor = getPropertyAccessor(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT, OB_PERMISSION_READ);
        accessor->getPropertyValue(&propertyValue);
        deviceDepthParam_.depthUnit = (float)propertyValue.floatValue;
    })
    CATCH_EXCEPTION_AND_LOG(WARN, "get device depth unit flexible adjust float failed!")

        d2cProfileList_ = algParamManager_->getD2CProfileList();  // for compatibility hw d2c on pipeline

    BEGIN_TRY_EXECUTE({
        OBPropertyValue propertyValue{ 0 };
        auto            accessor = getPropertyAccessor(OB_PROP_DISPARITY_TO_DEPTH_BOOL, OB_PERMISSION_READ);
        accessor->getPropertyValue(&propertyValue);
        hwD2DEnable_ = (bool)propertyValue.intValue;
    })
    CATCH_EXCEPTION_AND_LOG(WARN, "get device hwD2D status failed!")
    LOG_DEBUG("hwD2DEnable_: {0}", hwD2DEnable_);
    swD2DEnable_ = !hwD2DEnable_;  // if hw d2d is disabled, sw d2d is enabled by default
    LOG_DEBUG("swD2DEnable_: {0}", swD2DEnable_);

    imuCalibParam_ = algParamManager_->getIMUCalibrationParam();

    LOG_DEBUG("Init depth process param done!");
}

std::unique_ptr<PropertyAccessor> G330Device::getPropertyAccessor(OBPropertyID propertyId, OBPermissionType permission) {
    auto resLock = tryLockResource();
    propertyManager_->checkPropertyPermission(propertyId, permission);
    switch(propertyId) {

#ifdef WIN32
    case OB_PROP_COLOR_EXPOSURE_INT: {
        auto port = command_;
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        break;
    }
    case OB_PROP_IR_EXPOSURE_INT:
    case OB_PROP_DEPTH_EXPOSURE_INT: {
        propertyId = OB_PROP_DEPTH_EXPOSURE_INT;
        auto port  = command_;
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        break;
    }
#else
    case OB_PROP_IR_EXPOSURE_INT:
    case OB_PROP_DEPTH_EXPOSURE_INT: {
        propertyId  = OB_PROP_DEPTH_EXPOSURE_INT;
        auto sensor = getSensor(resLock, OB_SENSOR_DEPTH);
        auto port   = std::dynamic_pointer_cast<IPropertyPort>(sensor);
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        break;
    }
#endif
    case OB_PROP_COLOR_FLIP_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_COLOR);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_FLIP_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_LEFT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_RIGHT_FLIP_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_RIGHT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_DEPTH_FLIP_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_DEPTH);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_COLOR_ROTATE_INT: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_COLOR);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_ROTATE_INT: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_LEFT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_RIGHT_ROTATE_INT: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_RIGHT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_DEPTH_ROTATE_INT: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_DEPTH);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_COLOR_MIRROR_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_COLOR);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_DEPTH_MIRROR_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_DEPTH);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_MIRROR_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_LEFT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_IR_RIGHT_MIRROR_BOOL: {
        auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_IR_RIGHT);
        if(sensorEntry.backend.frameProcessor) {
            auto port = sensorEntry.backend.frameProcessor;
            return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        }
        break;
    }
    case OB_PROP_DEPTH_GAIN_INT:
    case OB_PROP_IR_GAIN_INT: {
        propertyId  = OB_PROP_DEPTH_GAIN_INT;
        auto sensor = getSensor(resLock, OB_SENSOR_DEPTH);
        auto port   = std::dynamic_pointer_cast<IPropertyPort>(sensor);
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        break;
    }
    case OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL:
    case OB_PROP_IR_AUTO_EXPOSURE_BOOL: {
        propertyId = OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL;
        auto port  = command_;
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
        break;
    }

    // case OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL:           // follow
    // case OB_PROP_DEPTH_EDGE_NOISE_REMOVAL_FILTER_BOOL:      // follow
    // case OB_PROP_DEPTH_SPATIAL_FAST_FILTER_BOOL:            // follow
    // case OB_PROP_DEPTH_SPATIAL_MODERATE_FILTER_BOOL:        // follow
    // case OB_PROP_DEPTH_SPATIAL_ADVANCED_FILTER_BOOL:        // follow
    // case OB_PROP_DEPTH_HOLE_FILLING_FILTER_BOOL:            // follow
    // case OB_PROP_DEPTH_TEMPORAL_FILTER_BOOL:                // follow
    // case OB_PROP_DEPTH_FILTER_DUMP_DIFF_LUT_INT:            // follow
    // case OB_STRUCT_DEPTH_DDO_CONFIG:                        // follow
    // case OB_STRUCT_DEPTH_DDO_CONFIG_DEFAULT:                // follow
    // case OB_STRUCT_DEPTH_NOISE_REMOVAL_FILTER_PARAMS:       // follow
    // case OB_STRUCT_DEPTH_EDGE_NOISE_REMOVAL_FILTER_PARAMS:  // follow
    // case OB_STRUCT_DEPTH_SPATIAL_FAST_FILTER_PARAMS:        // follow
    // case OB_STRUCT_DEPTH_SPATIAL_MODERATE_FILTER_PARAMS:    // follow
    // case OB_STRUCT_DEPTH_SPATIAL_ADVANCED_FILTER_PARAMS:    // follow
    // case OB_STRUCT_DEPTH_HOLE_FILLING_FILTER_PARAMS:        // follow
    // case OB_STRUCT_DEPTH_TEMPORAL_FILTER_PARAMS:            // follow
    // case OB_PROP_DEPTH_SOFT_FILTER_BOOL:                    // follow
    // case OB_PROP_DEPTH_MAX_DIFF_INT:                        // follow
    // case OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT: {
    //     auto sensorEntry = getSensorEntry(resLock, OB_SENSOR_DEPTH);
    //     if(sensorEntry.backend.frameProcessor) {
    //         auto port = std::dynamic_pointer_cast<IPropertyPort>(sensorEntry.backend.frameProcessor);
    //         return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
    //     }
    //     break;
    // }
    default:
        break;
    }
    if(propertyId == OB_PROP_GYRO_FULL_SCALE_INT) {
        auto port = std::dynamic_pointer_cast<IPropertyPort>(command_);
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, permission, devEventBus_));
    }
    return std::move(AbstractDevice::getPropertyAccessor(propertyId, permission));
}

std::unique_ptr<PropertyAccessor> G330Device::getPropertyAccessorForce(OBPropertyID propertyId) {
    auto                           resLock = tryLockResource();
    std::shared_ptr<IPropertyPort> port;
    if(propertyId >= OB_PROP_IR_AUTO_EXPOSURE_BOOL && propertyId <= OB_PROP_IR_GAIN_INT) {
        OBSensorType irSensorType = OB_SENSOR_IR_LEFT;
        auto         sensor       = getSensor(resLock, irSensorType);
        port                      = std::dynamic_pointer_cast<IPropertyPort>(sensor);
    }

    if(port) {
        return std::move(libobsensor::make_unique<PropertyAccessor>(std::move(resLock), port, propertyId, OB_PERMISSION_READ_WRITE, devEventBus_));
    }
    return AbstractDevice::getPropertyAccessorForce(propertyId);
}

void G330Device::deviceUpgrade(std::string filePath, DeviceUpgradeCallback upgradeCallback, bool async) {
    std::vector<uint8_t> fileData;

    std::ifstream file(filePath, std::ios::binary);
    if(!file.is_open()) {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Failed to open file: " << filePath);
    }

    file.seekg(0, std::ios::end);
    auto fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    fileData.resize(fileSize);
    file.read((char *)fileData.data(), fileSize);
    file.close();

    deviceUpgrade((char *)fileData.data(), fileSize, upgradeCallback, async);
}

void G330Device::deviceUpgrade(const char *fileData, uint32_t fileSize, DeviceUpgradeCallback upgradeCallback, bool async) {
    if(upgradeThread_.joinable()) {
        upgradeThread_.join();
    }
    auto        fwHandle     = std::make_shared<G330Firmware>((const uint8_t *)fileData, fileSize);
    const auto &fwFileHeader = fwHandle->getFileHeader();
    if(std::string(fwFileHeader.serial) != "Gemini2R" && std::string(fwFileHeader.serial) != "G300" && std::string(fwFileHeader.serial) != "Gemini 300"
       && std::string(fwFileHeader.serial) != "Gemini 330") {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Invalid firmware file with unmatched serial: " << fwFileHeader.serial);
    }
    const auto &firmwareDataList = fwHandle->getFirmwareDataList();
    auto        upgradeFunc      = [this, upgradeCallback, firmwareDataList]() {
        globalTspFitter_->pause();
        auto guard = std::unique_ptr<int, std::function<void(void *)>>(nullptr, [&](void *) { globalTspFitter_->resume(); });

        auto fwNum = firmwareDataList.size();
        auto fwIdx = 0;
        for(auto &firmwareData: firmwareDataList) {
            fwIdx++;
            if(!G330Firmware::isFirmwareDataAdaptable(firmwareData, deviceInfo_->pid_)) {
                continue;
            }

            auto        dataSize = firmwareData.header.actualSize;
            auto        address  = firmwareData.header.flash_address;
            std::string ispVer   = firmwareData.header.version;

            if(address == ISP_FLASH_ADDR && ispVer == ispVersion_) {
                // isp固件为最后一个时，直接回调成功。
                if(fwIdx >= fwNum) {
                    std::string info = "Upgrade successful! Please reboot your device manually!";
                    upgradeCallback(STAT_DONE, info.c_str(), 100);
                    return;
                }
                continue;
            }

            OBUpgradeState upgradeState = ERR_OTHER;

            const int MAX_RETRY = 4;
            int       retry     = MAX_RETRY;
            do {
                if(retry != MAX_RETRY) {
                    LOG_DEBUG("Update firmware: [{0}] failed! retry {1}/{2}", firmwareData.header.name, retry, MAX_RETRY);
                }
                BEGIN_TRY_EXECUTE({
                    command_->writeFlash(
                        address, firmwareData.data.data(), dataSize,
                        [&](OBDataTranState state, uint8_t percent) {
                            std::string msg;
                            switch(state) {
                            case DATA_TRAN_STAT_DONE:
                                if(fwIdx >= fwNum) {
                                    upgradeState = STAT_DONE;
                                    msg          = "Upgrade successful! Please reboot your device manually!";
                                }
                                else {
                                    upgradeState = STAT_FILE_TRANSFER;
                                    msg          = std::string("The ") + firmwareData.header.name + " firmware upgrade done!";
                                }
                                break;
                            case DATA_TRAN_STAT_TRANSFERRING:
                                upgradeState = STAT_FILE_TRANSFER;
                                msg          = std::string("The ") + firmwareData.header.name + " firmware data transferring!";
                                break;
                            case DATA_TRAN_STAT_VERIFYING:
                                upgradeState = STAT_VERIFY_IMAGE;
                                msg          = std::string("The ") + firmwareData.header.name + " firmware data verifying!";
                                break;
                            default:
                                upgradeState = ERR_OTHER;
                                msg          = std::string("The ") + firmwareData.header.name + "Upgrade failed!";
                                break;
                            }
                            upgradeCallback(upgradeState, msg.c_str(), percent);
                        },
                        false);
                            })
                CATCH_EXCEPTION_AND_EXECUTE({ break; })
            } while(retry-- > 0 && upgradeState != STAT_DONE && upgradeState != STAT_FILE_TRANSFER);

            if(upgradeState != STAT_DONE && upgradeState != STAT_FILE_TRANSFER) {
                break;
            }
        }
    };
    upgradeThread_ = std::thread(upgradeFunc);
    if(!async) {
        upgradeThread_.join();
    }
}

OBCameraParam G330Device::getCurCameraParam() {
    auto calibParamList = algParamManager_->getCalibrationCameraParamList();
    if(curD2CParamIndex_ + 1 > calibParamList.size()) {
        LOG_WARN("current d2c param index out of range!");
        return {};
    }
    auto cameraParam = calibParamList[curD2CParamIndex_];
    cameraParam      = preProcessCameraParam(cameraParam);
    return cameraParam;
}

std::vector<OBD2CProfile> G330Device::getD2CSupportedProfileList() {
    return d2cProfileList_;
}

std::vector<OBCameraParam> G330Device::getCalibrationCameraParamList() {
    return algParamManager_->getCalibrationCameraParamList();
}

OBDepthAlgModeChecksum G330Device::getCurrentDepthAlgModeChecksum() {
    if(strnlen(currentDepthAlgMode_.name, sizeof(currentDepthAlgMode_.name)) == 0) {
        currentDepthAlgMode_ = requestCurrentDepthAglMode();
    }
    return currentDepthAlgMode_;
}

OBDepthAlgModeChecksum G330Device::requestCurrentDepthAglMode() {
    uint32_t size;
    uint8_t *data = nullptr;

    OBDepthAlgModeChecksum algModeChecksum{ 0 };
    algModeChecksum.optionCode = OBDepthModeOptionCode::INVALID;

    auto         accessor        = getPropertyAccessorForce(OB_STRUCT_CURRENT_DEPTH_ALG_MODE);
    OBCmdVersion localCmdVersion = OB_CMD_VERSION_INVALID;
    accessor->getStructData([&](OBCmdVersion cmdVersion, uint8_t *myData, uint32_t myDataSize) {
        if(!data) {
            localCmdVersion = cmdVersion;
            size            = myDataSize;
            data            = new uint8_t[myDataSize]{ 0 };
        }
        memcpy(data, myData, myDataSize);
    });

    if(data && localCmdVersion != OB_CMD_VERSION_INVALID) {
        if(size >= sizeof(algModeChecksum)) {
            memcpy(&algModeChecksum, data, sizeof(algModeChecksum));
        }
        else {
            size = 0;
            delete[] data;
            data = nullptr;

            throw libobsensor::libobsensor_exception("Get current depth alg mode failed. data size not match", OB_EXCEPTION_TYPE_INVALID_VALUE);
        }

        size = 0;
        delete[] data;
        data = nullptr;
    }
    else {
        LOG_DEBUG("data:{0}, size:{1}, sizeof(mode):{2}", (uint64_t)data, size, sizeof(algModeChecksum));
        throw libobsensor::libobsensor_exception("Get current depth alg mode failed. data is null", OB_EXCEPTION_TYPE_INVALID_VALUE);
    }
    return algModeChecksum;
}

void G330Device::switchDepthAlgMode(const char *modeName) {
    auto                   modeList = getDepthAlgModeChecksumList();
    OBDepthAlgModeChecksum dstMode;
    memset(&dstMode, 0, sizeof(dstMode));
    for(auto &mode: modeList) {
        if(strcmp(mode.name, modeName) == 0) {
            dstMode = mode;
            break;
        }
    }

    if(strlen(dstMode.name) == 0) {
        std::string totalNames;
        for(auto &mode: modeList) {
            if(!totalNames.empty()) {
                totalNames = totalNames + ",";
            }
            totalNames = totalNames + std::string(mode.name);
        }
        throw unsupported_operation_exception(
            std::string("Invalid depth mode: " + std::string(modeName) + ", support depth work mode list: " + totalNames).c_str());
    }

    switchDepthAlgMode(dstMode);
}

void G330Device::switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode) {
    // check stream status
    for(auto it: sensorEntryList_) {
        auto sensor = it.second.sensor;
        if(sensor && sensor->isStreamStarted()) {
            std::ostringstream ss;
            ss << "Cannot switch depth work mode while stream is started. Please stop stream first! sensor " << sensor->getSensorType() << " is streaming";
            throw unsupported_operation_exception(ss.str());
        }
    }

    if(strncmp(currentDepthAlgMode_.name, targetDepthMode.name, sizeof(targetDepthMode.name)) == 0) {
        LOG_INFO("switchDepthWorkMode done! same mode. currentDepthMode: {0}, targetDepthMode:{1}", currentDepthAlgMode_, targetDepthMode);
        return;
    }

    // 发送指令切换模式
    auto serializer = createDataSerializer(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_CMD_VERSION_V0, (void *)&targetDepthMode);
    auto accessor   = getPropertyAccessorForce(OB_STRUCT_CURRENT_DEPTH_ALG_MODE);
    accessor->setStructData(serializer);

    // 检查返回状态；
    auto newDepthMode = requestCurrentDepthAglMode();
    if(memcmp(&newDepthMode.checksum, &targetDepthMode.checksum, sizeof(newDepthMode.checksum)) != 0) {
        throw libobsensor::libobsensor_exception("switchDepthAlgMode failed. checksum not equal. ", OB_EXCEPTION_TYPE_UNKNOWN);
    }
    LOG_INFO("switchDepthWorkMode done! oldDepthMode: {0}, newDepthMode: {1}", currentDepthAlgMode_, newDepthMode);
    currentDepthAlgMode_ = newDepthMode;
}

OBD2CAlignParam G330Device::getD2CAlignParam() {
    OBD2CAlignParam alignParam;
    alignParam.depth_undist_switch = false;  // 深度图是否去畸变
    alignParam.rgb_undist_switch   = true;   // rgb是否去畸变
    alignParam.enable_gap_fill     = true;   // 是否填充
    alignParam.enable_depth_Copy4  = true;   // 开启1copy4填补空洞策略

    alignParam.depth_unit_mm = deviceDepthParam_.depthUnit;  // 深度单位

    return alignParam;
}

OBD2CProfile G330Device::getSupportedProfileInfo(uint32_t colorWidth, uint32_t colorHeight, uint32_t depthWidth, uint32_t depthHeight, OBAlignMode alignMode) {
    OBD2CProfile info;
    memset(&info, 0, sizeof(OBD2CProfile));

    for(auto &sp: d2cProfileList_) {
        if(((sp.colorWidth == colorWidth || colorWidth == 0) && (sp.colorHeight == colorHeight || colorHeight == 0))
           && ((sp.depthWidth == depthWidth || depthWidth == 0) && (sp.depthHeight == depthHeight || depthHeight == 0))
           && ((alignMode == ALIGN_D2C_HW_MODE && (sp.alignType & ALIGN_D2C_HW)) || (alignMode == ALIGN_D2C_SW_MODE && (sp.alignType & ALIGN_D2C_SW)))) {
            info = sp;
            break;
        }
    }

    if(!info.valid()) {
        for(auto &sp: d2cProfileList_) {
            if((sp.colorWidth / (float)sp.colorHeight) == (colorWidth / (float)colorHeight) && sp.depthWidth == depthWidth && sp.depthHeight == sp.depthHeight
               && (sp.alignType & ALIGN_D2C_SW)) {
                info                              = sp;
                float scale                       = info.colorWidth / (float)colorWidth;
                info.postProcessParam.alignLeft   = info.postProcessParam.alignLeft / scale;
                info.postProcessParam.alignTop    = info.postProcessParam.alignTop / scale;
                info.postProcessParam.alignRight  = info.postProcessParam.alignRight / scale;
                info.postProcessParam.alignBottom = info.postProcessParam.alignBottom / scale;
                info.postProcessParam.depthScale  = info.postProcessParam.depthScale / scale;
                break;
            }
        }
    }

    if(!info.valid()) {
        for(auto &sp: d2cProfileList_) {
            if((sp.colorWidth / (float)sp.colorHeight == colorWidth / (float)colorHeight)
               && (sp.depthWidth / (float)sp.depthHeight == depthWidth / (float)depthHeight)
               && ((alignMode == ALIGN_D2C_HW_MODE && (sp.alignType & ALIGN_D2C_HW)) || (alignMode == ALIGN_D2C_SW_MODE && (sp.alignType & ALIGN_D2C_SW)))) {
                info = sp;
                break;
            }
        }
    }

    AbstractDevice::preProcessD2CSupportedProfile(&info);

    return info;
}

std::vector<OBDepthAlgModeChecksum> G330Device::getDepthAlgModeChecksumList() {
    std::unique_lock<std::mutex> lk(depthAlgModeChecksumMutex_);
    if(!depthAlgModeChecksumList_.empty()) {
        return depthAlgModeChecksumList_;
    }

    // 查询相机深度模式列表
    uint8_t     *data            = nullptr;
    uint32_t     size            = 0;
    OBCmdVersion localCmdVersion = OB_CMD_VERSION_INVALID;
    auto         accessor        = getPropertyAccessorForce(OB_RAW_DATA_DEPTH_ALG_MODE_LIST);
    BEGIN_TRY_EXECUTE({
        accessor->getStructDataList(
            [&](OBDataTranState state, OBCmdVersion cmdVersion, OBDataChunk *dataChunk) {
                if(state == DATA_TRAN_STAT_TRANSFERRING) {
                    if(data == nullptr) {
                        localCmdVersion = cmdVersion;
                        size            = dataChunk->fullDataSize;
                        data            = new uint8_t[size]{ 0 };
                    }
                    memcpy(data + dataChunk->offset, dataChunk->data, dataChunk->size);
                }
            },
            false);
    })
    CATCH_EXCEPTION_AND_LOG(WARN, "get depth alg mode checksum list failed.")

    if(size && localCmdVersion != OB_CMD_VERSION_INVALID) {
        depthAlgModeChecksumList_ = depthAlgModeChecksumListParse(data, size);
    }

    if(data != nullptr) {
        delete[] data;
        data = nullptr;
        size = 0;
    }

    return depthAlgModeChecksumList_;
}

std::vector<std::shared_ptr<FrameProcessingBlock>> G330Device::getRecommendedProcessingBlockList(OBSensorType type) {
    if(type == OB_SENSOR_DEPTH) {
        if(depthProcessingBlockList_.size() == 0) {
            depthProcessingBlockList_.push_back(std::make_shared<DecimationFilter>());
            depthProcessingBlockList_.push_back(std::make_shared<HdrMerge>());
            depthProcessingBlockList_.push_back(std::make_shared<SequenceIdFilter>());
            depthProcessingBlockList_.push_back(std::make_shared<ThresholdFilter>());

            std::shared_ptr<NoiseRemovalFilter> noiseFilter     = std::make_shared<NoiseRemovalFilter>();
            OBNoiseRemovalFilterParams          noiseFilerParam = noiseFilter->getFilterParams();
            noiseFilerParam.max_size                            = 80;
            noiseFilerParam.disp_diff                           = 256;
            noiseFilter->setDefaultFilterParams(noiseFilerParam);
            libdepth_disp::DDOConfig anchorCondig = noiseFilter->getAnchorConfig();
            anchorCondig.width                    = 848;
            anchorCondig.height                   = 480;
            anchorCondig.fraction_bit_size        = 8;
            noiseFilter->setAnchorConfig(anchorCondig);
            depthProcessingBlockList_.push_back(noiseFilter);

            std::shared_ptr<SpatialAdvancedFilter> spatFilter      = std::make_shared<SpatialAdvancedFilter>();
            OBSpatialAdvancedFilterParams          spatFilterParam = spatFilter->getFilterParams();
            spatFilterParam.alpha                                  = 0.5f;
            spatFilterParam.disp_diff                              = 160;
            spatFilterParam.radius                                 = 1;
            spatFilterParam.magnitude                              = 1;
            spatFilter->setDefaultFilterParams(spatFilterParam);
            libdepth_disp::DDOConfig spatAnchorCondig = spatFilter->getAnchorConfig();
            spatAnchorCondig.width                    = 848;
            spatAnchorCondig.height                   = 480;
            spatAnchorCondig.fraction_bit_size        = 8;
            spatFilter->setAnchorConfig(spatAnchorCondig);
            depthProcessingBlockList_.push_back(spatFilter);

            std::shared_ptr<TemporalFilter> tempFilter = std::make_shared<TemporalFilter>();
            tempFilter->setDefaultDiffScale(0.1);
            tempFilter->setDefaultWeight(0.4);
            tempFilter->enable(false);
            depthProcessingBlockList_.push_back(tempFilter);

            depthProcessingBlockList_.push_back(std::make_shared<HoleFillingFilter>());

            // Disparity to depth
            std::shared_ptr<DisparityTransform> dtFilter = std::make_shared<DisparityTransform>();
            depthProcessingBlockList_.push_back(dtFilter);

            for(int i = 0; i < depthProcessingBlockList_.size(); i++) {
                auto processingBlock = depthProcessingBlockList_[i];
                if(processingBlock != noiseFilter && processingBlock != dtFilter) {
                    processingBlock->enable(false);
                }
            }
        }
        return depthProcessingBlockList_;
    }
    else if(type == OB_SENSOR_COLOR) {
        if(colorProcessingBlockList_.size() == 0) {
            std::shared_ptr<DecimationFilter> decFilter = std::make_shared<DecimationFilter>();
            decFilter->enable(false);
            colorProcessingBlockList_.push_back(decFilter);
        }
        return colorProcessingBlockList_;
    }
    /*else if(type == OB_SENSOR_IR_RIGHT || type == OB_SENSOR_IR_LEFT) {
        if(irProcessingBlockList_.size() == 0) {
            irProcessingBlockList_.push_back(std::make_shared<DecimationFilter>());
            irProcessingBlockList_.push_back(std::make_shared<SequenceIdFilter>());
        }
        return irProcessingBlockList_;
    }*/

    std::vector<std::shared_ptr<FrameProcessingBlock>> processingBlockList;
    return processingBlockList;
}

std::vector<OBDepthAlgModeChecksum> G330Device::depthAlgModeChecksumListParse(const uint8_t *fileData, uint32_t dataSize) {
    std::vector<OBDepthAlgModeChecksum> output;
    const uint32_t                      typeSize = sizeof(OBDepthAlgModeChecksum);
    for(uint32_t i = 0, N = dataSize / typeSize; i < N; i++) {
        output.push_back(*(OBDepthAlgModeChecksum *)(fileData + i * typeSize));
    }
    return output;
}

void G330Device::reboot() {
    auto accessor = getPropertyAccessorForce(OB_PROP_DEVICE_RESET_BOOL);
    if(accessor) {
        preprocessReboot();

        OBPropertyValue value;
        value.intValue = 1;
        accessor->setPropertyValue(value);

        command_->handleDeviceDetached();
        deviceDetached_ = true;
    }
    else {
        throw libobsensor::unsupported_operation_exception("Reboot device: unsupported!");
    }
}

bool G330Device::onPropertyUpdate(OBPropertyID propertyId, OBPropertyValue propertyValue, OBPermissionType permissionType) {
    switch(propertyId) {
    case OB_PROP_DISPARITY_TO_DEPTH_BOOL: {
        hwD2DEnable_ = propertyValue.intValue;
        return AbstractDevice::onPropertyUpdate(propertyId, propertyValue, permissionType);
    }
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL: {
        swD2DEnable_ = propertyValue.intValue;
        return AbstractDevice::onPropertyUpdate(propertyId, propertyValue, permissionType);
    }
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT: {
        deviceDepthParam_.depthUnit = propertyValue.floatValue;
        return AbstractDevice::onPropertyUpdate(propertyId, propertyValue, permissionType);
    }
    case OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL: {
        AbstractDevice::onPropertyUpdate(propertyId, propertyValue, permissionType);
        return true;
    }
    default:
        break;
    }
    return AbstractDevice::onPropertyUpdate(propertyId, propertyValue, permissionType);
}

/* #endregion ------------G330Device end---------------- */

}  // namespace g2r
}  // namespace libobsensor
