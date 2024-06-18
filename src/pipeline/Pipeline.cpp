// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "Pipeline.hpp"

#include "logger/LoggerInterval.hpp"
#include "logger/LoggerHelper.hpp"
#include "context/Context.hpp"
#include "DevicePids.hpp"
#include "utils/Utils.hpp"

#include <cmath>
#include <algorithm>

namespace libobsensor {
Pipeline::Pipeline(std::shared_ptr<IDevice> dev) : device_(dev), streamState_(STREAM_STATE_STOPED) {
    LOG_DEBUG("Pipeline init ...");
    auto sensorTypeList = device_->getSensorTypeList();
    if(sensorTypeList.empty()) {
        throw std::runtime_error("This device has no valid sensor!");
    }

    loadFrameQueueSizeConfig();

    outputFrameQueue_ = std::make_shared<FrameQueue<const Frame>>(MAX_FRAME_QUEUE_SIZE);
    frameAggregator_  = std::make_shared<FrameAggregator>();
    frameAggregator_->setCallback([&](std::shared_ptr<const Frame> frame) { outputFrame(frame); });

    TRY_EXECUTE(enableFrameSync());

    auto deviceInfo = device_->getInfo();
    LOG_INFO("Pipeline created with device: {{name: {0}, sn: {1}}}, @0x{2:X}", deviceInfo->name_, deviceInfo->deviceSn_, (uint64_t)this);
}

Pipeline::~Pipeline() noexcept {
    LOG_DEBUG("Pipeline deInit start!");

    if(streamState_ == STREAM_STATE_STARTING || streamState_ == STREAM_STATE_STREAMING) {
        TRY_EXECUTE(stop());
    }

    outputFrameQueue_->clear();
    LOG_INFO("Pipeline destroyed! @0x{:X}", (uint64_t)this);
}

void Pipeline::loadDefaultConfig() {
    // todo: implement this function

    // config_ = std::make_shared<Config>();

    // auto ctx                = Context::getInstance();
    // auto xmlConfig          = ctx->getXmlConfig();
    // bool loadFromConfigFile = (xmlConfig->isLoadConfigFileSuccessful() && xmlConfig->isNodeContained("Pipeline"));

    // for(auto &sensorType: device_->getSensorTypeList()) {
    //     // 配置文件加载失败或不兼容时，尝试启用深度和彩色流
    //     if(!loadFromConfigFile && sensorType != OB_SENSOR_DEPTH && sensorType != OB_SENSOR_COLOR) {
    //         continue;
    //     }

    //     bool UseDefaultStreamProfile = true;
    //     if(loadFromConfigFile) {
    //         auto        sensorTypeName = mapSensorTypeToString(sensorType);
    //         std::string nodeName       = std::string("Pipeline.Stream.") + sensorTypeName;
    //         if(!xmlConfig->isNodeContained(nodeName)) {
    //             continue;  // 未找到改类型Sensor配置，跳过
    //         }
    //         xmlConfig->getBooleanValue(nodeName + ".UseDefaultStreamProfile", UseDefaultStreamProfile);
    //     }

    //     auto resLock     = device_->tryLockResource();
    //     auto sensor      = device_->getSensor(resLock, sensorType);
    //     auto profileList = sensor->getStreamProfileList();
    //     if(!profileList.empty()) {
    //         if(!UseDefaultStreamProfile) {  // 加载指定配置
    //             // defaultConfig->enableStream(profileList.front());
    //             int         width, height, fps;
    //             std::string formatStr;
    //             bool        loaded         = true;
    //             auto        sensorTypeName = mapSensorTypeToString(sensorType);
    //             std::string nodeName       = std::string("Pipeline.Stream.") + sensorTypeName;
    //             loaded &= xmlConfig->getIntValue(nodeName + ".Width", width);
    //             loaded &= xmlConfig->getIntValue(nodeName + ".Height", height);
    //             loaded &= xmlConfig->getIntValue(nodeName + ".FPS", fps);
    //             loaded &= xmlConfig->getStringValue(nodeName + ".Format", formatStr);
    //             if(loaded) {
    //                 auto format             = mapFormatStrToFormat(formatStr);
    //                 auto matchedProfileList = matchVideoStreamProfile(profileList, width, height, fps, format);
    //                 if(!matchedProfileList.empty()) {
    //                     config_->enableStream(matchedProfileList.front());  // 使用默认配置，第一项是默认配置
    //                     continue;                                           // 成功
    //                 }
    //                 else {
    //                     LOG_WARN("No matched profile found, use the default profile!");
    //                 }
    //             }
    //             // else if load failed, usr default profile
    //         }

    //         // 使用默认配置或者加载指定配置失败
    //         config_->enableStream(profileList.front());  // 使用默认配置，第一项是默认配置
    //     }
    // }
    // if(loadFromConfigFile) {
    //     int alignMode = 0;
    //     xmlConfig->getIntValue("Pipeline.AlignMode", alignMode);
    //     config_->setAlignMode(OBAlignMode(alignMode));

    //     bool frameSync = false;
    //     xmlConfig->getBooleanValue("Pipeline.FrameSync", frameSync);
    //     if(frameSync) {
    //         TRY_EXECUTE(enableFrameSync());
    //     }
    //     else {
    //         TRY_EXECUTE(disableFrameSync());
    //     }
    // }
}

void Pipeline::loadFrameQueueSizeConfig() {
    // todo: implement this function

    // auto ctx       = Context::getInstance();
    // auto xmlConfig = ctx->getXmlConfig();
    // if(xmlConfig->isLoadConfigFileSuccessful()) {
    //     xmlConfig->getIntValue("Memory.PipelineFrameQueueSize", MAX_FRAME_QUEUE_SIZE);
    //     if(MAX_FRAME_QUEUE_SIZE <= 0) {
    //         LOG_WARN("Read xml config:pipeline frame queue size is invalid!");
    //         MAX_FRAME_QUEUE_SIZE = 10;
    //     }
    // }
    // else {
    //     LOG_WARN("Default config file is not loaded!");
    // }
    // LOG_DEBUG("loadFrameQueueSizeConfig() config queue size: {}", MAX_FRAME_QUEUE_SIZE);
}

StreamProfileList Pipeline::getEnabledStreamProfileList() {
    if(!config_) {
        return {};
    }
    return config_->getEnabledStreamProfileList();
}

std::shared_ptr<Config> Pipeline::checkAndSetConfig(std::shared_ptr<const Config> cfg) {
    LOG_INFO("Check and set config start!");
    std::shared_ptr<Config> config                = cfg->clone();
    const auto              enabledStreamProfiles = cfg->getEnabledStreamProfileList();
    for(auto sp: enabledStreamProfiles) {
        auto streamType = sp->getType();
        auto sensorType = utils::mapStreamTypeToSensorType(streamType);
        auto sensor     = device_->getSensor(sensorType);
        if(!sensor) {
            throw std::runtime_error("No sensor matched!");
        }
        auto sensorSpList = sensor->getStreamProfileList();
        if(sensorType == OBSensorType::OB_SENSOR_ACCEL) {
            auto profile            = sp->as<AccelStreamProfile>();
            auto matchedProfileList = matchAccelStreamProfile(sensorSpList, profile->getFullScaleRange(), profile->getSampleRate());
            if(matchedProfileList.empty()) {
                throw std::runtime_error("No matched profile found!");
            }
            config->enableStream(matchedProfileList.front());
        }
        else if(sensorType == OBSensorType::OB_SENSOR_GYRO) {
            auto profile            = sp->as<GyroStreamProfile>();
            auto matchedProfileList = matchGyroStreamProfile(sensorSpList, profile->getFullScaleRange(), profile->getSampleRate());
            if(matchedProfileList.empty()) {
                throw std::runtime_error("No matched profile found!");
            }
            config->enableStream(matchedProfileList.front());
        }
        else {
            auto profile            = sp->as<VideoStreamProfile>();
            auto matchedProfileList = matchVideoStreamProfile(sensorSpList, profile->getWidth(), profile->getHeight(), profile->getFps(), profile->getFormat());
            if(matchedProfileList.empty()) {
                throw std::runtime_error("No matched profile found!");
            }
            config->enableStream(matchedProfileList.front());
        }
    }
    LOG_INFO("Check and set config done!");
    return config;
}

void Pipeline::start(std::shared_ptr<const Config> cfg) {
    LOG_DEBUG("Pipeline start() start!");

    if(cfg) {
        config_ = checkAndSetConfig(cfg);
    }
    else {
        LOG_DEBUG("start pipeline with default config");
        // loadDefaultConfig();  // todo: implement this function
        auto defConfig = std::make_shared<Config>();
        defConfig->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);
        defConfig->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);
        config_ = checkAndSetConfig(defConfig);
    }

    if(config_->isStreamEnabled(OB_STREAM_DEPTH) || config_->isStreamEnabled(OB_STREAM_COLOR)) {
        configAlignMode();
    }

    frameAggregator_->updateConfig(config_, true);

    streamState_ = STREAM_STATE_STARTING;
    BEGIN_TRY_EXECUTE({ startStream(); })
    CATCH_EXCEPTION_AND_EXECUTE({
        stop();
        throw;
    })

    LOG_INFO("Pipeline start done!");
}

void Pipeline::start(std::shared_ptr<const Config> cfg, FrameCallback callback) {
    pipelineCallback_ = callback;
    start(cfg);
}

std::shared_ptr<const Config> Pipeline::getConfig() {
    if(!config_) {
        loadDefaultConfig();
    }
    return config_;
}

void Pipeline::startStream() {
    LOG_INFO("Try to start streams!");
    auto spList = config_->getEnabledStreamProfileList();
    for(const auto &sp: spList) {
        auto streamType = sp->getType();
        auto sensorType = utils::mapStreamTypeToSensorType(streamType);
        auto sensor     = device_->getSensor(sensorType);
        if(!sensor) {
            throw std::runtime_error("No sensor matched!");
        }
        sensor->start(sp, [&](std::shared_ptr<const Frame> frame) { onFrameCallback(frame); });

        LOG_DEBUG("Sensor stream started, sensorType={}", sensor->getSensorType());
    }
    LOG_INFO("Start streams done!");
}

void Pipeline::onFrameCallback(std::shared_ptr<const Frame> frame) {
    std::unique_lock<std::mutex> lk(streamMutex_);
    if(streamState_ != STREAM_STATE_STOPED && streamState_ != STREAM_STATE_STOPPING) {
        if(streamState_ == STREAM_STATE_STARTING) {
            streamState_ = STREAM_STATE_STREAMING;
        }
        frameAggregator_->pushFrame(frame);
    }
    auto frameType = frame->getType();
    LOG_INTVL(LOG_INTVL_OBJECT_TAG + std::to_string(frameType), DEF_MIN_LOG_INTVL, spdlog::level::debug, "Frame received on pipeline! type={}", frameType);
}

void Pipeline::outputFrame(std::shared_ptr<const Frame> frame) {
    LOG_FREQ_CALC(ERROR, 5000, "Pipeline streaming... frameset output rate={freq}fps");
    if(streamState_ == STREAM_STATE_STREAMING) {
        if(pipelineCallback_ != nullptr) {
            pipelineCallback_(frame);
        }
        else {
            if(outputFrameQueue_->fulled()) {
                LOG_WARN_INTVL("Output frameset queue is full, drop oldest frameset!");
                outputFrameQueue_->dequeue();
            }
            outputFrameQueue_->enqueue(std::move(frame));
        }
    }
}

std::shared_ptr<const Frame> Pipeline::waitForFrame(uint32_t timeout_ms) {
    auto frame = outputFrameQueue_->dequeue(timeout_ms);
    if(!frame) {
        LOG_WARN_INTVL("Wait for frame timeout, you can try to increase the wait time! current timeout={}", timeout_ms);
        return nullptr;
    }
    return frame;
}

void Pipeline::stopStream() {
    // Femto设备内RGB-TOF主从模式时，关流顺序和开流顺序相反
    LOG_INFO("Try to stop streams!");
    if(!config_) {
        LOG_WARN("The config is null!");
        return;
    }

    auto cfgStreams = config_->getEnabledStreamProfileList();
    auto streamIter = cfgStreams.end();
    while(cfgStreams.size()) {
        streamIter--;
        auto streamType = (*streamIter)->getType();
        auto sensorType = utils::mapStreamTypeToSensorType(streamType);
        auto sensor     = device_->getSensor(sensorType);
        TRY_EXECUTE({ sensor->stop(); })
        LOG_INFO("Sensor stream stopped, sensorType={}", sensor->getSensorType());
        if(streamIter == cfgStreams.begin()) {
            break;
        }
    }

    LOG_INFO("Stop streams done!");
}

void Pipeline::stop() {
    LOG_INFO("Try to stop pipeline!");
    streamState_ = STREAM_STATE_STOPPING;
    BEGIN_TRY_EXECUTE({ stopStream(); })
    CATCH_EXCEPTION_AND_LOG(WARN, "Exception occurred when stop stream!")

    if(config_ && (config_->isStreamEnabled(OB_STREAM_DEPTH) || config_->isStreamEnabled(OB_STREAM_COLOR))) {
        resetAlignMode();
    }
    if(frameAggregator_) {
        frameAggregator_->clearAllFrameQueue();
    }

    // clear frameset
    outputFrameQueue_->flush();

    // clear callback
    pipelineCallback_ = nullptr;

    streamState_ = STREAM_STATE_STOPED;
    LOG_INFO("Stop pipeline done!");
}

std::shared_ptr<IDevice> Pipeline::getDevice() {
    if(device_) {
        return device_;
    }
    return nullptr;
}

void Pipeline::configAlignMode() {
    // todo: implement this function
    // if(!config_) {
    //     return;
    // }
    // auto alignMode = config_->getAlignMode();
    // device_->configAlignMode(config_);
}

void Pipeline::resetAlignMode() {
    // todo: implement this function
    // if(device_) {
    //     device_->resetAlignMode();
    // }
}

StreamProfileList Pipeline::getD2CDepthProfileList(std::shared_ptr<const StreamProfile> colorProfile, OBAlignMode alignMode) {
    // todo: implement this function
    utils::unusedVar(colorProfile);
    utils::unusedVar(alignMode);
    return {};

    // StreamProfileList d2cDepthProfileList;
    // auto              videoProfile  = colorProfile->as<VideoStreamProfile>();
    // auto              depthProfiles = getStreamProfileList(OB_SENSOR_DEPTH);
    // if(alignMode == ALIGN_DISABLE) {
    //     return depthProfiles;
    // }

    // auto list = device_->getD2CSupportedProfileList();
    // for(auto sp: list) {
    //     if(videoProfile->getWidth() == sp.colorWidth && videoProfile->getHeight() == sp.colorHeight) {
    //         for(auto profile: depthProfiles) {
    //             auto depthProfile = profile->as<VideoStreamProfile>();
    //             if(sp.depthWidth == depthProfile->getWidth() && sp.depthHeight == depthProfile->getHeight()
    //                && ((alignMode == ALIGN_D2C_HW_MODE && (sp.alignType & ALIGN_D2C_HW)) || (alignMode == ALIGN_D2C_SW_MODE && (sp.alignType &
    //                ALIGN_D2C_SW))))

    //             {
    //                 d2cDepthProfileList.push_back(depthProfile);
    //             }
    //         }
    //     }
    // }

    // return d2cDepthProfileList;
}

void Pipeline::enableFrameSync() {
    auto devicePid = device_->getInfo()->pid_;
    if(std::find(gFrameSyncAbleDevPids.begin(), gFrameSyncAbleDevPids.end(), devicePid) == gFrameSyncAbleDevPids.end()) {
        throw libobsensor::unsupported_operation_exception("Current device does not support frame sync!");
        return;
    }
    frameAggregator_->enableFrameSync(true);
}

void Pipeline::disableFrameSync() {
    frameAggregator_->enableFrameSync(false);
}

std::shared_ptr<const VideoStreamProfile> Pipeline::getCurrentVideoStreamProfile(OBStreamType type) {
    if(!config_) {
        return {};
    }

    for(const auto &sp: config_->getEnabledStreamProfileList()) {
        if(sp->getType() == type) {  // todo: different type
            return sp->as<const VideoStreamProfile>();
        }
    }
    return {};
}

}  // namespace libobsensor
