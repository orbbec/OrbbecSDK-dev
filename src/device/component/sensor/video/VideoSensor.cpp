#include "VideoSensor.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "logger/LoggerHelper.hpp"
#include "utils/Utils.hpp"
#include "stream/StreamProfile.hpp"
#include "frame/Frame.hpp"

namespace libobsensor {

VideoSensor::VideoSensor(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend)
    : SensorBase(owner, sensorType, backend) {
    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    if(!vsPort) {
        throw invalid_value_exception("Backend is not a valid IVideoStreamPort");
    }

    auto lazySelf   = std::make_shared<LazySensor>(owner, sensorType_);
    auto streamType = utils::mapSensorTypeToStreamType(sensorType_);

    auto backendSpList = vsPort->getStreamProfileList();
    for(auto &backendSp: backendSpList) {
        auto sp = backendSp->clone();
        sp->bindOwner(lazySelf);
        sp->setType(streamType);
        backendStreamProfileList_.push_back(sp);
    }

    for(auto &backendSp: backendStreamProfileList_) {
        auto sp = backendSp->clone();
        sp->bindOwner(lazySelf);
        sp->setType(streamType);
        streamProfileList_.push_back(sp);
        streamProfileBackendMap_[sp] = { backendSp, formatFilterConfigs_.end() };
    }
}

#define MIN_VIDEO_FRAME_DATA_SIZE 1024
void VideoSensor::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    activatedStreamProfile_ = sp;
    frameCallback_          = callback;
    updateStreamState(STREAM_STATE_STARTING);

    auto backendIter = streamProfileBackendMap_.find(sp);
    if(backendIter == streamProfileBackendMap_.end()) {
        throw invalid_value_exception("Can not find backend stream profile for activated stream profile");
    }
    currentBackendStreamProfile_ = backendIter->second.first;
    currentFormatFilterConfig_   = backendIter->second.second;

    if(currentFormatFilterConfig_ != formatFilterConfigs_.end() && currentFormatFilterConfig_->converter) {
        currentFormatFilterConfig_->converter->setConversion(currentFormatFilterConfig_->srcFormat, currentFormatFilterConfig_->dstFormat);
        currentFormatFilterConfig_->converter->setCallback(callback);
    }

    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->startStream(currentBackendStreamProfile_, [this](std::shared_ptr<Frame> frame) { onBackendFrameCallback(frame); });
}

void VideoSensor::onBackendFrameCallback(std::shared_ptr<Frame> frame) {
    auto vsp              = currentBackendStreamProfile_->as<VideoStreamProfile>();
    auto maxFrameDataSize = vsp->getMaxFrameDataSize();

    auto dataSize = frame->getDataSize();
    auto format   = frame->getFormat();

#ifdef OB_DEBUG
    auto fsp   = frame->getStreamProfile();
    auto owner = fsp->getOwner();
    if(fsp.get() != currentBackendStreamProfile_.get()) {
        throw invalid_value_exception("Frame's stream profile is not the same as activated stream profile");
    }
    if(owner.get() != static_cast<void *>(this)) {
        throw invalid_value_exception("Frame's owner is not this VideoSensor");
    }
#endif

    if(format == OB_FORMAT_MJPG && frame->getDataSize() < MIN_VIDEO_FRAME_DATA_SIZE) {
        LOG_WARN_INTVL("This frame will be dropped because data size less than mini size (1024 byte)! size={} @{}", dataSize, sensorType_);
        return;
    }
    else if(format == OB_FORMAT_MJPG && sensorType_ != OB_SENSOR_DEPTH && !utils::checkJpgImageData(frame->getData(), dataSize)) {
        LOG_WARN_INTVL("This frame will be dropped because jpg format verification failure! @{}", sensorType_);
        return;
    }
    else if(maxFrameDataSize < dataSize) {
        LOG_WARN_INTVL("This frame will be dropped because because the data size is larger than expected! size={}, expected={} @{}", dataSize, maxFrameDataSize,
                       sensorType_);
        return;
    }
    else if(IS_FIXED_SIZE_FORMAT(activatedStreamProfile_->getFormat()) && maxFrameDataSize != dataSize) {
        LOG_WARN_INTVL("This frame will be dropped because the data size does not match the expectation! size={}, expected={} @{}", dataSize, maxFrameDataSize,
                       sensorType_);
        return;
    }

    updateStreamState(STREAM_STATE_STREAMING);
    if(frameMetadataParserContainer_) {
        frame->registerMetadataParsers(frameMetadataParserContainer_);
    }

    if(frameTimestampCalculator_) {
        frameTimestampCalculator_->calculate(frame);
    }

    if(currentFormatFilterConfig_ != formatFilterConfigs_.end()) {
        if(currentFormatFilterConfig_->converter) {
            frame = currentFormatFilterConfig_->converter->process(frame);
        }
    }
    frame->setStreamProfile(activatedStreamProfile_);
    outputFrame(frame);
    LOG_FREQ_CALC(INFO, 5000, "{} Streaming... frameRate={freq}fps", sensorType_);
}

void VideoSensor::outputFrame(std::shared_ptr<Frame> frame) {
    if(frameProcessor_) {
        frame = frameProcessor_->process(frame);
    }
    frameCallback_(frame);
}

void VideoSensor::stop() {
    updateStreamState(STREAM_STATE_STOPPING);

    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->stopStream(currentBackendStreamProfile_);

    activatedStreamProfile_.reset();
    frameCallback_ = nullptr;

    updateStreamState(STREAM_STATE_STOPED);
}

void VideoSensor::updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs) {
    if(isStreamActivated()) {
        throw wrong_api_call_sequence_exception("Can not update format filter config while streaming");
    }
    formatFilterConfigs_ = configs;
    streamProfileList_.clear();
    auto lazySelf   = std::make_shared<LazySensor>(owner_, sensorType_);
    auto streamType = utils::mapSensorTypeToStreamType(sensorType_);
    for(const auto &backendSp: backendStreamProfileList_) {
        auto format = backendSp->getFormat();
        auto iter   = std::find_if(formatFilterConfigs_.begin(), formatFilterConfigs_.end(),
                                   [format](const FormatFilterConfig &config) { return config.srcFormat == format; });
        if(iter == formatFilterConfigs_.end()) {
            auto sp = backendSp->clone();
            sp->bindOwner(lazySelf);
            sp->setType(streamType);
            streamProfileList_.push_back(sp);
            streamProfileBackendMap_[sp] = { backendSp, formatFilterConfigs_.end() };
            continue;
        }

        if(iter->policy == FormatFilterPolicy::REMOVE) {
            continue;
        }
        else if(iter->policy == FormatFilterPolicy::ADD) {
            auto sp = backendSp->clone();
            sp->bindOwner(lazySelf);
            sp->setType(streamType);
            streamProfileList_.push_back(sp);
            streamProfileBackendMap_[sp] = { backendSp, formatFilterConfigs_.end() };
        }

        // FormatFilterPolicy：ADD or REPLACE
        auto sp = backendSp->clone();
        sp->setFormat(iter->dstFormat);
        sp->bindOwner(lazySelf);
        sp->setType(streamType);
        streamProfileList_.push_back(sp);
        streamProfileBackendMap_[sp] = { backendSp, iter };
    }

    LOG_DEBUG(" filtered stream profile list size={} @{}", streamProfileList_.size(), sensorType_);
    for(auto &sp: streamProfileList_) {
        LOG_DEBUG(" - {}", sp);
    }
}

void VideoSensor::setFrameMetadataParserContainer(std::shared_ptr<IFrameMetadataParserContainer> container) {
    if(isStreamActivated()) {
        throw wrong_api_call_sequence_exception("Can not update frame metadata parser container while streaming");
    }
    frameMetadataParserContainer_ = container;
}

void VideoSensor::setFrameTimestampCalculator(std::shared_ptr<IFrameTimestampCalculator> calculator) {
    if(isStreamActivated()) {
        throw wrong_api_call_sequence_exception("Can not update frame timestamp calculator while streaming");
    }
    frameTimestampCalculator_ = calculator;
}

void VideoSensor::setFrameProcessor(std::shared_ptr<FrameProcessor> frameProcessor) {
    if(isStreamActivated()) {
        throw wrong_api_call_sequence_exception("Can not update frame processor while streaming");
    }
    frameProcessor_ = frameProcessor;
}

}  // namespace libobsensor