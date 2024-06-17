#include "VideoSensor.hpp"
#include "exception/OBException.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/utils.hpp"
#include "stream/StreamProfile.hpp"
#include "frame/Frame.hpp"

namespace libobsensor {

VideoSensor::VideoSensor(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend)
    : SensorBase(owner, sensorType, backend) {
    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    if(!vsPort) {
        throw invalid_value_exception("Backend is not a valid IVideoStreamPort");
    }
    streamProfileList_ = vsPort->getStreamProfileList();

    for(auto &sp: streamProfileList_) {
        filteredStreamProfileList_.push_back(sp);
    }
}

#define MIN_VIDEO_FRAME_DATA_SIZE 1024
void VideoSensor::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    activatedStreamProfile_ = sp;
    frameCallback_          = callback;
    updateStreamState(STREAM_STATE_STARTING);

    currentBackendStreamProfile_ = sp;
    currentFormatFilterConfig_   = formatFilterConfigs_.end();
    auto iter                    = streamProfileFilterConfigMap_.find(sp);
    if(iter != streamProfileFilterConfigMap_.end()) {
        currentBackendStreamProfile_ = iter->second.first;
        currentFormatFilterConfig_   = iter->second.second;
        if(currentFormatFilterConfig_->converter) {
            currentFormatFilterConfig_->converter->setConversion(currentFormatFilterConfig_->srcFormat, currentFormatFilterConfig_->dstFormat);
        }
    }

    auto vsp              = currentBackendStreamProfile_->as<VideoStreamProfile>();
    auto maxFrameDataSize = vsp->getMaxFrameDataSize();

    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->startStream(currentBackendStreamProfile_, [this, maxFrameDataSize](std::shared_ptr<Frame> frame) {
        auto dataSize = frame->getDataSize();
        auto format   = frame->getFormat();

#ifdef OB_DEBUG
        auto fsp   = frame->getStreamProfile();
        auto owner = fsp->getOwner();
        if(fsp.get() != currentBackendStreamProfile_.get()) {
            throw invalid_value_exception("Frame's stream profile is not the same as activated stream profile");
        }
        if(owner.get() != this) {
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
            LOG_WARN_INTVL("This frame will be dropped because because the data size is larger than expected! size={}, expected={} @{}", dataSize,
                           maxFrameDataSize, sensorType_);
            return;
        }
        else if(IS_FIXED_SIZE_FORMAT(activatedStreamProfile_->getFormat()) && maxFrameDataSize != dataSize) {
            LOG_WARN_INTVL("This frame will be dropped because the data size does not match the expectation! size={}, expected={} @{}", dataSize,
                           maxFrameDataSize, sensorType_);
            return;
        }

        updateStreamState(STREAM_STATE_STREAMING);

        if(currentFormatFilterConfig_ != formatFilterConfigs_.end()) {
            if(currentFormatFilterConfig_->converter) {
                frame = currentFormatFilterConfig_->converter->process(frame);
            }
            frame->setStreamProfile(activatedStreamProfile_);
        }
        frameCallback_(frame);
    });
}

void VideoSensor::stop() {
    updateStreamState(STREAM_STATE_STOPPING);

    auto vsPort = std::dynamic_pointer_cast<IVideoStreamPort>(backend_);
    vsPort->stopStream(activatedStreamProfile_);

    activatedStreamProfile_.reset();
    frameCallback_ = nullptr;

    updateStreamState(STREAM_STATE_STOPED);
}

StreamProfileList VideoSensor::getStreamProfileList() const {
    return filteredStreamProfileList_;
};

void VideoSensor::updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs) {
    formatFilterConfigs_ = configs;
    auto backendSpList   = SensorBase::getStreamProfileList();
    filteredStreamProfileList_.clear();
    for(const auto &backendSp: backendSpList) {
        auto format = backendSp->getFormat();
        auto iter   = std::find_if(formatFilterConfigs_.begin(), formatFilterConfigs_.end(),
                                   [format](const FormatFilterConfig &config) { return config.srcFormat == format; });
        if(iter == formatFilterConfigs_.end()) {
            filteredStreamProfileList_.push_back(backendSp);
            continue;
        }

        if(iter->policy == FormatFilterPolicy::REMOVE) {
            continue;
        }
        else if(iter->policy == FormatFilterPolicy::ADD) {
            filteredStreamProfileList_.push_back(backendSp);
        }

        // FormatFilterPolicy：ADD or REPLACE
        auto newSp = backendSp->clone();
        newSp->setFormat(iter->dstFormat);
        filteredStreamProfileList_.push_back(newSp);  // add new format
        streamProfileFilterConfigMap_[newSp] = { backendSp, iter };
    }
    LOG_DEBUG(" filtered stream profile list size={} @{}", filteredStreamProfileList_.size(), sensorType_);
    for(auto &sp: filteredStreamProfileList_) {
        LOG_DEBUG(" - {}", sp);
    }
}

}  // namespace libobsensor