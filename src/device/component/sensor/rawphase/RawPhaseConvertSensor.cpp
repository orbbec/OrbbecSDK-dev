#include "RawPhaseConvertSensor.hpp"
#include "IDevice.hpp"
#include "stream/StreamProfileFactory.hpp"

namespace libobsensor {
RawPhaseConvertSensor::RawPhaseConvertSensor(IDevice *owner, const std::shared_ptr<ISourcePort> &backend, OBSensorType sensorType,
                                             const std::shared_ptr<RawPhaseStreamer> &streamer)
    : VideoSensor(owner, sensorType, backend), streamer_(streamer), sensorType_(sensorType) {
    profileVector_.push_back({ { 320, 288 }, { 7680, 434 } });
    profileVector_.push_back({ { 640, 576 }, { 7680, 434 } });
    profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
    profileVector_.push_back({ { 512, 512 }, { 8192, 290 } });
    auto propServer = owner->getPropertyServer();
    updateStreamProfileList();
    LOG_DEBUG("RawPhaseConvertSensor is created!");
}

// void RawPhaseConvertSensor::setIsPassiveIR(bool isPassiveIR) {
//     isPassiveIR_ = isPassiveIR;

//     for(auto it = profileVector_.begin(); it != profileVector_.end();) {
//         if(it->first.first == 1024 && it->first.second == 1024) {
//             it = profileVector_.erase(it);
//         }
//         else {
//             ++it;
//         }
//     }

//     if(isPassiveIR) {
//         profileVector_.push_back({ { 1024, 1024 }, { 8192, 130 } });
//     }
//     else {
//         profileVector_.push_back({ { 1024, 1024 }, { 4096, 1154 } });
//     }
//     streamProfileList_.clear();

//     streamProfileList_ = getStreamProfileList();
//     updateStreamProfileList();
// }

RawPhaseConvertSensor::~RawPhaseConvertSensor() noexcept {
    if(isStreamActivated()) {
        TRY_EXECUTE(stop());
    }
    LOG_DEBUG("RawPhaseConvertSensor is destroyed!");
}

void RawPhaseConvertSensor::updateStreamProfileList() {
    auto oldSp = streamProfileList_;
    streamProfileList_.clear();
    for(auto &sp: oldSp) {
        auto profile    = std::dynamic_pointer_cast<const VideoStreamProfile>(sp);
        auto resolution = std::make_pair(profile->getWidth(), profile->getHeight());
        auto iter       = std::find_if(
            profileVector_.begin(), profileVector_.end(),
            [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.first == resolution; });
        auto valueIter = std::find_if(
            profileVector_.begin(), profileVector_.end(),
            [resolution](const std::pair<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> &pair) { return pair.second == resolution; });
        if(iter == profileVector_.end() && valueIter != profileVector_.end()) {

            auto newSp = StreamProfileFactory::createVideoStreamProfile(profile->getType(), profile->getFormat(), (*valueIter).first.first,
                                                                        (*valueIter).first.second, profile->getFps());
            streamProfileList_.push_back(newSp);

            if((*valueIter).first.first == 320 && (*valueIter).first.second == 288) {
                auto newSp1 = StreamProfileFactory::createVideoStreamProfile(profile->getType(), profile->getFormat(), 640, 576, profile->getFps());
                streamProfileList_.push_back(newSp1);
            }
        }
    }
}

void RawPhaseConvertSensor::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    activatedStreamProfile_ = sp;
    frameCallback_          = callback;
    updateStreamState(STREAM_STATE_STARTING);
    auto owner      = getOwner();
    auto propServer = owner->getPropertyServer();

    if(sensorType_ == OB_SENSOR_DEPTH || sensorType_ == OB_SENSOR_IR) {
        auto streamSp = sp->as<VideoStreamProfile>();
        streamer_->start(streamSp, [this](std::shared_ptr<const Frame> frame) {
            updateStreamState(STREAM_STATE_STREAMING);
            if(frameCallback_) {
                frameCallback_(frame);
            }
        });
    }
}

void RawPhaseConvertSensor::stop() {
    updateStreamState(STREAM_STATE_STOPPING);
    auto owner      = getOwner();
    auto propServer = owner->getPropertyServer();
    streamer_->stop(activatedStreamProfile_);
    updateStreamState(STREAM_STATE_STOPPED);
}
}  // namespace libobsensor