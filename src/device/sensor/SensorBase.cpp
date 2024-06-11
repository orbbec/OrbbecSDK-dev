
#include "SensorBase.hpp"
#include "utils/PublicTypeHelper.hpp"

namespace libobsensor {
SensorBase::SensorBase(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend)
    : sensorType_(sensorType),
      owner_(owner),
      backend_(backend),
      streamState_(STREAM_STATE_STOPED),
      recoveryEnabled_(false),
      onRecovering_(false),
      recoveryCount_(0),
      maxRecoveryCount_(DefaultMaxRecoveryCount),
      noStreamTimeoutMs_(DefaultNoStreamTimeoutMs),
      streamInterruptTimeoutMs_(DefaultStreamInterruptTimeoutMs){

      };

SensorBase::~SensorBase() noexcept {
    if(streamStateWatcherThread_.joinable()) {
        recoveryEnabled_ = false;
        streamStateCv_.notify_all();
        streamStateWatcherThread_.join();
    }
    LOG_INFO("SensorBase is destroyed");
}

OBSensorType SensorBase::getSensorType() const {
    return sensorType_;
}

std::shared_ptr<IDevice> SensorBase::getOwner() const {
    return owner_.lock();
}

std::shared_ptr<ISourcePort> SensorBase::getBackend() const {
    return backend_;
}

OBStreamState SensorBase::getStreamState() const {
    return streamState_.load();
}

bool SensorBase::isStreamActivated() const {
    return streamState_ == STREAM_STATE_STARTING || streamState_ == STREAM_STATE_STREAMING || streamState_ == STREAM_STATE_ERROR;
}

void SensorBase::setStreamStateChangedCallback(StreamStateChangedCallback callback) {
    streamStateChangedCallback_ = callback;
}

StreamProfileList SensorBase::getStreamProfileList() const{
    StreamProfileList list;
    auto streamType = utils::mapSensorTypeToStreamType(sensorType_);
    for(auto sp : streamProfileList_) {
        auto spOwner = sp->getOwner();
        if(!spOwner || spOwner->device.lock() != owner_.lock() || sp->getType() == streamType) {
            auto newOwner = std::make_shared<LazySensor>(owner_, sensorType_);
            sp->bindOwner(newOwner);
            sp->setType(streamType);
        }
        list.push_back(sp);
    }
    return list;
}

std::shared_ptr<const StreamProfile> SensorBase::getActivatedStreamProfile() const {
    return activatedStreamProfile_;
}

FrameCallback SensorBase::getFrameCallback() const {
    return  frameCallback_;
}

void SensorBase::restartStream() {
    auto curSp = activatedStreamProfile_;
    auto callback = frameCallback_;
    stop();
    start(curSp, callback);
}

void SensorBase::updateStreamState(OBStreamState state) {
    std::unique_lock<std::mutex> lock(streamStateMutex_);
    if(onRecovering_) {
        return;
    }
    auto oldState = streamState_.load();
    streamState_.store(state);
    if( oldState != state && streamStateChangedCallback_) {
        streamStateChangedCallback_(state, activatedStreamProfile_);  // call the callback function
    }
    streamStateCv_.notify_all();
}

void SensorBase::enableStreamRecovery(bool enable, uint32_t maxRecoveryCount, int noStreamTimeoutMs, int streamInterruptTimeoutMs) {
    {
        std::unique_lock<std::mutex> lock(streamStateMutex_);
        recoveryCount_          = 0;
        recoveryEnabled_        = enable;
        maxRecoveryCount_       = maxRecoveryCount == 0 ? maxRecoveryCount_ : maxRecoveryCount;
        noStreamTimeoutMs_        = noStreamTimeoutMs == 0 ? noStreamTimeoutMs_ : noStreamTimeoutMs;
        streamInterruptTimeoutMs_ = streamInterruptTimeoutMs == 0 ? streamInterruptTimeoutMs_ : streamInterruptTimeoutMs;
    }
    if(streamStateWatcherThread_.joinable()) {
        return;
    }
    streamStateWatcherThread_ = std::thread([this]() { watchStreamState(); });
}

void SensorBase::disableStreamRecovery() {
    recoveryEnabled_ = false;
    streamStateCv_.notify_all();
    if(streamStateWatcherThread_.joinable()) {
        streamStateWatcherThread_.join();
    }
}

void SensorBase::watchStreamState() {
    recoveryCount_ = 0;
    while(recoveryEnabled_) {
        std::unique_lock<std::mutex> lock(streamStateMutex_);
        if(streamState_ == STREAM_STATE_STOPED || streamState_ == STREAM_STATE_STOPPING || streamState_ == STREAM_STATE_ERROR) {
            streamStateCv_.wait(lock);
            recoveryCount_ = 0;
        }
        else if(streamState_ == STREAM_STATE_STARTING && noStreamTimeoutMs_ > 0 ) {
            streamStateCv_.wait_for(lock, std::chrono::milliseconds(noStreamTimeoutMs_));
            if(streamState_ != STREAM_STATE_STARTING || recoveryEnabled_ == false) {
                recoveryCount_ = 0;
                continue;
            }
            if(recoveryCount_ < maxRecoveryCount_) {
                onRecovering_ = true;
                TRY_EXECUTE(restartStream());
                recoveryCount_++;
                onRecovering_ = false;
                continue;
            }
            updateStreamState(STREAM_STATE_ERROR);
            LOG_ERROR("Failed to start stream for sensor after {} retries", maxRecoveryCount_);
        }
        else if(streamState_ == STREAM_STATE_STREAMING && streamInterruptTimeoutMs_ > 0) {
            auto sts = streamStateCv_.wait_for(lock, std::chrono::milliseconds(streamInterruptTimeoutMs_));
            if(sts != std::cv_status::timeout || streamState_ != STREAM_STATE_STREAMING || recoveryEnabled_ == false) {
                recoveryCount_ = 0;
                continue;
            }
            if(recoveryCount_ < maxRecoveryCount_) {
                onRecovering_ = true;
                TRY_EXECUTE(restartStream());
                recoveryCount_++;
                onRecovering_ = false;
                continue;
            }
            updateStreamState(STREAM_STATE_ERROR);
            LOG_ERROR("Failed to recover stream for sensor after {} retries", maxRecoveryCount_);
        }
    }
}
}  // namespace libobsensor