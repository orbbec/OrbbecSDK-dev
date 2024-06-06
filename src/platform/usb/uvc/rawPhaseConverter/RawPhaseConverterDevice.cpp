#include "RawPhaseConverterDevice.hpp"
#include "common/logger/Logger.hpp"
#include <common/exception/ObException.hpp>

namespace libobsensor {
namespace pal {

RawPhaseConverterDevice::RawPhaseConverterDevice(std::shared_ptr<const USBSourcePortInfo> portInfo)
    : srcUvcPort_(nullptr), profile_(nullptr), streamStart_(false) {}

RawPhaseConverterDevice::~RawPhaseConverterDevice() noexcept {
    TRY_EXECUTE({ stopAllStream(); })
}

void RawPhaseConverterDevice::onFrameRawDataCallback(VideoFrameObject fo) {
    // std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    std::unique_lock<std::recursive_mutex> lk(streamMutex_, std::defer_lock);
    if(lk.try_lock()) {
        if(!streamStart_) {
            return;
        }

        processFrame(fo);
    }
}

std::shared_ptr<const SourcePortInfo> RawPhaseConverterDevice::getSourcePortInfo(void) const {

    if(srcUvcPort_) {
        return srcUvcPort_->getSourcePortInfo();
    }

    return nullptr;
}

std::vector<std::shared_ptr<const VideoStreamProfile>> RawPhaseConverterDevice::getStreamProfileList() {
    return srcUvcPort_->getStreamProfileList();
}

void RawPhaseConverterDevice::startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    srcUvcPort_->startStream(profile, callback);

    profile_ = profile;

    if(srcUvcPort_) {
        if(streamStart_) {
            frameCallbacks_[profile->streamType] = callback;
            return;
        }
        srcUvcPort_->startStream(profile, [&](pal::VideoFrameObject fo) { onFrameRawDataCallback(fo); });
        frameCallbacks_[profile->streamType] = callback;
        streamStart_                         = true;
    }
}

void RawPhaseConverterDevice::stopStream(std::shared_ptr<const VideoStreamProfile> profile) {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    if(srcUvcPort_ && streamStart_) {
        std::map<OBStreamType, VideoFrameCallback>::iterator iter = frameCallbacks_.begin();
        while(iter != frameCallbacks_.end()) {
            if((*iter).first == profile->streamType) {
                iter = frameCallbacks_.erase(iter);
                continue;
            }
            iter++;
        }

        if(frameCallbacks_.empty()) {
            streamStart_ = false;
            srcUvcPort_->stopStream(profile);
        }
    }
}

void RawPhaseConverterDevice::stopAllStream() {
    std::unique_lock<std::recursive_mutex> streamLock(streamMutex_);
    if(srcUvcPort_ && streamStart_) {
        streamStart_ = false;
        frameCallbacks_.clear();
        srcUvcPort_->stopAllStream();
    }
}

bool RawPhaseConverterDevice::getPu(OBPropertyID propertyId, int32_t &value) {
    if(srcUvcPort_) {
        return srcUvcPort_->getPu(propertyId, value);
    }

    return false;
}

bool RawPhaseConverterDevice::setPu(OBPropertyID propertyId, int32_t value) {
    if(srcUvcPort_) {
        return srcUvcPort_->setPu(propertyId, value);
    }
    return false;
}

ControlRange RawPhaseConverterDevice::getPuRange(OBPropertyID propertyId) {
    if(srcUvcPort_) {
        return srcUvcPort_->getPuRange(propertyId);
    }
    return ControlRange();
}

bool RawPhaseConverterDevice::sendData(const uint8_t *data, const uint32_t dataLen) {
    if(srcUvcPort_) {
        return srcUvcPort_->sendData(data, dataLen);
    }
    return false;
}

bool RawPhaseConverterDevice::recvData(uint8_t *data, uint32_t *dataLen) {
    if(srcUvcPort_) {
        return srcUvcPort_->recvData(data, dataLen);
    }
    return false;
}

#ifdef __ANDROID__
std::string RawPhaseConverterDevice::getUsbConnectType() {
    if(srcUvcPort_) {
        return srcUvcPort_->getUsbConnectType();
    }
    throw new libobsensor::invalid_value_exception("srcUvcPort_ is null!");
}
#endif

std::vector<std::shared_ptr<const VideoStreamProfile>> RawPhaseConverterDevice::getRawPhaseStreamProfileList() {
    std::vector<std::shared_ptr<const VideoStreamProfile>> profileList;
    if(srcUvcPort_) {
        profileList = srcUvcPort_->getStreamProfileList();
    }
    return profileList;
}

bool RawPhaseConverterDevice::getStreamStarted() {
    return streamStart_;
}

}  // namespace pal

}  // namespace libobsensor
