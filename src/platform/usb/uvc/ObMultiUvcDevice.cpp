#include <common/encryption/TEA.h>
#include <common/exception/ObException.hpp>
#include "ObMultiUvcDevice.hpp"

#ifdef WIN32
#include "WmfUvcDevicePort.hpp"
#else
#include "ObLibuvcDevicePort.hpp"
#endif  //

#include "common/logger/Logger.hpp"

#define DEPTH_STREAM_DATA_TAG (0xde)
#define IR_STREAM_DATA_TAG (0xa2)

namespace libobsensor {
namespace pal {

ObMultiUvcDevice::ObMultiUvcDevice(std::shared_ptr<UsbDevice> usbDev, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : streamStart_(false), timestampOffsetFlag_(false),frameCallbacks_(10) {

#ifdef WIN32
    depthUvcPort_ = std::make_shared<WmfUvcDevicePort>(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
#else
    depthUvcPort_ = std::make_shared<ObLibuvcDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
#endif
}

ObMultiUvcDevice::~ObMultiUvcDevice() noexcept {
    stopAllStream();
}

std::shared_ptr<const SourcePortInfo> ObMultiUvcDevice::getSourcePortInfo(void) const {

    if(depthUvcPort_) {
        return depthUvcPort_->getSourcePortInfo();
    }

    return nullptr;
}

std::vector<std::shared_ptr<const VideoStreamProfile>> ObMultiUvcDevice::getStreamProfileList() {
    return depthUvcPort_->getStreamProfileList();
}

void ObMultiUvcDevice::onMultiFrameRawDataCallback(pal::VideoFrameObject fo) {
    std::unique_lock<std::recursive_mutex> streamLock(multiStreamMutex_);
    if(!streamStart_) {
        return;
    }

    if(fo.scrDataSize > 0 && fo.scrDataBuf != nullptr) {

        uint8_t *srcData   = (uint8_t *)fo.scrDataBuf;
        uint8_t  frameType = srcData[0];
        if(srcData) {
            if(frameType == DEPTH_STREAM_DATA_TAG) {
                auto callback = frameCallbacks_[OB_STREAM_DEPTH];
                if(callback) {
                    // if(timestampOffsetFlag_){
                    //    fo.deviceTime = fo.deviceTime + (1000/profile_->fps) * 1000;
                    //}
                    // LOG(ERROR) << "Depth device time:" << fo.deviceTime;
                    callback(fo);
                }
            }
            else if(frameType == IR_STREAM_DATA_TAG) {
                auto callback = frameCallbacks_[OB_STREAM_IR_RIGHT];
                if(callback) {
                    // LOG(ERROR) << "Right IR device time:" << fo.deviceTime;
                    callback(fo);
                }
            }
        }
    }
}

void ObMultiUvcDevice::startStream(std::shared_ptr<const VideoStreamProfile> profile, FrameCallbackUnsafe callback) {
    std::unique_lock<std::recursive_mutex> streamLock(multiStreamMutex_);
    if(profile->streamType == OB_STREAM_DEPTH) {
        profile_ = profile;
    }

    if(depthUvcPort_) {
        if(streamStart_) {
            frameCallbacks_.at(profile->streamType) = callback;
            return;
        }
        depthUvcPort_->startStream(profile, [&](pal::VideoFrameObject fo) { onMultiFrameRawDataCallback(fo); });
        frameCallbacks_.at(profile->streamType) = callback;
        streamStart_                         = true;
    }
}

void ObMultiUvcDevice::stopStream(std::shared_ptr<const VideoStreamProfile> profile) {
    std::unique_lock<std::recursive_mutex> streamLock(multiStreamMutex_);
    if(depthUvcPort_ && streamStart_) {
        frameCallbacks_.at(profile->streamType) = nullptr;

        bool allStreamStop = true;
        for(auto iter = frameCallbacks_.begin(); iter != frameCallbacks_.end(); iter++) {
            if((*iter) != nullptr) {
                allStreamStop = false;
                break;
            }
        }

        if(allStreamStop) {
            streamStart_ = false;
            depthUvcPort_->stopStream(profile);
        }
    }
}

void ObMultiUvcDevice::stopAllStream() {
    std::unique_lock<std::recursive_mutex> streamLock(multiStreamMutex_);
    if(depthUvcPort_ && streamStart_) {
        streamStart_ = false;
        depthUvcPort_->stopAllStream();
    }
}

bool ObMultiUvcDevice::getPu(OBPropertyID propertyId, int32_t &value) {
    if(depthUvcPort_) {
        return depthUvcPort_->getPu(propertyId, value);
    }

    return false;
}

bool ObMultiUvcDevice::setPu(OBPropertyID propertyId, int32_t value) {
    if(depthUvcPort_) {
        depthUvcPort_->setPu(propertyId, value);
    }

    return false;
}

ControlRange ObMultiUvcDevice::getPuRange(OBPropertyID propertyId) {
    return depthUvcPort_->getPuRange(propertyId);
}

bool ObMultiUvcDevice::sendData(const uint8_t *data, const uint32_t dataLen) {
    if(depthUvcPort_) {
        return depthUvcPort_->sendData(data, dataLen);
    }

    return false;
}

bool ObMultiUvcDevice::recvData(uint8_t *data, uint32_t *dataLen) {
    if(depthUvcPort_) {
        return depthUvcPort_->recvData(data, dataLen);
    }

    return false;
}

#ifdef __ANDROID__
std::string ObMultiUvcDevice::getUsbConnectType() {
    if(depthUvcPort_) {
        return depthUvcPort_->getUsbConnectType();
    }
    throw new libobsensor::invalid_value_exception("depthUvcPort_ is null!");
}
#endif

bool ObMultiUvcDevice::getStreamStarted() {
    return streamStart_;
}

}  // namespace pal

}  // namespace libobsensor
