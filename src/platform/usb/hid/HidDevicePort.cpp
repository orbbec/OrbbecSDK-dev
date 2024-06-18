// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "HidDevicePort.hpp"

#include "frame/FrameFactory.hpp"
#include "logger/Logger.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
HidDevicePort::HidDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : portInfo_(portInfo), usbDevice_(usbDevice), isStreaming_(false), frameQueue_(10) {
    LOG_DEBUG("obHid Device open info_.infIndex={}", static_cast<uint32_t>(portInfo_->infIndex));

    auto intf = usbDevice_->getInterface(portInfo_->infIndex);
    if(!intf) {
        LOG_ERROR("interface not found!");
        return;
    }

    readEndpoint_ = intf->firstEndpoint(OB_USB_ENDPOINT_DIRECTION_READ, OB_USB_ENDPOINT_INTERRUPT);
    if(!readEndpoint_) {
        LOG_ERROR("read endpoint not found!");
        return;
    }

    messenger_ = usbDevice_->open(portInfo_->infIndex);
    if(!messenger_) {
        LOG_ERROR("open hid device failed!");
        return;
    }

    interruptCallback_ = std::make_shared<UsbRequestCallback>([&](obUsbRequest response) {
        if(isStreaming_) {
            auto datasize = response->getActualLength();
            if(response->getActualLength() > 0) {
                auto frame = FrameFactory::createFrame(OB_FRAME_UNKNOWN, OB_FORMAT_UNKNOWN, datasize);
                frame->updateData(response->getBuffer().data(), datasize);
                frameQueue_.enqueue(frame);
            }
            std::unique_lock<std::mutex> lock(messengerMutex_);
            messenger_->submitRequest(interruptRequest_);
        }
    });

    std::unique_lock<std::mutex> lock(messengerMutex_);
    interruptRequest_ = messenger_->createRequest(readEndpoint_);
    interruptRequest_->setBuffer(std::vector<uint8_t>(readEndpoint_->getMaxPacketSize()));
    interruptRequest_->setCallback(interruptCallback_);
}

HidDevicePort::~HidDevicePort() noexcept {
    LOG_DEBUG("HidDevicePort::~HidDevicePort()");
    if(isStreaming_) {
        isStreaming_ = false;
        std::unique_lock<std::mutex> lock(messengerMutex_);
        messenger_->cancelRequest(interruptRequest_);
        utils::sleepMs(100);  // todo: wait for cancel request to complete
    }
    interruptCallback_->cancel();
    interruptRequest_.reset();
    messenger_.reset();

    frameQueue_.stop();
    LOG_DEBUG("obHidDevice destroy");
}

void HidDevicePort::startStream(FrameCallbackUnsafe callback) {
    if(isStreaming_) {
        throw wrong_api_call_sequence_exception("HidDevicePort::startStream() called while streaming");
    }
    isStreaming_ = true;

    frameQueue_.start(callback);

    auto sts = messenger_->submitRequest(interruptRequest_);
    if(sts != OB_USB_STATUS_SUCCESS) {
        frameQueue_.stop();
        isStreaming_ = false;
        throw std::runtime_error("failed to submit interrupt request, error: " + usbStatusToString.at(sts));
    }

    LOG_DEBUG("HidDevicePort::stopCapture done");
}

void HidDevicePort::stopStream() {
    if(!isStreaming_) {
        throw wrong_api_call_sequence_exception("HidDevicePort::stopStream() called while not streaming");
    }
    isStreaming_ = false;
    std::unique_lock<std::mutex> lock(messengerMutex_);
    messenger_->cancelRequest(interruptRequest_);
    frameQueue_.flush();
}

std::shared_ptr<const SourcePortInfo> HidDevicePort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace libobsensor
