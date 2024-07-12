// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "HidDevicePort.hpp"

#include "frame/FrameFactory.hpp"
#include "logger/Logger.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/Utils.hpp"
#include "usb/enumerator/Enumerator.hpp"

namespace libobsensor {
HidDevicePort::HidDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : portInfo_(portInfo), usbDevice_(usbDevice), isStreaming_(false), frameQueue_(10) {
    endpoint_ = UsbEnumerator::getEndpointAddress(usbDevice, portInfo->infIndex, LIBUSB_ENDPOINT_TRANSFER_TYPE_INTERRUPT, LIBUSB_ENDPOINT_IN);
    if(libusb_kernel_driver_active(usbDevice->devHandle.get(), portInfo->infIndex) == 1) {
        auto res = libusb_detach_kernel_driver(usbDevice->devHandle.get(), portInfo->infIndex);
        if(res != LIBUSB_SUCCESS) {
            throw io_exception("detach kernel driver failed, error: " + std::string(libusb_strerror(res)));
        }
    }
    auto res = libusb_claim_interface(usbDevice->devHandle.get(), portInfo->infIndex);
    if(res != LIBUSB_SUCCESS) {
        throw io_exception("claim interface failed, error: " + std::string(libusb_strerror(res)));
    }
    LOG_DEBUG("HidDevicePort::HidDevicePort done");
}

HidDevicePort::~HidDevicePort() noexcept {
    if(isStreaming_) {
        frameQueue_.stop();
        stopStream();
    }
}

void HidDevicePort::startStream(FrameCallbackUnsafe callback) {
    if(isStreaming_) {
        throw wrong_api_call_sequence_exception("HidDevicePort::startStream() called while streaming");
    }
    isStreaming_ = true;
    frameQueue_.start(callback);

    streamThread_ = std::thread([this]() {
        while(isStreaming_) {
            auto frame = FrameFactory::createFrame(OB_FRAME_UNKNOWN, OB_FORMAT_UNKNOWN, endpoint_.wMaxPacketSize);
            int  transferred;
            auto res = libusb_interrupt_transfer(usbDevice_->devHandle.get(), endpoint_.bEndpointAddress, frame->getDataMutable(), frame->getDataSize(),
                                                 &transferred, 1000);
            if(res != LIBUSB_SUCCESS && isStreaming_) {
                LOG_WARN_INTVL(utils::string::to_string() << "interrupt transfer failed, error: " << libusb_strerror(res));
                continue;
            }
            frameQueue_.enqueue(frame);
        }
    });
    LOG_DEBUG("HidDevicePort::startStream done");
}

void HidDevicePort::stopStream() {
    if(!isStreaming_) {
        throw wrong_api_call_sequence_exception("HidDevicePort::stopStream() called while not streaming");
    }
    isStreaming_ = false;
    auto res     = libusb_interrupt_transfer(usbDevice_->devHandle.get(), endpoint_.bEndpointAddress, nullptr, 0, nullptr, 0);
    if(res != LIBUSB_SUCCESS) {
        LOG_WARN("interrupt transfer failed, error: {}", libusb_strerror(res));
    }

    streamThread_.join();
    frameQueue_.flush();

    res = libusb_release_interface(usbDevice_->devHandle.get(), portInfo_->infIndex);
    if(res != LIBUSB_SUCCESS) {
        LOG_WARN("release interface failed, error: {}", libusb_strerror(res));
    }
    if(libusb_kernel_driver_active(usbDevice_->devHandle.get(), portInfo_->infIndex) == 1) {
        res = libusb_attach_kernel_driver(usbDevice_->devHandle.get(), portInfo_->infIndex);
        if(res != LIBUSB_SUCCESS) {
            LOG_WARN("attach kernel driver failed, error: {}", libusb_strerror(res));
        }
    }
    LOG_DEBUG("HidDevicePort::stopStream done");
}

std::shared_ptr<const SourcePortInfo> HidDevicePort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace libobsensor
