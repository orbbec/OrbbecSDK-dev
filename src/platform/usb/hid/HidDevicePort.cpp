// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "HidDevicePort.hpp"

#include "logger/Logger.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
namespace pal {

const int INTERRUPT_BUFFER_SIZE = 256;
HidDevicePort::HidDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : portInfo_(portInfo), usbDevice_(usbDevice), isStreaming_(false) {
    LOG_DEBUG("obHid Device open info_.infIndex={}", (uint32_t)portInfo_->infIndex);
    auto intf = usbDevice_->getInterface(portInfo_->infIndex);

    if(!intf) {
        return;
    }
    readEndpoint_ = intf->firstEndpoint(OB_USB_ENDPOINT_DIRECTION_READ, OB_USB_ENDPOINT_INTERRUPT);
    if(!readEndpoint_) {
        return;
    }

    messenger_ = usbDevice_->open(portInfo_->infIndex);

    if(messenger_) {
        interruptCallback_ = std::make_shared<UsbRequestCallback>([&](obUsbRequest response) {
            if(response->getActualLength() > 0) {
                std::vector<std::weak_ptr<DataStreamWatcher>> tempWatchers;
                {
                    // 避免对watchers_加锁锁住item->onDataReceived的调用过程
                    std::unique_lock<std::mutex> lk(watchersMutex_);
                    tempWatchers = watchers_;
                }
                for(auto &item_weak_ptr: tempWatchers) {
                    auto item = item_weak_ptr.lock();
                    if(item) {
                        item->onDataReceived(response->getBuffer().data(), response->getActualLength());
                    }
                }
            }
            std::unique_lock<std::mutex> lk(messengerMutex_);
            if(!messenger_ && isStreaming_)
                return;
            messenger_->submitRequest(interruptRequest_);
        });
        std::unique_lock<std::mutex> lk(messengerMutex_);
        interruptRequest_ = messenger_->createRequest(readEndpoint_);
        interruptRequest_->setBuffer(std::vector<uint8_t>(readEndpoint_->getMaxPacketSize()));
        interruptRequest_->setCallback(interruptCallback_);
    }
    else {
        LOG_ERROR("open hid device failed!");
    }
}

HidDevicePort::~HidDevicePort() noexcept {
    LOG_DEBUG("HidDevicePort::~HidDevicePort()");

    {
        std::unique_lock<std::mutex> lk(watchersMutex_);
        watchers_.clear();
    }

    if(isStreaming_) {
        isStreaming_ = false;
        std::unique_lock<std::mutex> lk(messengerMutex_);
        messenger_->cancelRequest(interruptRequest_);
        utils::sleepMs(100);
    }
    interruptCallback_->cancel();
    interruptRequest_.reset();
    messenger_.reset();

    LOG_DEBUG("obHidDevice destroy");
}

void HidDevicePort::removeWatcher(std::shared_ptr<DataStreamWatcher> watcher) {
    LOG_DEBUG("HidDevicePort::removeWatcher");
    bool watchersEmpty = false;
    {
        std::unique_lock<std::mutex> lk(watchersMutex_);
        for(auto iter = watchers_.begin(); iter != watchers_.end();) {
            auto item = iter->lock();
            if(!item || item == watcher) {
                iter = watchers_.erase(iter);
            }
            else {
                ++iter;
            }
        }
        watchersEmpty = watchers_.empty();
    }

    if(watchersEmpty) {
        isStreaming_ = false;
        std::unique_lock<std::mutex> lk(messengerMutex_);
        messenger_->cancelRequest(interruptRequest_);
        utils::sleepMs(100);
    }

    LOG_DEBUG("HidDevicePort::stopCapture done");
}

void HidDevicePort::addWatcher(std::shared_ptr<DataStreamWatcher> watcher) {
    LOG_DEBUG("HidDevicePort::startCapture start");
    {
        std::unique_lock<std::mutex> lk(watchersMutex_);
        for(auto iter = watchers_.begin(); iter != watchers_.end();) {
            auto item = iter->lock();
            if(!item || item == watcher) {
                iter = watchers_.erase(iter);
            }
            else {
                ++iter;
            }
        }
        watchers_.push_back(watcher);
    }

    if(isStreaming_ == false) {
        isStreaming_ = true;
        LOG_DEBUG("HidDevicePort::submit Request start");

        std::unique_lock<std::mutex> lk(messengerMutex_);
        auto                         sts = messenger_->submitRequest(interruptRequest_);
        if(sts != OB_USB_STATUS_SUCCESS)
            throw std::runtime_error("failed to submit interrupt request, error: " + usbStatusToString.at(sts));
    }
    LOG_DEBUG("HidDevicePort::startCapture done");
}

std::shared_ptr<const SourcePortInfo> HidDevicePort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace pal
}  // namespace libobsensor
