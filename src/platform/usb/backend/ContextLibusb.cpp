// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#include "ContextLibusb.hpp"

#include "logger/LoggerInterval.hpp"

namespace libobsensor {
namespace pal {
UsbContext::UsbContext() : ctx_(NULL), count_(0), list_(nullptr) {

#ifdef __ANDROID__
    auto rc = libusb_set_option(ctx_, LIBUSB_OPTION_WEAK_AUTHORITY, NULL);
    if(rc != LIBUSB_SUCCESS) {
        LOG_ERROR("libusb set option LIBUSB_OPTION_WEAK_AUTHORITY failed!");
        throw std::runtime_error("libusb set option LIBUSB_OPTION_WEAK_AUTHORITY failed");
    }
#endif

    auto sts = libusb_init(&ctx_);
    if(sts != LIBUSB_SUCCESS) {
        LOG_ERROR("libusb_init failed");
    }

#ifndef __ANDROID__
    count_ = libusb_get_device_list(ctx_, &list_);
#endif
    startEventHandler();
}

UsbContext::~UsbContext() noexcept {
    libusb_free_device_list(list_, true);
    handlerRequests_ = 0;
    stopEventHandler();
    libusb_exit(ctx_);
}

libusb_context *UsbContext::get() {
    return ctx_;
}

void UsbContext::startEventHandler() {
    LOG_DEBUG("UsbContext::startEventHandler()");
    std::lock_guard<std::mutex> lk(mutex_);
    if(handlerRequests_ == 0) {
        killHandlerThread_ = 0;
        eventHandler_      = std::thread([&]() {
            while(!killHandlerThread_) {
                auto rc = libusb_handle_events_completed(ctx_, &killHandlerThread_);
                if(rc != LIBUSB_SUCCESS) {
                    LOG_WARN_INTVL("libusb_handle_events_completed failed: {}", libusb_strerror(rc));
                }
            }
        });
    }
    handlerRequests_++;
}

void UsbContext::stopEventHandler() {
    LOG_DEBUG("UsbContext::stopEventHandler()");
    std::lock_guard<std::mutex> lk(mutex_);
    if(handlerRequests_ <= 1) {
        killHandlerThread_ = 1;
        libusb_interrupt_event_handler(ctx_);
        if(eventHandler_.joinable()) {
            eventHandler_.join();
        }
    }
    handlerRequests_--;
}

libusb_device *UsbContext::getDevice(uint8_t index) {
    return index < count_ ? list_[index] : NULL;
}

size_t UsbContext::deviceCount() {
    return count_;
}

void UsbContext::refreshDeviceList() {
#ifndef __ANDROID__
    libusb_free_device_list(list_, true);
    count_ = libusb_get_device_list(ctx_, &list_);
#endif
}
}  // namespace pal
}  // namespace libobsensor
