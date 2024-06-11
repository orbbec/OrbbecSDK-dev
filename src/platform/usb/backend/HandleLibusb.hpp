// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "ContextLibusb.hpp"
#include "InterfaceLibusb.hpp"

#include "logger/Logger.hpp"
#include <chrono>
#include <errno.h>
#include <libusb.h>

namespace libobsensor {

static UsbStatus libusbStatusToOb(int sts) {
    switch(sts) {
    case LIBUSB_SUCCESS:
        return OB_USB_STATUS_SUCCESS;
    case LIBUSB_ERROR_IO:
        return OB_USB_STATUS_IO;
    case LIBUSB_ERROR_INVALID_PARAM:
        return OB_USB_STATUS_INVALID_PARAM;
    case LIBUSB_ERROR_ACCESS:
        return OB_USB_STATUS_ACCESS;
    case LIBUSB_ERROR_NO_DEVICE:
        return OB_USB_STATUS_NO_DEVICE;
    case LIBUSB_ERROR_NOT_FOUND:
        return OB_USB_STATUS_NOT_FOUND;
    case LIBUSB_ERROR_BUSY:
        return OB_USB_STATUS_BUSY;
    case LIBUSB_ERROR_TIMEOUT:
        return OB_USB_STATUS_TIMEOUT;
    case LIBUSB_ERROR_OVERFLOW:
        return OB_USB_STATUS_OVERFLOW;
    case LIBUSB_ERROR_PIPE:
        return OB_USB_STATUS_PIPE;
    case LIBUSB_ERROR_INTERRUPTED:
        return OB_USB_STATUS_INTERRUPTED;
    case LIBUSB_ERROR_NO_MEM:
        return OB_USB_STATUS_NO_MEM;
    case LIBUSB_ERROR_NOT_SUPPORTED:
        return OB_USB_STATUS_NOT_SUPPORTED;
    case LIBUSB_ERROR_OTHER:
        return OB_USB_STATUS_OTHER;
    default:
        return OB_USB_STATUS_OTHER;
    }
}

class HandleLibusb {
public:
    HandleLibusb(std::shared_ptr<UsbContext> context, libusb_device_handle *handle, std::shared_ptr<UsbInterfaceLibusb> interface)
        : context_(std::move(context)), firstInterface_(std::move(interface)), handle_(handle) {
        claimInterface(interface->getNumber());
        for(auto &&i: interface->getAssociatedInterfaces())
            claimInterface(i->getNumber());
        context_->startEventHandler();
    }

    ~HandleLibusb() noexcept {
        context_->stopEventHandler();
        for(auto &&i: firstInterface_->getAssociatedInterfaces())
            libusb_release_interface(handle_, i->getNumber());
    }

    libusb_device_handle *get() {
        return handle_;
    }

private:
    UsbStatus claimInterface(uint8_t interface) {
        if(libusb_kernel_driver_active(handle_, interface) == 1)      // find out if kernel driver is attached
            if(libusb_detach_kernel_driver(handle_, interface) == 0)  // detach driver from device if attached.
                LOG_ERROR("handle_libusb - detach kernel driver");

        auto sts = libusb_claim_interface(handle_, interface);
        if(sts != LIBUSB_SUCCESS) {
            auto rs_sts = libusbStatusToOb(sts);
            LOG_ERROR("failed to claim usb interface: {0}, error: {1}", (int)interface, usbStatusToString.at(rs_sts));
            return rs_sts;
        }

        return OB_USB_STATUS_SUCCESS;
    }

    std::shared_ptr<UsbContext>         context_;
    std::shared_ptr<UsbInterfaceLibusb> firstInterface_;
    libusb_device_handle               *handle_;
};

}  // namespace libobsensor
