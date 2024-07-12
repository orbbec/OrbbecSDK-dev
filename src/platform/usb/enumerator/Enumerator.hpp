// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#ifndef __ANDROID__

#include "UsbTypes.hpp"

#include <memory>
#include <vector>
#include <thread>
#include <mutex>

#include <libusb.h>

namespace libobsensor {

class UsbEnumerator {

public:
    UsbEnumerator();
    ~UsbEnumerator() noexcept;

    const std::vector<UsbInterfaceInfo> &queryUsbInterfaces();

    std::shared_ptr<UsbDevice> openUsbDevice(const std::string &devUrl, uint8_t retry = 1);

    static libusb_endpoint_descriptor getEndpointAddress(std::shared_ptr<UsbDevice> dev, int interfaceIndex, libusb_endpoint_transfer_type transferType,
                                                         libusb_endpoint_direction direction);

private:
    void                                  startEventHandleThread();
    void                                  stopEventHandleThread();
    std::shared_ptr<libusb_device_handle> openLibusbDevice(const std::string &devUrl);

private:
    std::vector<UsbInterfaceInfo> devInterfaceList_;

    libusb_context *libusbCtx_;
    std::thread     libusbEventHandlerThread_;
    int             libusbEventHandlerExit_ = 0;

    std::mutex                                                 libusbDeviceHandleMutex_;
    std::map<std::string, std::weak_ptr<libusb_device_handle>> libusbDeviceHandles_;
};

}  // namespace libobsensor

#endif  // __ANDROID__