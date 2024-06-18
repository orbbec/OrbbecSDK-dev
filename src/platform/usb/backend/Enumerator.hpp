// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#ifndef __ANDROID__

#include "Device.hpp"
#include <memory>
#include <vector>

namespace libobsensor {

class UsbContext;  // forward declaration
class UsbEnumerator {
public:
    UsbEnumerator();
    ~UsbEnumerator() noexcept;
    std::shared_ptr<UsbDevice>        createUsbDevice(const std::string &devUrl, const uint8_t retry = 1);  // retry, 失败重试
    const std::vector<UsbDeviceInfo> &queryDevicesInfo();

private:
    bool getStringDesc(UsbDeviceInfo &info);

private:
    std::vector<UsbDeviceInfo> devInfoList_;

    std::shared_ptr<UsbContext> usbCtx_;
    std::recursive_mutex        usbCtxMutex_;
};

}  // namespace libobsensor

#endif  // __ANDROID__