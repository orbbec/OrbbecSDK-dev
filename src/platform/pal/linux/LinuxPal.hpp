// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "ObPal.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"

#include <iostream>
#include <vector>
#include <map>

#if defined(BUILD_USB_PORT)
#include "usb/enumerator/IUsbEnumerator.hpp"
#endif
namespace libobsensor {

class LinuxPal : public ObPal {
private:
    LinuxPal();

    static std::weak_ptr<LinuxPal> instanceWeakPtr_;
    static std::mutex              instanceMutex_;
    friend std::shared_ptr<ObPal>  ObPal::getInstance();

public:
    ~LinuxPal() noexcept override;

    std::shared_ptr<ISourcePort> createSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) override;

#if defined(BUILD_USB_PORT)
public:
    std::shared_ptr<DeviceWatcher> createUsbDeviceWatcher() const override;
    SourcePortInfoList             queryUsbSourcePort() override;

private:
    void loadXmlConfig();

    std::shared_ptr<IUsbEnumerator> usbEnumerator_;

    typedef enum {
        UVC_BACKEND_TYPE_AUTO,  // if support v4l2 metadata, use v4l2, else use libusb. default is auto
        UVC_BACKEND_TYPE_LIBUVC,
        UVC_BACKEND_TYPE_V4L2,
    } UvcBackendType;

    UvcBackendType uvcBackendType_ = UVC_BACKEND_TYPE_LIBUVC;
#endif

private:
    std::mutex                                                                  sourcePortMapMutex_;
    std::map<std::shared_ptr<const SourcePortInfo>, std::weak_ptr<ISourcePort>> sourcePortMap_;
};

}  // namespace libobsensor
