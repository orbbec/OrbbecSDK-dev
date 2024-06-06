// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "ISourcePort.hpp"
#include "DeviceWatcher.hpp"

#if defined(BUILD_NET_PORT)
#include "ethernet/Ethernet.hpp"
#endif

#include <mutex>

namespace libobsensor {
namespace pal  // pal
{
class ObPal {
public:
    static std::shared_ptr<ObPal> getInstance();

    virtual ~ObPal() noexcept = default;

    virtual std::shared_ptr<ISourcePort> createSourcePort(std::shared_ptr<const SourcePortInfo>) = 0;

#if defined(BUILD_USB_PORT)
    virtual std::shared_ptr<DeviceWatcher> createUsbDeviceWatcher() const = 0;
    virtual SourcePortInfoList             queryUsbSourcePort()           = 0;
#endif

#if defined(BUILD_NET_PORT)
    virtual SourcePortInfoList queryNetSourcePort() = 0;
    virtual std::shared_ptr<DeviceWatcher> createNetDeviceWatcher() const  = 0;
#endif

};

}  // namespace pal
}  // namespace libobsensor
