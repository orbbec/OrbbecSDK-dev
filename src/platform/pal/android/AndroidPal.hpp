// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "ObPal.hpp"

#include <memory>

#if defined(BUILD_USB_PORT)
#include <pal/android/AndroidUsbDeviceManager.hpp>
#endif

namespace libobsensor {
namespace pal {
class AndroidPal : public ObPal {
public:
    AndroidPal();
    virtual ~AndroidPal() noexcept;

    virtual std::shared_ptr<ISourcePort> createSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) override;

#if defined(BUILD_USB_PORT)
    virtual std::shared_ptr<DeviceWatcher> createUsbDeviceWatcher() const override;
    virtual SourcePortInfoList             queryUsbSourcePort() override;
    virtual std::shared_ptr<ISourcePort>    createOpenNIDevicePort(std::shared_ptr<const SourcePortInfo>) override;
    virtual std::shared_ptr<ISourcePort>    createMultiUvcDevicePort(std::shared_ptr<const SourcePortInfo> portInfo) override;
    virtual std::shared_ptr<ISourcePort>    createRawPhaseConverterDevicePort(RawPhaseConverterPortType type, std::shared_ptr<const SourcePortInfo>) override;
#endif

    std::shared_ptr<AndroidUsbDeviceManager> getAndroidUsbManager() {
        return androidUsbManager_;
    }

private:
    std::shared_ptr<AndroidUsbDeviceManager>                                   androidUsbManager_;
    std::mutex                                                                 sourcePortMapMutex_;
    std::map<std::shared_ptr<const SourcePortInfo>, std::weak_ptr<ISourcePort>> sourcePortMap_;
};
}  // namespace pal
}  // namespace libobsensor
