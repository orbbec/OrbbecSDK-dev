// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "ISourcePort.hpp"
#include "usb/backend/Enumerator.hpp"
#include "usb/backend/Messenger.hpp"
#include "usb/backend/Device.hpp"

#include <cstdio>
#include <cstdlib>
#include <condition_variable>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <set>

namespace libobsensor {
namespace pal {

class HidDevicePort : public IDataStreamPort {
public:
    HidDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo);

    ~HidDevicePort() noexcept override;

    void addWatcher(std::shared_ptr<DataStreamWatcher> watcher) override;
    void removeWatcher(std::shared_ptr<DataStreamWatcher> watcher) override;

    virtual std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const override;

private:
    std::shared_ptr<const USBSourcePortInfo>      portInfo_;
    std::shared_ptr<UsbDevice>                    usbDevice_ = nullptr;
    std::mutex                                    messengerMutex_;
    std::shared_ptr<UsbMessenger>                 messenger_;
    obUsbRequest                                  interruptRequest_;
    obUsbRequestCallback                          interruptCallback_;
    std::shared_ptr<UsbEndpoint>                  readEndpoint_;
    std::atomic_bool                              isStreaming_;
    std::vector<std::weak_ptr<DataStreamWatcher>> watchers_;
    std::mutex                                    watchersMutex_;
};

}  // namespace pal
}  // namespace libobsensor
