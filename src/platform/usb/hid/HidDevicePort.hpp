// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "ISourcePort.hpp"
#include "usb/backend/Enumerator.hpp"
#include "usb/backend/Messenger.hpp"
#include "usb/backend/Device.hpp"
#include "frame/FrameQueue.hpp"

namespace libobsensor {

class HidDevicePort : public IDataStreamPort {
public:
    HidDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo);

    ~HidDevicePort() noexcept override;

    void startStream(FrameCallbackUnsafe callback) override;
    void stopStream() override;

     std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const override;

private:
    std::shared_ptr<const USBSourcePortInfo>      portInfo_;
    std::shared_ptr<UsbDevice>                    usbDevice_ = nullptr;
    std::mutex                                    messengerMutex_;
    std::shared_ptr<UsbMessenger>                 messenger_;
    obUsbRequest                                  interruptRequest_;
    obUsbRequestCallback                          interruptCallback_;
    std::shared_ptr<UsbEndpoint>                  readEndpoint_;

    std::atomic_bool                              isStreaming_;
    FrameQueue<Frame>                            frameQueue_;
};

}  // namespace libobsensor
