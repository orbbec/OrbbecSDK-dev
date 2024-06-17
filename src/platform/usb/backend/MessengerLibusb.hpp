// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "HandleLibusb.hpp"
#include "InterfaceLibusb.hpp"
#include "RequestLibusb.hpp"
#include "Messenger.hpp"

namespace libobsensor {

class UsbDeviceLibusb;

class UsbMessengerLibusb : public UsbMessenger {
public:
             UsbMessengerLibusb(const std::shared_ptr<UsbDeviceLibusb> &device, std::shared_ptr<HandleLibusb> handle);
    virtual ~UsbMessengerLibusb() noexcept override;

    virtual UsbStatus    controlTransfer(int request_type, int request, int value, int index, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                         uint32_t timeoutMs) override;
    virtual UsbStatus    bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                      uint32_t timeoutMs) override;
    virtual UsbStatus    resetEndpoint(const std::shared_ptr<UsbEndpoint> &endpoint, uint32_t timeout_ms) override;
    virtual UsbStatus    submitRequest(const obUsbRequest &request) override;
    virtual UsbStatus    cancelRequest(const obUsbRequest &request) override;
    virtual obUsbRequest createRequest(std::shared_ptr<UsbEndpoint> endpoint) override;

private:
    const std::shared_ptr<UsbDeviceLibusb> device_;
    std::mutex                             mutex_;
    std::shared_ptr<HandleLibusb>          handle_;
};

}  // namespace libobsensor
