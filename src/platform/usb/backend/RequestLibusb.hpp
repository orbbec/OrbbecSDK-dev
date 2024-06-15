// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "libusb.h"
#include "Device.hpp"
#include "Request.hpp"

namespace libobsensor {

class UsbRequestLibusb : public UsbRequestBase {
public:
    UsbRequestLibusb(libusb_device_handle *devHandle, std::shared_ptr<UsbEndpoint> endpoint);
    virtual ~UsbRequestLibusb() noexcept;

    virtual int              getActualLength() const override;
    virtual void            *getNativeRequest() const override;
    virtual usbRequestStatus getRequestStatus() const override;

    std::shared_ptr<UsbRequest> getShared() const;
    void                        setShared(const std::shared_ptr<UsbRequest> &shared);
    void                        setActive(bool state);

protected:
    virtual void     setNativeBufferLength(int length) override;
    virtual int      getNativeBufferLength() override;
    virtual void     setNativeBuffer(uint8_t *buffer) override;
    virtual uint8_t *getNativeBuffer() const override;

private:
    bool                             active_ = false;
    std::weak_ptr<UsbRequest>        shared_;
    std::shared_ptr<libusb_transfer> transfer_;
    bool                             isIso_                  = false;
    uint32_t                         endpointBytesPerPacket_ = 0;
    libusb_device_handle            *devHandle_;
};

}  // namespace libobsensor
