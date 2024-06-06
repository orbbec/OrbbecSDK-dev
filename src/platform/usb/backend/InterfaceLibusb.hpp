// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "EndpointLibusb.hpp"
#include "Device.hpp"
#include "Interface.hpp"

#include <libusb.h>

namespace libobsensor {
namespace pal {
class UsbInterfaceLibusb : public UsbInterface {
public:
    UsbInterfaceLibusb(libusb_interface inf);

    virtual ~UsbInterfaceLibusb() noexcept override;

    virtual uint8_t getNumber() const override {
        return _desc.bInterfaceNumber;
    }
    virtual uint8_t getClass() const override {
        return _desc.bInterfaceClass;
    }
    virtual uint8_t getSubclass() const override {
        return _desc.bInterfaceSubClass;
    }
    virtual const std::vector<std::shared_ptr<UsbEndpoint>> getEndpoints() const override {
        return _endpoints;
    }
    virtual const std::shared_ptr<UsbEndpoint> firstEndpoint(const EndpointDirection direction, const EndpointType type = OB_USB_ENDPOINT_BULK) const override;
    virtual const std::shared_ptr<UsbEndpoint> getEndpoint(const uint8_t endPointAddress, const EndpointDirection direction,
                                                           const EndpointType type = OB_USB_ENDPOINT_BULK) const override;

    virtual const std::vector<std::shared_ptr<UsbInterface>> getAssociatedInterfaces() const {
        return _associatedInterfaces;
    }
    virtual uint8_t getAltsettingCount() const override {
        return (uint8_t)inf_.num_altsetting;
    }
    void addAssociatedInterface(const std::shared_ptr<UsbInterface> &interface);

private:
    libusb_interface                           inf_;
    libusb_interface_descriptor                _desc;
    std::vector<std::shared_ptr<UsbEndpoint>>  _endpoints;
    std::vector<std::shared_ptr<UsbInterface>> _associatedInterfaces;
};
}  // namespace pal
}  // namespace libobsensor
