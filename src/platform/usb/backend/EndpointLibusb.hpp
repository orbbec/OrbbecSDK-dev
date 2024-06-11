

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "usb/backend/Endpoint.hpp"

#include <libusb.h>

namespace libobsensor {
namespace pal {
class UsbEndpointLibusb : public UsbEndpoint {
public:
    UsbEndpointLibusb(libusb_endpoint_descriptor desc, uint8_t interfaceNumber) : desc_(desc), interfaceNumber_(interfaceNumber) {}

    ~UsbEndpointLibusb()  noexcept override = default;

    uint8_t getAddress() const override {
        return desc_.bEndpointAddress;
    }
    EndpointType getType() const override {
        return (EndpointType)desc_.bmAttributes;
    }
    uint8_t getInterfaceNumber() const override {
        return interfaceNumber_;
    }

    EndpointDirection getDirection() const override {
        return desc_.bEndpointAddress >= OB_USB_ENDPOINT_DIRECTION_READ ? OB_USB_ENDPOINT_DIRECTION_READ : OB_USB_ENDPOINT_DIRECTION_WRITE;
    }
    uint32_t getMaxPacketSize() const override {
        return desc_.wMaxPacketSize;
    }
    libusb_endpoint_descriptor getDescriptor() {
        return desc_;
    }

private:
    libusb_endpoint_descriptor desc_;
    uint8_t                    interfaceNumber_;
};
}  // namespace pal
}  // namespace libobsensor
