// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#include "InterfaceLibusb.hpp"

#include "MessengerLibusb.hpp"

namespace libobsensor {
namespace pal {
UsbInterfaceLibusb::UsbInterfaceLibusb(libusb_interface intf) : _desc(*intf.altsetting) {
    inf_ = intf;
    for(int e = 0; e < _desc.bNumEndpoints; ++e) {
        auto ep = _desc.endpoint[e];
        _endpoints.push_back(std::make_shared<UsbEndpointLibusb>(ep, _desc.bInterfaceNumber));
    }
}

UsbInterfaceLibusb::~UsbInterfaceLibusb() noexcept {}

const std::shared_ptr<UsbEndpoint> UsbInterfaceLibusb::firstEndpoint(EndpointDirection direction, EndpointType type) const {

    if(type == OB_USB_ENDPOINT_ISOCHRONOUS) {
        for(int alt_idx = 0; alt_idx < _associatedInterfaces.size(); alt_idx++) {
            auto altsetting = _associatedInterfaces[alt_idx];

            /* Find the endpoint with the number specified in the VS header */
            for(int ep_idx = 0; ep_idx < altsetting->getEndpoints().size(); ep_idx++) {
                auto endpoint = getEndpoints()[ep_idx];

                if(endpoint->getAddress() == 0x81) {

                    // int ret = libusb_set_interface_alt_setting( context.usb_device, altsetting->getNumber(), alt_idx );

                    return endpoint;
                }
            }
        }
    }
    else {
        for(auto &&ep: _endpoints) {
            if(ep->getType() != type)
                continue;
            if(ep->getDirection() != direction)
                continue;
            return ep;
        }
    }
    return nullptr;
}

const std::shared_ptr<UsbEndpoint> UsbInterfaceLibusb::getEndpoint(const uint8_t endPointAddress, EndpointDirection direction, EndpointType type) const {
    if(type == OB_USB_ENDPOINT_ISOCHRONOUS) {
        for(int alt_idx = 0; alt_idx < _associatedInterfaces.size(); alt_idx++) {
            auto altsetting = _associatedInterfaces[alt_idx];

            /* Find the endpoint with the number specified in the VS header */
            for(int ep_idx = 0; ep_idx < altsetting->getEndpoints().size(); ep_idx++) {
                auto endpoint = getEndpoints()[ep_idx];

                if(endpoint->getAddress() == endPointAddress) {
                    return endpoint;
                }
            }
        }
    }
    else {
        for(auto &&ep: _endpoints) {
            if(ep->getType() != type)
                continue;

            if(ep->getDirection() != direction)
                continue;

            if(ep->getAddress() == endPointAddress) {
                return ep;
            }
        }
    }

    return nullptr;
}

void UsbInterfaceLibusb::addAssociatedInterface(const std::shared_ptr<UsbInterface> &interface) {
    if(interface)
        _associatedInterfaces.push_back(interface);
}
}  // namespace pal
}  // namespace libobsensor
