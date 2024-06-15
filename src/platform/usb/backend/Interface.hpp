// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "Endpoint.hpp"
#include "Messenger.hpp"
#include "UsbTypes.hpp"

#include <memory>
#include <vector>

namespace libobsensor {

class UsbInterface {
public:
    UsbInterface()                   = default;
    virtual ~UsbInterface() noexcept = default;

    virtual uint8_t                                         getNumber() const          = 0;
    virtual uint8_t                                         getClass() const           = 0;
    virtual uint8_t                                         getSubclass() const        = 0;
    virtual const std::vector<std::shared_ptr<UsbEndpoint>> getEndpoints() const       = 0;
    virtual uint8_t                                         getAltsettingCount() const = 0;

    virtual const std::shared_ptr<UsbEndpoint> firstEndpoint(const EndpointDirection direction, const EndpointType type = OB_USB_ENDPOINT_BULK) const = 0;
    virtual const std::shared_ptr<UsbEndpoint> getEndpoint(const uint8_t endPointAddress, const EndpointDirection direction,
                                                           const EndpointType type = OB_USB_ENDPOINT_BULK) const                                      = 0;
};


}  // namespace libobsensor