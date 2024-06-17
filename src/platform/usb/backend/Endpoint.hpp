// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "UsbTypes.hpp"

#include <memory>
#include <stdint.h>

namespace libobsensor {

class UsbEndpoint {
public:
    virtual ~UsbEndpoint() noexcept                      = default;
    virtual uint8_t           getAddress() const         = 0;
    virtual EndpointType      getType() const            = 0;
    virtual EndpointDirection getDirection() const       = 0;
    virtual uint8_t           getInterfaceNumber() const = 0;
    virtual uint32_t          getMaxPacketSize() const   = 0;
};

}  // namespace libobsensor