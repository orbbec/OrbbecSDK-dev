// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "Interface.hpp"
#include "UsbTypes.hpp"

#include <memory>
#include <stdint.h>
#include <vector>

namespace libobsensor {
namespace pal {
class UsbDevice {
public:
    virtual ~UsbDevice() noexcept = default;

    virtual const std::vector<std::shared_ptr<UsbInterface>> getInterfaces() const                        = 0;
    virtual const std::shared_ptr<UsbInterface>              getInterface(uint8_t interface_number) const = 0;
    virtual const std::shared_ptr<UsbMessenger>              open(uint8_t interface_number)               = 0;
    virtual const std::vector<UsbDescriptor>                 getDescriptors() const                       = 0;
};

}  // namespace pal
}  // namespace libobsensor
