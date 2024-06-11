// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.
#pragma once

#include "Endpoint.hpp"
#include "Request.hpp"

namespace libobsensor {
namespace pal {
class UsbMessenger {
public:
    virtual ~UsbMessenger() noexcept = default;

    virtual UsbStatus    controlTransfer(int request_type, int request, int value, int index, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                         uint32_t timeout_ms)                                             = 0;
    virtual UsbStatus    bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                      uint32_t timeout_ms)                                                = 0;
    virtual UsbStatus    resetEndpoint(const std::shared_ptr<UsbEndpoint> &endpoint, uint32_t timeout_ms) = 0;
    virtual UsbStatus    submitRequest(const obUsbRequest &request)                                       = 0;
    virtual UsbStatus    cancelRequest(const obUsbRequest &request)                                       = 0;
    virtual obUsbRequest createRequest(std::shared_ptr<UsbEndpoint> endpoint)                             = 0;
};

}  // namespace pal
}  // namespace libobsensor
