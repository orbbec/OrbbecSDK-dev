// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.
#include "MessengerLibusb.hpp"
#include "DeviceLibusb.hpp"
#include "HandleLibusb.hpp"

#include "logger/Logger.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

UsbMessengerLibusb::UsbMessengerLibusb(const std::shared_ptr<UsbDeviceLibusb> &device, std::shared_ptr<HandleLibusb> handle)
    : device_(device), handle_(handle) {}

UsbMessengerLibusb::~UsbMessengerLibusb() noexcept {}

UsbStatus UsbMessengerLibusb::resetEndpoint(const std::shared_ptr<UsbEndpoint> &endpoint, uint32_t timeout_ms) {
    utils::unusedVar(timeout_ms);
    auto  ep  = endpoint->getAddress();
    auto sts = libusb_clear_halt(handle_->get(), ep);
    if(sts < 0) {
        std::string strerr = strerror(errno);
        LOG_WARN("reset_endpoint returned error, index:{0}, error:{1}, number:{2}", ep, strerr, int(errno));
        return libusbStatusToOb(sts);
    }
    return OB_USB_STATUS_SUCCESS;
}

UsbStatus UsbMessengerLibusb::controlTransfer(int request_type, int request, int value, int index, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                              uint32_t timeout_ms) {
    auto sts = libusb_control_transfer(handle_->get(), (uint8_t)request_type, (uint8_t)request, (uint16_t)value, (uint16_t)index, buffer, (uint16_t)length, timeout_ms);
    if(sts < 0) {
        std::string strerr = strerror(errno);
        LOG_WARN("control_transfer returned error, index:{0}, error:{1}, number:{2}", index, strerr, int(errno));
        return libusbStatusToOb(sts);
    }
    transferred = uint32_t(sts);
    return OB_USB_STATUS_SUCCESS;
}

UsbStatus UsbMessengerLibusb::bulkTransfer(const std::shared_ptr<UsbEndpoint> &endpoint, uint8_t *buffer, uint32_t length, uint32_t &transferred,
                                           uint32_t timeout_ms) {
    int  actual_length = 0;
    auto sts           = libusb_bulk_transfer(handle_->get(), endpoint->getAddress(), buffer, length, &actual_length, timeout_ms);
    if(sts < 0) {
        std::string strerr = strerror(errno);
        LOG_WARN("bulkTransfer returned error, endpoint: 0x{0:x}, error:{1}, number:{2}", int(endpoint->getAddress()), strerr, int(errno));
        return libusbStatusToOb(sts);
    }
    transferred = actual_length;
    return OB_USB_STATUS_SUCCESS;
}

obUsbRequest UsbMessengerLibusb::createRequest(std::shared_ptr<UsbEndpoint> endpoint) {
    auto rv = std::make_shared<UsbRequestLibusb>(handle_->get(), endpoint);
    rv->setShared(rv);
    return rv;
}

UsbStatus systemStatusToOb(int sts) {
    switch(sts) {
    case ENODEV:
        return OB_USB_STATUS_NO_DEVICE;
    default:
        return OB_USB_STATUS_OTHER;
    }
}

UsbStatus UsbMessengerLibusb::submitRequest(const obUsbRequest &request) {
    auto nr = reinterpret_cast<libusb_transfer *>(request->getNativeRequest());
    if(nr->dev_handle == nullptr)
        return OB_USB_STATUS_INVALID_PARAM;
    auto req = std::dynamic_pointer_cast<UsbRequestLibusb>(request);

    req->setActive(true);
    auto sts = libusb_submit_transfer(nr);
    if(sts < 0) {
        req->setActive(false);
        std::string strerr = strerror(errno);
        LOG_WARN("usb_request_queue returned error, endpoint:{0}, error:{1}, number:{2} ", (int)request->getEndpoint()->getAddress(), strerr, (int)errno);

        if (libusbStatusToOb(errno) == OB_USB_STATUS_OTHER) {
            return systemStatusToOb(errno);
        }
    }
    return OB_USB_STATUS_SUCCESS;
}

UsbStatus UsbMessengerLibusb::cancelRequest(const obUsbRequest &request) {
    auto nr  = reinterpret_cast<libusb_transfer *>(request->getNativeRequest());
    auto sts = libusb_cancel_transfer(nr);
    if(sts < 0) {
        std::string strerr = strerror(errno);
        LOG_WARN("usb_request_cancel returned error, endpoint:{0}, error:{1}, number:{2} ", (int)request->getEndpoint()->getAddress(), strerr, (int)errno);

        if (libusbStatusToOb(errno) == OB_USB_STATUS_OTHER) {
            return systemStatusToOb(errno);
        }
    }
    return OB_USB_STATUS_SUCCESS;
}

}  // namespace libobsensor
