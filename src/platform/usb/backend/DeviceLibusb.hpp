// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once

#include "MessengerLibusb.hpp"

#if defined(ANDROID) || defined(__ANDROID__)
#include "pal/android/AndroidUsbDeviceManager.hpp"
#endif

namespace libobsensor {


#ifndef __ANDROID__
std::string getDevicePath(libusb_device *usbDevice);
int         safe_open_device(libusb_device *device, libusb_device_handle **dev_handle);
int         safe_close_device(libusb_device *device, libusb_device_handle *dev_handle);
#endif

class UsbDeviceLibusb : public UsbDevice, public std::enable_shared_from_this<UsbDeviceLibusb> {
public:
#ifdef __ANDROID__
    UsbDeviceLibusb(std::shared_ptr<AndroidUsbDeviceManager> usbManager, libusb_device *device, libusb_device_handle *deviceHandle,
                    libusb_device_descriptor &desc, std::shared_ptr<UsbContext> context, std::string devUrl);
#else
    UsbDeviceLibusb(libusb_device *device, const libusb_device_descriptor &desc, std::shared_ptr<UsbContext> context);
#endif
    virtual ~UsbDeviceLibusb() noexcept;

    virtual const std::vector<std::shared_ptr<UsbInterface>> getInterfaces() const override {
        return interfaces_;
    }
    virtual const std::shared_ptr<UsbInterface> getInterface(uint8_t interfaceNumber) const override;
    virtual const std::shared_ptr<UsbMessenger> open(uint8_t interfaceNumber) override;
    virtual const std::vector<UsbDescriptor>    getDescriptors() const override {
        return descriptors_;
    }
    libusb_device *getDevice() {
        return device_;
    }
    libusb_device_handle *getDeviceHandle() {
        return handle_;
    }
    libusb_context *getContext() {
        return context_->get();
    }

private:
    void init();

private:
    libusb_device                             *device_;
    libusb_device_descriptor                   usbDeviceDescriptor_;
    std::vector<std::shared_ptr<UsbInterface>> interfaces_;
    std::vector<UsbDescriptor>                 descriptors_;
    std::shared_ptr<UsbContext>                context_;
    libusb_device_handle                      *handle_;
    std::shared_ptr<HandleLibusb>              get_handle(uint8_t interfaceNumber);

#ifdef __ANDROID__
private:
    std::shared_ptr<AndroidUsbDeviceManager> androidUsbManager_;
    const std::string                        devUrl_;
#endif
};

}  // namespace libobsensor
