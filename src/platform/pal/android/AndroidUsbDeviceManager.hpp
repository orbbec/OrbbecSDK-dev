// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "ObPal.hpp"
#include "usb/backend/Device.hpp"
#include "usb/backend/ContextLibusb.hpp"

#include <string>
#include <map>
#include <memory>
#include <jni.h>

namespace libobsensor {

struct AndroidUsbDeviceHandle {
    std::string              url = "";
    intptr_t                 fd  = NULL;
    libusb_device_descriptor desc;
    libusb_device           *device       = nullptr;
    libusb_device_handle    *deviceHandle = nullptr;

    uint16_t ref = 0;
};

class AndroidUsbDeviceManager : public DeviceWatcher, public std::enable_shared_from_this<AndroidUsbDeviceManager> {
public:
    AndroidUsbDeviceManager();
    ~AndroidUsbDeviceManager();

    virtual void               start(deviceChangedCallback callback) override;
    virtual void               stop() override;
    void                       onDeviceChanged(OBDeviceChangedType changeType_, const UsbDeviceInfo &usbDevInfo);
    std::vector<UsbDeviceInfo> getDeviceInfoList();

    std::shared_ptr<UsbDevice> openUsbDevice(const std::string &devUrl);
    void                       closeUsbDevice(const std::string &devUrl);

    void addUsbDevice(JNIEnv *env, jobject usbDevInfo);

    void removeUsbDevice(JNIEnv *env, jobject usbDevInfo);

    void registerDeviceWatcher(JNIEnv *env, jclass typeDeviceWatcher, jobject jDeviceWatcher);

private:
    std::recursive_mutex                          mutex_;
    deviceChangedCallback                         callback_ = nullptr;
    std::vector<UsbDeviceInfo>                    deviceInfoList_;
    std::shared_ptr<UsbContext>                   usbCtx_;
    std::map<std::string, AndroidUsbDeviceHandle> deviceHandleMap_;

    std::mutex       jvmMutex_;
    JavaVM          *gJVM_;
    volatile jobject jObjDeviceWatcher_;
};

}  // namespace libobsensor
