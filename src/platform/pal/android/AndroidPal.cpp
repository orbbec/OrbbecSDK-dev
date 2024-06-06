// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "AndroidPal.hpp"

#if defined(BUILD_USB_PORT)
#include <pal/android/AndroidUsbDeviceManager.hpp>
#include "usb/vendor/VendorUsbDevicePort.hpp"
#include "usb/uvc/ObLibuvcDevicePort.hpp"
#include <usb/hid/HidDevicePort.hpp>
#include "usb/uvc/rawPhaseConverter/MSDEConverterDevice.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"
#endif

#if defined(BUILD_NET_PORT)
#include "ethernet/Ethernet.hpp"
#endif

namespace libobsensor {
namespace pal {

AndroidPal::AndroidPal() {
#if defined(BUILD_USB_PORT)
    androidUsbManager_ = std::make_shared<AndroidUsbDeviceManager>();
#endif
}
AndroidPal::~AndroidPal() noexcept {
#if defined(BUILD_USB_PORT)
    androidUsbManager_ = nullptr;
#endif
};

std::shared_ptr<ISourcePort> AndroidPal::createSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    std::unique_lock<std::mutex> lock(sourcePortMapMutex_);
    std::shared_ptr<ISourcePort>  port;

    // clear expired weak_ptr
    for(auto it = sourcePortMap_.begin(); it!= sourcePortMap_.end();) {
        if(it->second.expired()) {
            it = sourcePortMap_.erase(it);
        }
        else {
            ++it;
        }
    }

    // check if the port already exists in the map
    for(const auto &pair: sourcePortMap_) {
        if(pair.first->equal(portInfo)) {
            port = pair.second.lock();
            if(port != nullptr) {
                return port;
            }
        }
    }

    switch(portInfo->portType) {
#if defined(BUILD_USB_PORT)
    case SOURCE_PORT_USB_VENDOR: {
        auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        port = std::make_shared<VendorUsbDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        break;
    }
    case SOURCE_PORT_USB_UVC: {
        auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        port = std::make_shared<ObLibuvcDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        break;
    }
    case SOURCE_PORT_USB_HID: {
        auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        port = std::make_shared<HidDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        break;
    }
#endif

#if defined(BUILD_NET_PORT)
    case SOURCE_PORT_NET_VENDOR:
        port = std::make_shared<VendorNetDataPort>(std::dynamic_pointer_cast<const NetSourcePortInfo>(portInfo));
        break;
    case SOURCE_PORT_NET_RTSP:
        port = std::make_shared<RTSPStreamPort>(std::dynamic_pointer_cast<const RTSPStreamPortInfo>(portInfo));
        break;
    case SOURCE_PORT_NET_VENDOR_STREAM:
        port = std::make_shared<NetDataStreamPort>(std::dynamic_pointer_cast<const NetDataStreamPortInfo>(portInfo));
        break;
#endif
    default:
        throw libobsensor::invalid_value_exception("unsupported source port type!");
        break;
    }
    if(port != nullptr) {
        sourcePortMap_.insert(std::make_pair(portInfo, port));
    }
    return port;
}

#if defined(BUILD_USB_PORT)
std::shared_ptr<DeviceWatcher> AndroidPal::createUsbDeviceWatcher() const {
    LOG_INFO("Create AndroidUsbDeviceManager!");
    return androidUsbManager_;
}

SourcePortInfoList AndroidPal::queryUsbSourcePort() {
    SourcePortInfoList portInfoList;

    const auto &usbInfoList = androidUsbManager_->getDeviceInfoList();
    for(const auto &info: usbInfoList) {
        auto portInfo      = std::make_shared<USBSourcePortInfo>(cvtUsbClassToPortType(info.cls));
        portInfo->url      = info.url;
        portInfo->uid      = info.uid;
        portInfo->vid      = info.vid;
        portInfo->pid      = info.pid;
        portInfo->serial   = info.serial;
        portInfo->connSpec = usb_spec_names.find(info.conn_spec)->second;
        portInfo->infUrl   = info.infUrl;
        portInfo->infIndex = info.infIndex;
        portInfo->infName  = info.infName;
        // portInfo->hubId    = info.hubId;

        portInfoList.push_back(portInfo);
    }
    return portInfoList;
}

std::shared_ptr<ISourcePort> AndroidPal::createOpenNIDevicePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    if(portInfo->portType == SOURCE_PORT_USB_VENDOR) {
        auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        return std::make_shared<OpenNIDevicePortLinux>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
    }

    return nullptr;
}

std::shared_ptr<ISourcePort> AndroidPal::createRawPhaseConverterDevicePort(RawPhaseConverterPortType type, std::shared_ptr<const SourcePortInfo> portInfo) {
    switch(type) {
    case RAW_PHASE_CONVERTER_MSDE: {
        std::unique_lock<std::mutex> lock(sourcePortMapMutex_);
        std::shared_ptr<ISourcePort>  port;
        auto                         it = sourcePortMap_.find(portInfo);
        if(it != sourcePortMap_.end()) {
            port = it->second.lock();
            if(port != nullptr) {
                return port;
            }
        }
        auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator create UnpackDevicePort failed!");
        }
        port = std::make_shared<MSDEConverterDevice>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        if(port != nullptr) {
            sourcePortMap_.insert(std::make_pair(portInfo, port));
        }
        return port;
    }
    }
    return nullptr;
}

std::shared_ptr<ISourcePort> AndroidPal::createMultiUvcDevicePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    auto usbDev = androidUsbManager_->openUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
    if(usbDev == nullptr) {
        throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
    }
    return std::make_shared<ObMultiUvcDevice>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
}

#endif

}  // namespace pal
}  // namespace libobsensor
