// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#include "MacPal.hpp"

#include "logger/Logger.hpp"
#include "exception/ObException.hpp"

#if defined(BUILD_USB_PORT)
#include "usb/hid/HidDevicePort.hpp"
#include "usb/vendor/VendorUsbDevicePort.hpp"
#include "usb/uvc/ObLibuvcDevicePort.hpp"
#include "usb/uvc/rawPhaseConverter/MSDEConverterDevice.hpp"
#include "core/Context.hpp"
#include "core/device/DeviceInfoConfig.hpp"
#include <usb/openni/linux/OpenNIUSBLinux.hpp>
#include "usb/uvc/ObMultiUvcDevice.hpp"
#endif

#if defined(BUILD_NET_PORT)
#include "ethernet/Ethernet.hpp"
#endif

#include <cctype>  // std::tolower
#include <chrono>

namespace libobsensor {
namespace pal {


MacPal::MacPal() {
#if defined(BUILD_USB_PORT)
    usbEnumerator_ = std::make_shared<UsbEnumerator>();
#endif
}

MacPal::~MacPal() noexcept {}

std::shared_ptr<ISourcePort> MacPal::createSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    std::unique_lock<std::mutex> lock(sourcePortMapMutex_);
    std::shared_ptr<ISourcePort>  port;

    // clear expired weak_ptr
    for(auto it = sourcePortMap_.begin(); it != sourcePortMap_.end();) {
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
        auto usbDev = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        port = std::make_shared<VendorUsbDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        break;
    }
    case SOURCE_PORT_USB_UVC: {
        auto usbPortInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo);

        auto usbDev = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        port = std::make_shared<ObLibuvcDevicePort>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        break;
    }
    case SOURCE_PORT_USB_HID: {
        auto usbPortInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo);
        auto usbDev      = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
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
    if(port) {
        sourcePortMap_.insert(std::make_pair(portInfo, port));
    }
    return port;
}

#if defined(BUILD_USB_PORT)
std::shared_ptr<DeviceWatcher> MacPal::createUsbDeviceWatcher() const {
    LOG_INFO("Create PollingDeviceWatcher!");

    if(LibusbDeviceWatcher::hasCapability()) {
        return std::make_shared<LibusbDeviceWatcher>();
    }
}

SourcePortInfoList MacPal::queryUsbSourcePort() {
    SourcePortInfoList portInfoList;

    const auto &usbInfoList = usbEnumerator_->queryDevicesInfo();
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
        portInfo->hubId    = info.hubId;

        portInfoList.push_back(portInfo);
    }

    return portInfoList;
}

std::shared_ptr<ISourcePort> MacPal::createOpenNIDevicePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    if(portInfo->portType == SOURCE_PORT_USB_VENDOR) {
        auto usbDev = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
        }
        return std::make_shared<OpenNIDevicePortLinux>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
    }

    return nullptr;
}

std::shared_ptr<ISourcePort> MacPal::createRawPhaseConverterDevicePort(RawPhaseConverterPortType type, std::shared_ptr<const SourcePortInfo> portInfo) {
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

        auto usbDev = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
        if(usbDev == nullptr) {
            throw libobsensor::camera_disconnected_exception("usbEnumerator create UnpackDevicePort failed!");
        }
        port = std::make_shared<MSDEConverterDevice>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
        if(port) {
            sourcePortMap_.insert(std::make_pair(portInfo, port));
        }
        return port;
    }
    }
    return nullptr;
}

std::shared_ptr<ISourcePort> MacPal::createMultiUvcDevicePort(std::shared_ptr<const SourcePortInfo> portInfo) {
    auto usbDev = usbEnumerator_->createUsbDevice(std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo)->url);
    if(usbDev == nullptr) {
        throw libobsensor::camera_disconnected_exception("usbEnumerator createUsbDevice failed!");
    }
    return std::make_shared<ObMultiUvcDevice>(usbDev, std::dynamic_pointer_cast<const USBSourcePortInfo>(portInfo));
}

std::string parseDevicePath(libusb_device *usbDevice) {
    auto usb_bus = std::to_string(libusb_get_bus_number(usbDevice));
    // As per the USB 3.0 specs, the current maximum limit for the depth is 7.
    const auto               max_usb_depth            = 8;
    uint8_t                  usb_ports[max_usb_depth] = {};
    std::stringstream        port_path;
    auto                     port_count = libusb_get_port_numbers(usbDevice, usb_ports, max_usb_depth);
    auto                     usb_dev    = std::to_string(libusb_get_device_address(usbDevice));
    libusb_device_descriptor dev_desc;
    auto                     r = libusb_get_device_descriptor(usbDevice, &dev_desc);
    for(size_t i = 0; i < port_count; ++i) {
        port_path << std::to_string(usb_ports[i]) << (((i + 1) < port_count) ? "." : "");
    }
    return usb_bus + "-" + port_path.str() + "-" + usb_dev;
}

int deviceArrivalCallback(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
    auto watcher = (LibusbDeviceWatcher *)user_data;
    LOG_DEBUG("Device arrival event occurred");
    watcher->callback_(OB_DEVICE_ARRIVAL, parseDevicePath(device));
    return 0;
}

int deviceRemovedCallback(libusb_context *ctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
    auto watcher = (LibusbDeviceWatcher *)user_data;
    LOG_DEBUG("Device removed event occurred");
    watcher->callback_(OB_DEVICE_REMOVED, parseDevicePath(device));
    return 0;
}

#endif
}  // namespace pal
}  // namespace libobsensor
