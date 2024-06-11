// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.
#ifndef __ANDROID__

#include "logger/Logger.hpp"
#include "DeviceLibusb.hpp"
#include "Enumerator.hpp"
#include "UsbTypes.hpp"

#include "utils/Utils.hpp"
#include "exception/ObException.hpp"

#include <algorithm>
#include <libusb.h>

namespace libobsensor {

std::string getDeviceHubID(const std::string &devicePath) {
    auto sstart = devicePath.find_first_of('-');
    auto send   = devicePath.find_last_of('-');
    if(sstart == std::string::npos || send == std::string::npos || sstart + 1 > send - 1) {
        return devicePath.substr(0, sstart);
    }
    std::string object = devicePath.substr(sstart + 1, send - sstart - 1);
    auto        ssend  = object.find_last_of('.');
    if(ssend == std::string::npos) {
        return devicePath.substr(0, sstart);
    }
    else {
        return devicePath.substr(0, sstart) + "-" + object.substr(0, ssend);
    }
}

#ifdef WIN32
/*
 *  	0x01516fe8 "\\\\?\\USB#VID_2BC5&PID_0633#6&3436DA6F&1&2#{A5DCBF10-6530-11D2-901F-00C04FB951ED}"
 *    0x0152e920 "\\\\?\\USB#VID_2BC5&PID_0532#USB#{A5DCBF10-6530-11D2-901F-00C04FB951ED}"
 */
// Parse the following USB path format = \?usb#vid_vvvv&pid_pppp&mi_ii#aaaaaaaaaaaaaaaa#{gggggggg-gggg-gggg-gggg-gggggggggggg}
// vvvv = USB vendor ID represented in 4 hexadecimal characters.
// pppp = USB product ID represented in 4 hexadecimal characters.
// ii = USB interface number.
// aaaaaaaaaaaaaaaa = unique Windows-generated string based on things such as the physical USB port address and/or interface number.
// gggggggg-gggg-gggg-gggg-gggggggggggg = device interface GUID assigned in the driver or driver INF file and is used to link applications to device with
// specific drivers loaded.
std::string getDeviceUidByWin(const std::string &devicePath) {
    auto name = devicePath;
    utils::toLower(name);
    auto tokens = utils::tokenize(name, '#');
    if(tokens.empty() || (tokens[0] != R"(\\?\usb)" && tokens[0] != R"(\\?\hid)"))
        return "";  // Not a USB device
    if(tokens.size() < 3) {
        LOG_ERROR("malformed usb device path: {}", name);
        return "";
    }

    auto ids = utils::tokenize(tokens[2], '&');
    if(ids.empty()) {
        LOG_ERROR("malformed id string: {}", tokens[2]);
        return "";
    }

    if(ids.size() == 1) {
        std::string TempId = tokens[2];
        if(TempId.length() >= 11) {
            return tokens[2];
        }
    }

    if(ids.size() > 2) {
        // uid = ids[1];
        return utils::remove(tokens[2], "&");
    }

    return "";
}

std::string getDeviceUidByWinDeviceID(const std::string &deviceID) {
    auto name = deviceID;
    utils::toLower(name);
    auto tokens = utils::tokenize(name, '\\');
    if(tokens.empty() || tokens[0] != R"(usb)")
        return "";  // Not a USB device
    if(tokens.size() < 3) {
        LOG_ERROR("malformed usb device path: {}", name);
        return "";
    }

    auto ids = utils::tokenize(tokens[2], '&');
    if(ids.empty()) {
        LOG_ERROR("malformed id string: {}", tokens[2]);
        return "";
    }

    if(ids.size() > 2)
        return ids[1];
    return "";
}

std::string getDeviceHubIdByWin(const std::string &devicePath) {
    auto        name = devicePath;
    utils::toLower(name);
    auto tokens = utils::tokenize(name, '#');
    if(tokens.empty() || (tokens[0] != R"(\\?\usb)" && tokens[0] != R"(\\?\hid)"))
        return "";  // Not a USB device
    if(tokens.size() < 3) {
        LOG_ERROR("malformed usb device path: {}", name);
        return "";
    }
    return tokens[2];
}
#endif

std::string getStringDescriptorStr(libusb_device *device, uint8_t desc_index) {
    std::string str;
    if(desc_index > 0) {
        // desc_index no equal zero

        libusb_device_handle *dev_handle = nullptr;

        auto sts = safe_open_device(device, &dev_handle);
        if(sts != LIBUSB_SUCCESS) {
            dev_handle = nullptr;
        }

        if(dev_handle) {
            uint8_t len;
            libusb_get_string_descriptor(dev_handle, desc_index, 0x0409, &len, 1);  // get length

            auto *str_desc = new uint8_t[len];
            libusb_get_string_descriptor(dev_handle, desc_index, 0x0409, str_desc, len);

            // string_descriptor:  byte0: bLength  byte1: bDescriptorType  byte2~len: bString

            for(int i = 2; i < len; i++) {
                if(str_desc[i] < 0x7f && str_desc[i] != 0) {  // range of ASCII: 0x00~0x7f
                    str += static_cast<char>(str_desc[i]);
                }
            }

            delete []str_desc;
        }
        safe_close_device(device, dev_handle);
    }
    return str;
}

std::vector<UsbDeviceInfo> getSubDevices(libusb_device *device, libusb_device_descriptor desc) {
#ifdef WIN32
    std::string    hub_id;
    libusb_device *parent_dev = libusb_get_parent(device);
    if(parent_dev != nullptr) {
        auto hub_dev_path = libusb_get_windows_path(parent_dev);
        if(hub_dev_path) {
            hub_id = getDeviceHubIdByWin(hub_dev_path);
        }
        // LOG_ERROR("hubid = " << hub_id;
    }
#endif

    std::vector<UsbDeviceInfo> rv;
    for(uint8_t c = 0; c < desc.bNumConfigurations; ++c) {
        libusb_config_descriptor *config = nullptr;
        auto                      ret    = libusb_get_config_descriptor(device, c, &config);
        if(LIBUSB_SUCCESS != ret) {
            LOG_WARN("failed to read USB config descriptor: error={}", ret);
            continue;
        }

        for(uint8_t i = 0; i < config->bNumInterfaces; ++i) {
            auto inf = config->interface[i];

            // avoid publish streaming interfaces TODO:MK
            if(inf.altsetting->bInterfaceSubClass == 2) {
                continue;
            }
            // when device is in DFU state, two USB devices are detected, one of OB_USB_CLASS_VENDOR_SPECIFIC (255) class
            // and the other of OB_USB_CLASS_APPLICATION_SPECIFIC (254) class.
            // in order to avoid listing two usb devices for a single physical device we ignore the application specific class
            // https://www.usb.org/defined-class-codes#anchor_BaseClassFEh
            if(inf.altsetting->bInterfaceClass == OB_USB_CLASS_APPLICATION_SPECIFIC || inf.altsetting->bInterfaceClass == OB_USB_CLASS_HUB) {
                continue;
            }

            UsbDeviceInfo info{};
            info.url = getDevicePath(device);
            if(info.url.empty()) {
                continue;
            }
            info.cls = static_cast<UsbClass>(inf.altsetting->bInterfaceClass);

#ifdef WIN32
            if(info.cls == OB_USB_CLASS_HID) {
                auto sub_composite_device_id = libusb_get_sub_composite_device_id(device, i);
                if(sub_composite_device_id != nullptr) {
                    info.uid = getDeviceUidByWinDeviceID(sub_composite_device_id);
                }
                auto interface_path = libusb_get_interface_path(device, i);
                if(interface_path != nullptr) {
                    info.infUrl = interface_path;
                }
            }
            else {
                auto interface_path = libusb_get_interface_path(device, i);
                if(interface_path != nullptr) {
                    info.infUrl = interface_path;
                    info.uid    = getDeviceUidByWin(interface_path);
                }
                else if(config->bNumInterfaces == 1) {  // 非复合设备
                    auto dev_path = libusb_get_windows_path(device);
                    info.infUrl   = dev_path;
                    if(dev_path != nullptr) {
                        info.uid = getDeviceUidByWin(dev_path);
                    }
                }
            }
            info.hubId = hub_id;
#else
            info.infUrl = info.url + "." + std::to_string(i);

            info.uid   = info.url;
            info.hubId = getDeviceHubID(info.url);
#endif

            info.conn_spec    = static_cast<UsbSpec>(desc.bcdUSB);
            info.vid          = desc.idVendor;
            info.pid          = desc.idProduct;
            info.infIndex     = i;
            info.infNameIndex = inf.altsetting->iInterface;

            // 由于字符串描述符需要每次从设备获取，为减少设备的通信访问（热拔插检测需要定时query），延后SN的获取
            // info.serial = getStringDescriptorStr(device, desc.iSerialNumber);

            // LOG_DEBUG("info.vid = " << info.vid
            //     << ", info.pid = " << info.pid
            //     << ", info.infIndex = " << (uint32_t)info.infIndex
            //     << ", info.serial = " << info.serial;
            rv.push_back(info);
        }

        libusb_free_config_descriptor(config);
    }
    return rv;
}

UsbEnumerator::UsbEnumerator() {
    usbCtx_ = std::make_shared<UsbContext>();
}

UsbEnumerator::~UsbEnumerator()noexcept {
    usbCtx_.reset();  // 析构后自动释放
}

const std::vector<UsbDeviceInfo> &UsbEnumerator::queryDevicesInfo() {
    // LOG_DEBUG("queryDevicesInfo start ...");  // 失败软重启较快，这个log会导致检测不到设备掉线

    std::vector<UsbDeviceInfo>             tempInfoVec;
    std::unique_lock<std::recursive_mutex> lock(usbCtxMutex_);
    usbCtx_->refreshDeviceList();

    for(size_t idx = 0; idx < usbCtx_->deviceCount(); ++idx) {
        auto device = usbCtx_->getDevice(static_cast<uint8_t>(idx));
        if(device == nullptr)
            continue;
        libusb_device_descriptor desc{};
        auto                     ret = libusb_get_device_descriptor(device, &desc);
        if(LIBUSB_SUCCESS == ret) {
            if(desc.idVendor == ORBBEC_USB_VID) {  // 通过vid==0X2bc5过滤掉非奥比设备
                auto subDevices = getSubDevices(device, desc);
                tempInfoVec.insert(tempInfoVec.end(), subDevices.begin(), subDevices.end());
            }
        }
        else {
            LOG_WARN("failed to read USB device descriptor: error={}", ret);
        }
    }

    // remove disconnected
    for(auto devInfoIter = devInfoList_.begin(); devInfoIter != devInfoList_.end();) {
        bool found = false;
        for(auto newDevInfoIter = tempInfoVec.begin(); newDevInfoIter != tempInfoVec.end(); ++newDevInfoIter) {
            if(*devInfoIter == *newDevInfoIter) {
                found = true;
                break;
            }
        }
        if(!found) {
            devInfoIter = devInfoList_.erase(devInfoIter);
            if(devInfoIter == devInfoList_.end()) {
                break;
            }
        }
        else {
            ++devInfoIter;
        }
    }

    // add new
    for(auto devInfoIter = tempInfoVec.begin(); devInfoIter != tempInfoVec.end(); ++devInfoIter) {
        bool found = false;
        for(auto oldDevInfoIter = devInfoList_.begin(); oldDevInfoIter != devInfoList_.end(); ++oldDevInfoIter) {
            if(*oldDevInfoIter == *devInfoIter) {
                found = true;
                break;
            }
        }
        if(!found) {
            getStringDesc(*devInfoIter);  // 从设备获取SN（字符串描述符）
            devInfoList_.push_back(*devInfoIter);
        }
    }

    LOG_DEBUG("queryDevicesInfo done!");
    return devInfoList_;
}

std::shared_ptr<UsbDevice> UsbEnumerator::createUsbDevice(const std::string &devUrl, const uint8_t retry) {
    std::unique_lock<std::recursive_mutex> lock(usbCtxMutex_);

    for(size_t idx = 0; idx < usbCtx_->deviceCount(); ++idx) {
        auto device = usbCtx_->getDevice(static_cast<uint8_t>(idx));

        if(device == nullptr || getDevicePath(device) != devUrl)
            continue;

        libusb_device_descriptor desc{};
        auto                     ret = libusb_get_device_descriptor(device, &desc);
        if(LIBUSB_SUCCESS == ret) {
            BEGIN_TRY_EXECUTE({ return std::make_shared<UsbDeviceLibusb>(device, desc, usbCtx_); })
            CATCH_EXCEPTION_AND_LOG(WARN, "failed to create usb device at index: {0}, url:{1}", (int)idx, devUrl)
        }
        else {
            LOG_WARN("failed to read USB device descriptor: error={}", ret);
        }
    }

    if(retry) {
        LOG_DEBUG("retry to create usb device: {}", devUrl);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        usbCtx_->refreshDeviceList();
        return createUsbDevice(devUrl, retry - 1);
    }
    return nullptr;
}

bool findSN2Toupper(const std::string &src, std::string &dst) {
    auto name = src;
    utils::toLower(name);
    auto tokens = utils::tokenize(name, '#');
    if(tokens.empty() || (tokens[0] != R"(\\?\usb)" && tokens[0] != R"(\\?\hid)"))
        return false;  // Not a USB device
    if(tokens.size() < 3) {
        return false;
    }

    if(tokens[2].find("&") != std::string::npos) {
        return false;
    }

    if(tokens[2].length() < 11) {
        return false;
    }
    std::transform(tokens[2].begin(), tokens[2].end(), std::back_inserter(dst),
                   [](const char &ch) { return static_cast<char>(std::toupper(static_cast<unsigned char>(ch))); });
    return true;
}

bool UsbEnumerator::getStringDesc(UsbDeviceInfo &info) {
    std::unique_lock<std::recursive_mutex> lock(usbCtxMutex_);

    for(size_t idx = 0; idx < usbCtx_->deviceCount(); ++idx) {
        auto device  = usbCtx_->getDevice(static_cast<uint8_t>(idx));
        auto devPath = getDevicePath(device);
        if(device == nullptr || devPath != info.url)
            continue;

        libusb_device_descriptor desc{};
        libusb_get_device_descriptor(device, &desc);
        info.serial  = getStringDescriptorStr(device, desc.iSerialNumber);
        info.infName = getStringDescriptorStr(device, info.infNameIndex);

#ifdef WIN32
        if(info.serial.empty()) {
            std::string toupperSNStr;
            if(findSN2Toupper(info.url, toupperSNStr)) {
                info.serial = toupperSNStr;
            }
        }
#endif

        return true;
    }

    LOG_WARN("Can not find device to get device serial number");
    return false;
}


}  // namespace libobsensor
#endif  // __ANDROID__