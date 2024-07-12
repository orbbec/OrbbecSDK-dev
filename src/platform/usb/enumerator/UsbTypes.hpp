// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "ISourcePort.hpp"

#include <libusb.h>

#include <map>
#include <sstream>

#define ORBBEC_USB_VID 0x2BC5

namespace libobsensor {

typedef enum _endpointDirection { OB_USB_ENDPOINT_DIRECTION_WRITE = 0, OB_USB_ENDPOINT_DIRECTION_READ = 0x80 } EndpointDirection;

// Binary-coded decimal represent the USB specification to which the UVC device
// complies
typedef enum _usbSpec {
    usb_undefined = 0,
    usb1_type     = 0x0100,
    usb1_1_type   = 0x0110,
    usb2_type     = 0x0200,
    usb2_1_type   = 0x0210,
    usb3_type     = 0x0300,
    usb3_1_type   = 0x0310,
    usb3_2_type   = 0x0320,
} UsbSpec;

static const std::map<UsbSpec, std::string> usb_spec_names = { { usb_undefined, "USB" },  { usb1_type, "USB1.0" },   { usb1_1_type, "USB1.1" },
                                                               { usb2_type, "USB2.0" },   { usb2_1_type, "USB2.1" }, { usb3_type, "USB3.0" },
                                                               { usb3_1_type, "USB3.1" }, { usb3_2_type, "USB3.2" } };

struct UsbInterfaceInfo {
    std::string url;
    std::string uid;
    std::string hubId;
    uint16_t    vid = 0;
    uint16_t    pid = 0;
    std::string serial;
    UsbSpec     conn_spec = usb_undefined;

    uint8_t     infIndex         = 0;
    uint8_t     infNameDescIndex = 0;  // iInterface (index of string description of interface)
    std::string infName;
    std::string infUrl;  // to distinguish between different pins of the same device
    uint8_t     cls = LIBUSB_CLASS_VENDOR_SPEC;

    operator std::string() {
        std::stringstream s;

        s << "vid- " << std::hex << vid << "\npid- " << std::hex << pid << "\ninfIndex- " << (uint32_t)infIndex << "\nusb specification- " << std::hex
          << (uint16_t)conn_spec << std::dec << "\nuid- " << uid;

        return s.str();
    }
};

inline bool operator==(const UsbInterfaceInfo &a, const UsbInterfaceInfo &b) {
    return (a.vid == b.vid) && (a.pid == b.pid) && (a.infIndex == b.infIndex) && (a.uid == b.uid) && (a.infUrl == b.infUrl);
}

struct UsbDevice {
    libusb_context                       *context   = nullptr;
    std::shared_ptr<libusb_device_handle> devHandle = nullptr;
};

SourcePortType cvtUsbClassToPortType(uint8_t cls);

}  // namespace libobsensor