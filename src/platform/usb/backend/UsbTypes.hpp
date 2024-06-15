// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include <map>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>
#include "ISourcePort.hpp"

#define USB_DT_DEVICE 0x01
#define USB_DT_CONFIG 0x02
#define USB_DT_STRING 0x03
#define USB_DT_INTERFACE 0x04
#define USB_DT_ENDPOINT 0x05
#define USB_DT_DEVICE_QUALIFIER 0x06
#define USB_DT_OTHER_SPEED_CONFIG 0x07
#define USB_DT_INTERFACE_POWER 0x08
#define USB_DT_OTG 0x09
#define USB_DT_DEBUG 0x0a
#define USB_DT_INTERFACE_ASSOCIATION 0x0b
#define USB_DT_SECURITY 0x0c
#define USB_DT_KEY 0x0d
#define USB_DT_ENCRYPTION_TYPE 0x0e
#define USB_DT_BOS 0x0f
#define USB_DT_DEVICE_CAPABILITY 0x10
#define USB_DT_WIRELESS_ENDPOINT_COMP 0x11
#define USB_DT_WIRE_ADAPTER 0x21
#define USB_DT_RPIPE 0x22
#define USB_DT_CS_RADIO_CONTROL 0x23
#define USB_DT_PIPE_USAGE 0x24
#define USB_DT_SS_ENDPOINT_COMP 0x30
#define USB_DT_SSP_ISOC_ENDPOINT_COMP 0x31
#define USB_DT_CS_DEVICE (USB_TYPE_CLASS | USB_DT_DEVICE)
#define USB_DT_CS_CONFIG (USB_TYPE_CLASS | USB_DT_CONFIG)
#define USB_DT_CS_STRING (USB_TYPE_CLASS | USB_DT_STRING)
#define USB_DT_CS_INTERFACE (USB_TYPE_CLASS | USB_DT_INTERFACE)
#define USB_DT_CS_ENDPOINT (USB_TYPE_CLASS | USB_DT_ENDPOINT)

#define ORBBEC_USB_VID 0x2BC5

namespace libobsensor {

typedef enum _usbStatus {
    OB_USB_STATUS_SUCCESS       = 0,
    OB_USB_STATUS_IO            = -1,
    OB_USB_STATUS_INVALID_PARAM = -2,
    OB_USB_STATUS_ACCESS        = -3,
    OB_USB_STATUS_NO_DEVICE     = -4,
    OB_USB_STATUS_NOT_FOUND     = -5,
    OB_USB_STATUS_BUSY          = -6,
    OB_USB_STATUS_TIMEOUT       = -7,
    OB_USB_STATUS_OVERFLOW      = -8,
    OB_USB_STATUS_PIPE          = -9,
    OB_USB_STATUS_INTERRUPTED   = -10,
    OB_USB_STATUS_NO_MEM        = -11,
    OB_USB_STATUS_NOT_SUPPORTED = -12,
    OB_USB_STATUS_OTHER         = -13

} UsbStatus;

typedef enum _endpointDirection { OB_USB_ENDPOINT_DIRECTION_WRITE = 0, OB_USB_ENDPOINT_DIRECTION_READ = 0x80 } EndpointDirection;

typedef enum _endpointType {
    OB_USB_ENDPOINT_CONTROL,
    OB_USB_ENDPOINT_ISOCHRONOUS,
    OB_USB_ENDPOINT_BULK,
    OB_USB_ENDPOINT_INTERRUPT,

} EndpointType;

// https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/supported-usb-classes#microsoft-provided-usb-device-class-drivers
typedef enum _usbClass {
    OB_USB_CLASS_UNSPECIFIED          = 0x00,
    OB_USB_CLASS_AUDIO                = 0x01,
    OB_USB_CLASS_COM                  = 0x02,
    OB_USB_CLASS_HID                  = 0x03,
    OB_USB_CLASS_PID                  = 0x05,
    OB_USB_CLASS_IMAGE                = 0x06,
    OB_USB_CLASS_PRINTER              = 0x07,
    OB_USB_CLASS_MASS_STORAGE         = 0x08,
    OB_USB_CLASS_HUB                  = 0x09,
    OB_USB_CLASS_CDC_DATA             = 0x0A,
    OB_USB_CLASS_SMART_CARD           = 0x0B,
    OB_USB_CLASS_CONTENT_SECURITY     = 0x0D,
    OB_USB_CLASS_VIDEO                = 0x0E,
    OB_USB_CLASS_PHDC                 = 0x0F,
    OB_USB_CLASS_AV                   = 0x10,
    OB_USB_CLASS_BILLBOARD            = 0x11,
    OB_USB_CLASS_DIAGNOSTIC_DEVICE    = 0xDC,
    OB_USB_CLASS_WIRELESS_CONTROLLER  = 0xE0,
    OB_USB_CLASS_MISCELLANEOUS        = 0xEF,
    OB_USB_CLASS_APPLICATION_SPECIFIC = 0xFE,
    OB_USB_CLASS_VENDOR_SPECIFIC      = 0xFF
} UsbClass;

typedef enum _usbRequestStatus {
    /** Transfer completed without error. Note that this does not indicate
     * that the entire amount of requested data was transferred. */
    OB_USB_TRANSFER_COMPLETED,

    /** Transfer failed */
    OB_USB_TRANSFER_ERROR,

    /** Transfer timed out */
    OB_USB_TRANSFER_TIMED_OUT,

    /** Transfer was cancelled */
    OB_USB_TRANSFER_CANCELLED,

    /** For bulk/interrupt endpoints: halt condition detected (endpoint
     * stalled). For control endpoints: control request not supported. */
    OB_USB_TRANSFER_STALL,

    /** Device was disconnected */
    OB_USB_TRANSFER_NO_DEVICE,

    /** Device sent more data than requested */
    OB_USB_TRANSFER_OVERFLOW,
} usbRequestStatus;

typedef enum _usbSubclass { OB_USB_SUBCLASS_VIDEO_CONTROL = 0x01, OB_USB_SUBCLASS_VIDEO_STREAMING = 0x02 } UsbSubclass;

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

struct UsbDeviceInfo {
    std::string url       = "";
    std::string uid       = "";
    std::string hubId     = "";
    uint16_t    vid       = 0;
    uint16_t    pid       = 0;
    std::string serial    = "";
    UsbSpec     conn_spec = usb_undefined;

    uint8_t     infIndex     = 0;
    uint8_t     infNameIndex = 0;  // iInterface (index of string description of interface)
    std::string infName      = "";
    std::string infUrl       = "";  // to distinguish between different pins of the same device
    UsbClass    cls          = OB_USB_CLASS_UNSPECIFIED;

    operator std::string() {
        std::stringstream s;

        s << "vid- " << std::hex << vid << "\npid- " << std::hex << pid << "\ninfIndex- " << (uint32_t)infIndex << "\nusb specification- " << std::hex
          << (uint16_t)conn_spec << std::dec << "\nuid- " << uid;

        return s.str();
    }
};

inline bool operator==(const UsbDeviceInfo &a, const UsbDeviceInfo &b) {
    return (a.vid == b.vid) && (a.pid == b.pid) && (a.infIndex == b.infIndex) && (a.uid == b.uid) && (a.infUrl == b.infUrl);
}

static std::map<UsbStatus, std::string> usbStatusToString = {
    { OB_USB_STATUS_SUCCESS, "OB_USB_STATUS_SUCCESS" },
    { OB_USB_STATUS_IO, "OB_USB_STATUS_IO" },
    { OB_USB_STATUS_INVALID_PARAM, "OB_USB_STATUS_INVALID_PARAM" },
    { OB_USB_STATUS_ACCESS, "OB_USB_STATUS_ACCESS" },
    { OB_USB_STATUS_NO_DEVICE, "OB_USB_STATUS_NO_DEVICE" },
    { OB_USB_STATUS_NOT_FOUND, "OB_USB_STATUS_NOT_FOUND" },
    { OB_USB_STATUS_BUSY, "OB_USB_STATUS_BUSY" },
    { OB_USB_STATUS_TIMEOUT, "OB_USB_STATUS_TIMEOUT" },
    { OB_USB_STATUS_OVERFLOW, "OB_USB_STATUS_OVERFLOW" },
    { OB_USB_STATUS_PIPE, "OB_USB_STATUS_PIPE" },
    { OB_USB_STATUS_INTERRUPTED, "OB_USB_STATUS_INTERRUPTED" },
    { OB_USB_STATUS_NO_MEM, "OB_USB_STATUS_NO_MEM" },
    { OB_USB_STATUS_NOT_SUPPORTED, "OB_USB_STATUS_NOT_SUPPORTED" },
    { OB_USB_STATUS_OTHER, "OB_USB_STATUS_OTHER" },
};

struct UsbConfigDescriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
};

struct UsbInterfaceDescriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};

struct UsbDescriptor {
    uint8_t              length;
    uint8_t              type;
    std::vector<uint8_t> data;
};

SourcePortType cvtUsbClassToPortType(UsbClass cls);

// For openni
typedef struct OBUSBEndPointHandle *OB_USB_EP_HANDLE;

typedef struct OBUsbConnection {
    OB_USB_EP_HANDLE UsbEp;
    bool             bIsOpen;
    uint32_t         nMaxPacketSize;
} OBUsbConnection;

typedef std::function<bool(unsigned char *pBuffer, uint32_t nBufferSize, void *pCallbackData)> OpenNIDataReadCallbackFunc;


}  // namespace libobsensor