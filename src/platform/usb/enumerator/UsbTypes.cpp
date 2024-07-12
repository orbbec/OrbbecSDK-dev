#include "UsbTypes.hpp"
namespace libobsensor {

SourcePortType cvtUsbClassToPortType(uint8_t cls) {
    switch(static_cast<libusb_class_code>(cls)) {
    case LIBUSB_CLASS_HID:
        return SOURCE_PORT_USB_HID;
    case LIBUSB_CLASS_VIDEO:
        return SOURCE_PORT_USB_UVC;
    case LIBUSB_CLASS_VENDOR_SPEC:
        return SOURCE_PORT_USB_VENDOR;
    default:
        break;
    }
    return SOURCE_PORT_UNKNOWN;
}

}  // namespace libobsensor
