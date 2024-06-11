#include "UsbTypes.hpp"
namespace libobsensor {
namespace pal {
SourcePortType cvtUsbClassToPortType(UsbClass cls) {
    switch(cls) {
    case OB_USB_CLASS_HID:
        return SOURCE_PORT_USB_HID;
    case OB_USB_CLASS_VIDEO:
        return SOURCE_PORT_USB_UVC;
    case OB_USB_CLASS_VENDOR_SPECIFIC:
        return SOURCE_PORT_USB_VENDOR;
    default:
        break;
    }
    return SOURCE_PORT_UNKNOWN;
}
}  // namespace pal
}  // namespace libobsensor
