#include "VendorUsbDevicePort.hpp"

#include "logger/Logger.hpp"
#include <iomanip>
#include <sstream>
#include <algorithm>

#ifdef __ANDROID__
#include "usb/backend/DeviceLibusb.hpp"
#include <libusb.h>
#endif

#include "utils/Utils.hpp"

namespace libobsensor {


VendorUsbDevicePort::VendorUsbDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo)
    : portInfo_(portInfo), usbDev_(usbDevice) {
    auto vendorIntf = usbDev_->getInterface(portInfo->infIndex);
    if(!vendorIntf || vendorIntf->getClass() != OB_USB_CLASS_VENDOR_SPECIFIC) {
        throw std::runtime_error("can't find VENDOR_SPECIFIC interface of device.");
    }
    usbMessenger_ = usbDev_->open(vendorIntf->getNumber());
    if(usbMessenger_ == nullptr) {
        throw std::runtime_error("failed to open usb");
    }
    bulkWriteEndpoint_ = vendorIntf->firstEndpoint(OB_USB_ENDPOINT_DIRECTION_WRITE);
}

VendorUsbDevicePort::~VendorUsbDevicePort() noexcept = default;

uint32_t VendorUsbDevicePort::sendAndReceive(const uint8_t* sendData, uint32_t sendLen, uint8_t* recvData, uint32_t exceptedRecvLen) {
    uint32_t transferred = 0;
    auto     sendBuf     = const_cast<uint8_t *>(sendData);
    auto     ret         = usbMessenger_->controlTransfer(0x40, 0, 0, 0, sendBuf, sendLen, transferred, 5000);
    if(ret != OB_USB_STATUS_SUCCESS) {
        LOG_WARN("control transfer send data failed: {}", ret);
        return 0;
    }
    ret = usbMessenger_->controlTransfer(0xc0, 0, 0, 0, recvData, exceptedRecvLen, transferred, 5000);
    if(ret != OB_USB_STATUS_SUCCESS) {
        LOG_WARN("control transfer recv data failed: {}", ret);
        return {};
    }
    return transferred;
}

bool VendorUsbDevicePort::readFromBulkEndPoint(std::vector<uint8_t> &data) {
    utils::unusedVar(data);
    return false;
}

bool VendorUsbDevicePort::writeToBulkEndPoint(std::vector<uint8_t> &data) {
    uint32_t transferred = 0;
    auto     ret         = usbMessenger_->bulkTransfer(bulkWriteEndpoint_, data.data(), (uint32_t)data.size(), transferred, 5000);
    return ret == OB_USB_STATUS_SUCCESS;
}

std::shared_ptr<const SourcePortInfo> VendorUsbDevicePort::getSourcePortInfo() const {
    return portInfo_;
}

#ifdef __ANDROID__
std::string VendorUsbDevicePort::getUsbConnectType() {
    auto                     libusbDev = std::static_pointer_cast<UsbDeviceLibusb>(usbDev_);
    libusb_device_descriptor desc;
    auto                     ret = libusb_get_device_descriptor(libusbDev->getDevice(), &desc);
    return usb_spec_names.find(UsbSpec(desc.bcdUSB))->second;
}
#endif


}  // namespace libobsensor
