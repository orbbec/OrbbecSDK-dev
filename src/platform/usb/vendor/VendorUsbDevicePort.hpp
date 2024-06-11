#pragma once
#include "ISourcePort.hpp"
#include "usb/backend/Device.hpp"

namespace libobsensor {
namespace pal {
class VendorUsbDevicePort : public IVendorDataPort {
public:
    VendorUsbDevicePort(const std::shared_ptr<UsbDevice> &usbDevice, std::shared_ptr<const USBSourcePortInfo> portInfo);
    ~VendorUsbDevicePort() noexcept override;

    std::vector<uint8_t> sendAndReceive(const std::vector<uint8_t> &sendData, uint32_t exceptedLength) ;

    virtual bool readFromBulkEndPoint(std::vector<uint8_t> &data);  // bulk transfer
    virtual bool writeToBulkEndPoint(std::vector<uint8_t> &data);   // bulk transfer

    std::shared_ptr<const SourcePortInfo> getSourcePortInfo() const override;

#ifdef __ANDROID__
    virtual std::string getUsbConnectType();
#endif

protected:
    std::shared_ptr<const USBSourcePortInfo> portInfo_;
    std::shared_ptr<UsbDevice>               usbDev_;

    std::shared_ptr<UsbMessenger> usbMessenger_;
    std::shared_ptr<UsbEndpoint>  bulkWriteEndpoint_;
};

}  // namespace pal
}  // namespace libobsensor
