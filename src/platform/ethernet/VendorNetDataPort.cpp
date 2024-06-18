#include "VendorNetDataPort.hpp"
#include "exception/ObException.hpp"

#include <algorithm>

namespace libobsensor {

VendorNetDataPort::VendorNetDataPort(std::shared_ptr<const NetSourcePortInfo> portInfo) : portInfo_(portInfo) {
    auto noConstPortInfo = std::const_pointer_cast<NetSourcePortInfo>(portInfo);
    tcpClient_ = std::make_shared<VendorTCPClient>(noConstPortInfo->address, noConstPortInfo->port);
}
VendorNetDataPort::~VendorNetDataPort() {}

bool VendorNetDataPort::sendData(const uint8_t *data, const uint32_t dataLen) {
    BEGIN_TRY_EXECUTE(tcpClient_->write(data, dataLen))
    CATCH_EXCEPTION_AND_EXECUTE(return false)
    return true;
}

bool VendorNetDataPort::recvData(uint8_t *data, uint32_t *dataLen) {
    int readSize = 0;
    *dataLen     = 0;
    BEGIN_TRY_EXECUTE({ readSize = tcpClient_->read(data, 1024); })
    CATCH_EXCEPTION_AND_EXECUTE(return false)
    *dataLen = (std::max)(0, readSize);
    return readSize > 0;
}

std::shared_ptr<const SourcePortInfo> VendorNetDataPort::getSourcePortInfo() const {
    return portInfo_;
}

}  // namespace libobsensor