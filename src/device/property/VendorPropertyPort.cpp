#include "VendorPropertyPort.hpp"
#include "exception/ObException.hpp"
#include "HostProtocol.hpp"

namespace libobsensor {

VendorPropertyPort::VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend) : backend_(backend), recvData_(1024), sendData_(1024) {
    auto port = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    if(!port) {
        throw invalid_value_exception("VendorPropertyPort backend must be IVendorDataPort");
    }
}

void VendorPropertyPort::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initSetPropertyReq(sendData_.data(), propertyId, value.intValue);

    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    uint16_t respDataSize = 0;
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);
}

void VendorPropertyPort::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initGetPropertyReq(sendData_.data(), propertyId);

    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);

    protocol::checkStatus(res);

    auto resp       = protocol::parseGetPropertyResp(recvData_.data(), respDataSize);
    value->intValue = resp->data.cur;
}

void VendorPropertyPort::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initGetPropertyReq(sendData_.data(), propertyId);

    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);

    auto resp            = protocol::parseGetPropertyResp(recvData_.data(), respDataSize);
    range->cur.intValue  = resp->data.cur;
    range->max.intValue  = resp->data.max;
    range->min.intValue  = resp->data.min;
    range->step.intValue = resp->data.step;
    range->def.intValue  = resp->data.def;
}

void VendorPropertyPort::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initSetStructureDataReq(sendData_.data(), propertyId, data.data(), static_cast<uint16_t>(data.size()));

    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);
}

const std::vector<uint8_t> &VendorPropertyPort::getStructureData(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initGetStructureDataReq(sendData_.data(), propertyId);

    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);

    auto resp              = protocol::parseGetStructureDataResp(recvData_.data(), respDataSize);
    auto structureDataSize = protocol::getStructureDataSize(resp);
    outputData_.resize(structureDataSize);
    memcpy(outputData_.data(), resp->data, structureDataSize);
    return outputData_;
}

void VendorPropertyPort::clearBuffers() {
    memset(recvData_.data(), 0, recvData_.size());
    memset(sendData_.data(), 0, sendData_.size());
}

}  // namespace libobsensor