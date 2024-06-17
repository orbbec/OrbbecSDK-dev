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

void VendorPropertyPort::getCmdVersionProtoV11(uint32_t propertyId, uint16_t *version) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto     req          = protocol::initGetCmdVersionReq(sendData_.data(), propertyId);
    uint16_t respDataSize = 64;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);

    auto resp = protocol::parseGetCmdVerDataResp(recvData_.data(), respDataSize);
    *version  = *(uint16_t *)(resp->data);
}

void VendorPropertyPort::getRawData(uint32_t propertyId, get_data_callback callback, uint32_t transPacketSize) {
    std::lock_guard<std::mutex> lock(mutex_);

    OBDataTranState tranState = DATA_TRAN_STAT_TRANSFERRING;
    OBDataChunk     dataChunk = { sendData_.data(), 0, 0, 0 };
    uint32_t        dataSize;

    // init
    {
        clearBuffers();
        auto     req          = protocol::initGetRawData(sendData_.data(), propertyId, 0);
        uint16_t respDataSize = 64;
        auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
        auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);
        auto resp = protocol::parseGetReadDataResp(recvData_.data(), respDataSize);
        dataSize  = resp->dataSize;
    }

    for(uint32_t packetOffset = 0; packetOffset < dataSize; packetOffset += transPacketSize) {
        uint32_t packetLen = std::min(transPacketSize, dataSize - packetOffset);
        clearBuffers();
        auto     req          = protocol::initReadRawData(sendData_.data(), propertyId, packetOffset, packetLen);
        uint16_t respDataSize = 1024;
        auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
        auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);

        if(callback) {
            dataChunk.data         = recvData_.data() + sizeof(protocol::RespHeader);
            dataChunk.size         = packetLen;
            dataChunk.offset       = packetOffset;
            dataChunk.fullDataSize = dataSize;
            if(packetOffset + packetLen >= dataSize) {
                tranState = DATA_TRAN_STAT_DONE;
            }
            callback(tranState, &dataChunk);
        }
    }

    // finish
    {
        clearBuffers();
        auto     req          = protocol::initGetRawData(sendData_.data(), propertyId, 1);
        uint16_t respDataSize = 64;
        auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
        auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);
    }
}


void VendorPropertyPort::clearBuffers() {
    memset(recvData_.data(), 0, recvData_.size());
    memset(sendData_.data(), 0, sendData_.size());
}

}  // namespace libobsensor