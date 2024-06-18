#include "VendorPropertyPort.hpp"
#include "exception/ObException.hpp"
#include "HostProtocol.hpp"

namespace libobsensor {

template <uint16_t CMD_VER>
VendorPropertyPort<CMD_VER>::VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend) : backend_(backend), recvData_(1024), sendData_(1024) {
    auto port = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    if(!port) {
        throw invalid_value_exception("VendorPropertyPort backend must be IVendorDataPort");
    }
}

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initSetPropertyReq(sendData_.data(), propertyId, value.intValue);

    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    uint16_t respDataSize = 0;
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);
}

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
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

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
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

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
    auto req = protocol::initSetStructureDataReq(sendData_.data(), propertyId, data.data(), static_cast<uint16_t>(data.size()));

    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);
}

template <uint16_t CMD_VER> const std::vector<uint8_t> &VendorPropertyPort<CMD_VER>::getStructureData(uint32_t propertyId) {
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

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::getCmdVersionProtoV11(uint32_t propertyId, uint16_t *version) {
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

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::getRawData(uint32_t propertyId, get_data_callback callback, uint32_t transPacketSize) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();
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

template <uint16_t CMD_VER> std::vector<uint8_t> VendorPropertyPort<CMD_VER>::getStructureDataProtoV11(uint32_t propertyId) {
    std::lock_guard<std::mutex> lock(mutex_);
    clearBuffers();

    auto     req          = protocol::initGetStructureDataV11Req(sendData_.data(), propertyId);
    uint16_t respDataSize = 0;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);

    auto resp = protocol::parseGetStructureDataV11Resp(recvData_.data(), respDataSize);
    if(resp->cmdVer != CMD_VER) {
        res.statusCode    = protocol::HP_STATUS_DEVICE_RESPONSE_CMD_VERSION_UNMATCHED;
        res.respErrorCode = protocol::HP_RESP_ERROR_UNKNOWN;
        res.msg           = "get structure data return cmd version unmatched" + std::to_string(resp->cmdVer) + " expect: " + std::to_string(CMD_VER);
        protocol::checkStatus(res);
    }

    uint16_t             dataSize = protocol::getProtoV11StructureDataSize(resp);
    std::vector<uint8_t> structureData;
    memcpy(structureData.data(), resp->data, dataSize);
    return structureData;
}

template <uint16_t CMD_VER>
std::vector<std::vector<uint8_t>> VendorPropertyPort<CMD_VER>::getStructureDataListProtoV11(uint32_t propertyId, uint32_t tranPacketSize) {
    std::lock_guard<std::mutex>       lock(mutex_);
    uint32_t                          dataSize = 0;
    std::vector<std::vector<uint8_t>> structureDataList{};
    clearBuffers();
    auto     req          = protocol::initStartGetStructureDataList(sendData_.data(), propertyId);
    uint16_t respDataSize = 64;
    auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
    auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
    protocol::checkStatus(res);

    auto resp = protocol::parseInitStructureDataListResp(recvData_.data(), respDataSize);
    if(resp->cmdVer != CMD_VER) {
        res.statusCode    = protocol::HP_STATUS_DEVICE_RESPONSE_CMD_VERSION_UNMATCHED;
        res.respErrorCode = protocol::HP_RESP_ERROR_UNKNOWN;
        res.msg           = "init get structure data list return cmd version unmatched";
        protocol::checkStatus(res);
    }
    auto resp1 = protocol::parseGetReadDataResp(recvData_.data(), respDataSize);
    dataSize   = resp1->dataSize;

    {
        std::vector<uint8_t> dataVec;
        for(uint32_t packetOffset = 0; packetOffset < dataSize; packetOffset += tranPacketSize) {
            clearBuffers();  // reset request and response buffer cache
            uint32_t packetSize = std::min(tranPacketSize, dataSize - packetOffset);

            auto req1    = protocol::initReadStructureDataList(sendData_.data(), propertyId, packetOffset, packetSize);
            respDataSize = 1024;
            port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
            res          = protocol::execute(port, sendData_.data(), sizeof(req1), recvData_.data(), &respDataSize);
            if(!protocol::checkStatus(res)) {
                break;
            }

            dataVec.insert(dataVec.end(), recvData_.data() + sizeof(protocol::RespHeader), recvData_.data() + sizeof(protocol::RespHeader) + packetSize);
        }
        if(dataVec.size() == dataSize) {
            for(uint32_t offset = 0; offset < dataSize; offset += sizeof(std::vector<uint8_t>)) {
                // FIXME: check if the size is correct
                structureDataList.push_back(*(std::vector<uint8_t> *)(dataVec.data() + offset));
            }
        }
    }

    {
        clearBuffers();
        req = protocol::initFinishGetStructureDataList(sendData_.data(), propertyId);
        res = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);
    }

    protocol::checkStatus(res);
    return structureDataList;
}

template <uint16_t CMD_VER> void VendorPropertyPort<CMD_VER>::clearBuffers() {
    memset(recvData_.data(), 0, recvData_.size());
    memset(sendData_.data(), 0, sendData_.size());
}

template class VendorPropertyPort<0>;
template class VendorPropertyPort<1>;
}  // namespace libobsensor