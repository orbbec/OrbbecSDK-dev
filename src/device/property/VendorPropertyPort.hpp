#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"
#include <mutex>

namespace libobsensor {

class VendorPropertyPort : public IPropertyExtensionPort {
public:
    VendorPropertyPort(const std::shared_ptr<ISourcePort> &backend);
    ~VendorPropertyPort() noexcept override = default;

    void                        setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void                        getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void                        getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;
    void                        setStructureData(uint32_t propertyId, const std::vector<uint8_t> &data) override;
    const std::vector<uint8_t> &getStructureData(uint32_t propertyId) override;
    void                        getCmdVersionProtoV11(uint32_t propertyId, uint16_t *version) override;
    void                        getRawData(uint32_t propertyId, get_data_callback callback, uint32_t transPacketSize) override;

private:
    void clearBuffers();

private:
    std::shared_ptr<ISourcePort> backend_;
    std::mutex                   mutex_;
    std::vector<uint8_t>         recvData_;
    std::vector<uint8_t>         sendData_;
    std::vector<uint8_t>         outputData_;

public:
    template <typename T, uint16_t CMD_VER> T getStructureDataProtoV11(uint32_t propertyId) {
        std::lock_guard<std::mutex> lock(mutex_);
        clearBuffers();

        auto     req          = initGetStructureDataV11Req(sendData_.data(), propertyId);
        uint16_t respDataSize = 0;
        auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
        auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);

        auto resp = protocol::parseGetStructureDataV11Resp(recvData_.data(), respDataSize);
        if(resp->cmdVer != CMD_VER) {
            res.status_code     = HP_STATUS_DEVICE_RESPONSE_CMD_VERSION_UNMATCHED;
            res.resp_error_code = HP_RESP_ERROR_UNKNOWN;
            res.msg             = rsutils::string::from() << "response with wrong cmd version: " << ver << ", expect: " << CMD_VER;
            protocol::checkStatus(res);
        }

        T structureData;
        memcpy(&structureData, resp->data, sizeof(T));
        return structureData;
    }

    template <typename T, uint16_t CMD_VER> T getStructureDataListProtoV11(uint32_t propertyId, uint32_t tranPacketSize) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t                    dataSize;
        std::vector<T>              structureDataList;
        clearBuffers();
        auto     req          = protocol::initStartGetStructureDataList(sendData_.data(), propertyId);
        uint16_t respDataSize = 64;
        auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
        auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
        protocol::checkStatus(res);

        auto resp = protocol::parseGetStructureDataV11Resp(recvData_.data(), respDataSize);
        if(resp->cmdVer != CMD_VER) {
            res.status_code     = HP_STATUS_DEVICE_RESPONSE_CMD_VERSION_UNMATCHED;
            res.resp_error_code = HP_RESP_ERROR_UNKNOWN;
            res.msg             = "init get structure data list return cmd version unmatched";
            protocol::checkStatus(res);
        }
        auto resp = protocol::parseGetReadDataResp(recvData_.data(), respDataSize);
        dataSize  = resp->dataSize;

        {
            std::vector<uint8_t> dataVec;
            for(uint32_t packetOffset = 0; packetOffset < dataSize; packetOffset += tranPacketSize) {
                clearBuffers();  // reset request and response buffer cache
                uint32_t packetSize = std::min(tranPacketSize, dataSize - packetOffset);

                auto     req          = protocol::initReadStructureDataList(sendData_.data(), propertyId, packetOffset, packetSize);
                uint16_t respDataSize = 1024;
                auto     port         = std::dynamic_pointer_cast<IVendorDataPort>(backend_);
                auto     res          = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
                if(!protocol::checkStatus(res)) {
                    break;
                }

                protocol::RespHeader *header = (protocol::RespHeader *)recvData_.data();
                dataVec.insert(dataVec.end(), recvData_.data() + protocol::HP_RESP_HEADER_SIZE,
                                recvData_.data() + protocol::HP_RESP_HEADER_SIZE + packetSize);
            }
            if(dataVec.size() == dataSize) {
                for(uint32_t offset = 0; offset < dataSize; offset += sizeof(T)) {
                    structureDataList.push_back(*(T *)(dataVec.data() + offset));
                }
            }
        }

        {
            clearBuffers();
            auto req = protocol::initFinishGetStructureDataList(sendData_.data(), propertyId);
            auto res = protocol::execute(port, sendData_.data(), sizeof(req), recvData_.data(), &respDataSize);
            protocol::checkStatus(res);
        }

        protocol::checkStatus(res);
        return structureDataList;
    }
};
}  // namespace libobsensor
