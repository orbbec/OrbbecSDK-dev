#include "G330Firmware.hpp"
#include "common/exception/ObException.hpp"
#include "common/utility/ObUtils.hpp"
#include "common/logger/Logger.hpp"
#include <fstream>
#include <memory>
#include <functional>
namespace libobsensor {


G330Firmware::G330Firmware(const uint8_t *data, size_t dataSize) {
    if(dataSize < sizeof(uint32_t)) {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Parse firmware data failed:" << " invalid input data or data has been damaged");
    }
    fileHeader_.headerSize = *(uint32_t *)data;
    if(dataSize < fileHeader_.headerSize || fileHeader_.headerSize > sizeof(G330FirmwareFileHeader)) {
        throw libobsensor::invalid_value_exception(ObUtils::to_string()
                                                   << "Parse firmware file header failed:" << " invalid input data or data has been damaged");
    }
    std::memcpy(&fileHeader_, data, fileHeader_.headerSize);
    LOG_DEBUG("File header: serial={}, firmwareDataCount={}, timestamp={}", fileHeader_.serial, fileHeader_.firmwareDataCount, fileHeader_.timestamp);

    auto offset = fileHeader_.headerSize;
    for(int i = 0; i < fileHeader_.firmwareDataCount; i++) {
        g330_firmware_data firmwareData;
        firmwareData.header.headerSize = *(uint32_t *)(data + offset);
        if(dataSize < offset + firmwareData.header.headerSize || firmwareData.header.headerSize > sizeof(G330FirmwareDataHeader)) {
            throw libobsensor::invalid_value_exception(ObUtils::to_string()
                                                       << "Parse firmware data header failed:" << " invalid input data or data has been damaged");
        }
        std::memcpy(&firmwareData.header, data + offset, firmwareData.header.headerSize);
        if(firmwareData.header.adapterDevPidList[0] == 0x0000) {
            // set first value to 0xffff to compatible with old version firmware file
            firmwareData.header.adapterDevPidList[0] = G330_FIRMWARE_DATA_ADAPTER_ALL_PIDS;
        }
        LOG_DEBUG("firmwareData header: name={}, version={}, headerSize={}, dataSize={}, actualSize={}, address={}, adapterDevPidList[0]={}",
                  firmwareData.header.name, firmwareData.header.version, firmwareData.header.headerSize, firmwareData.header.dataSize,
                  firmwareData.header.actualSize, firmwareData.header.flash_address, firmwareData.header.adapterDevPidList[0]);

        offset += firmwareData.header.headerSize;
        if(dataSize < offset + firmwareData.header.dataSize) {
            throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Parse firmware data failed:" << " invalid input data or data has been damaged");
        }
        firmwareData.data.resize(firmwareData.header.dataSize);
        std::memcpy(firmwareData.data.data(), data + offset, firmwareData.header.dataSize);
        checkFirmwareData(firmwareData);

        offset += firmwareData.header.dataSize;
        firmwareDataList_.push_back(firmwareData);
    }
    LOG_DEBUG("{} firmware data successfully loaded.", firmwareDataList_.size());
}

const std::vector<g330_firmware_data> &G330Firmware::getFirmwareDataList() const {
    return firmwareDataList_;
}

const G330FirmwareFileHeader &G330Firmware::getFileHeader() const {
    return fileHeader_;
}

bool G330Firmware::isFirmwareDataAdaptable(const g330_firmware_data &firmwareData, uint16_t pid) {
    auto listSize = sizeof(firmwareData.header.adapterDevPidList) / sizeof(firmwareData.header.adapterDevPidList[0]);
    if(firmwareData.header.adapterDevPidList[0] == G330_FIRMWARE_DATA_ADAPTER_ALL_PIDS) {
        return true;
    }
    for(int i = 0; i < listSize; i++) {
        if(firmwareData.header.adapterDevPidList[i] == pid && firmwareData.header.adapterDevPidList[i] != 0x0000) {
            return true;
        }
    }
    return false;
}

void G330Firmware::checkFirmwareData(const g330_firmware_data &firmwareData) {
    uint32_t checksum = 0;
    for(auto byte: firmwareData.data) {
        checksum += byte;
    }
    if(firmwareData.header.checksum != checksum) {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Checksum mismatch: " << " invalid input file or file has been damaged");
    }
}


}  // namespace libobsensor
