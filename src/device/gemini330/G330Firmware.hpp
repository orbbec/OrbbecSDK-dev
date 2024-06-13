#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace libobsensor {


#pragma pack(1)
typedef struct {
    uint32_t headerSize;
    uint32_t firmwareDataCount;
    char     timestamp[16];
    char     serial[16];
    char     reserved[24];
} G330FirmwareFileHeader;

#define G330_FIRMWARE_DATA_ADAPTER_ALL_PIDS 0xFFFF
typedef struct {
    uint32_t headerSize;
    uint32_t dataSize;
    uint64_t flash_address;
    uint32_t actualSize;
    uint32_t checksum;
    char     version[16];
    char     name[64];

    /**
     * @brief adapter device pid list
     * If the first element is 0xFFFF, it means all devices are applicable.
     * If the first element is 0x0000, it means invalid pid.
     */
    uint16_t adapterDevPidList[12];
} G330FirmwareDataHeader;

typedef struct {
    G330FirmwareDataHeader header;
    std::vector<uint8_t>  data;
} g330_firmware_data;
#pragma pack()

class G330Firmware {
public:
    G330Firmware(const uint8_t *data, size_t size);
    const std::vector<g330_firmware_data> &getFirmwareDataList() const;
    const G330FirmwareFileHeader          &getFileHeader() const;

    static bool isFirmwareDataAdaptable(const g330_firmware_data &firmwareData, uint16_t pid);

private:
    static void checkFirmwareData(const g330_firmware_data &firmwareData);

private:
    G330FirmwareFileHeader          fileHeader_{};
    std::vector<g330_firmware_data> firmwareDataList_;
};


}  // namespace libobsensor