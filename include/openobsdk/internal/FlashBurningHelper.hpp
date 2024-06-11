#pragma once
#include <libobsensor/ObSensor.hpp>

#include <vector>
#include <string>
#include <fstream>

namespace ob {

#define G2R_CALIB_PARAM_RECTIFY_KK_OFFSET 0xB0000
#define G2R_CALIB_PARAM_RECTIFY_LUT_OFFSET 0xBF000
#define G2R_CALIB_PARAM_IMU_OFFSET 0xBD000
#define G2R_SN_OFFSET 0xA1000

class FlashBurningHelper {
public:
    // usage: FlashBurningHelper helper(device);
    FlashBurningHelper(std::shared_ptr<Device> device) : device_(device) {}
    virtual ~FlashBurningHelper() noexcept = default;

    // usage: burnSerialNumber(G2R_SN_OFFSET, "1234567890", "1234567890");
    virtual bool burnSerialNumber(const std::string &deviceSerialNumber, const std::string &asicSerialNumber = "") {
        std::vector<char> buffer;
        if(deviceSerialNumber.size() > 16 || asicSerialNumber.size() > 16) {
            throw std::runtime_error("Serial number is too long");
        }
        buffer.resize(32);
        memset(buffer.data(), 0, 32);
        memcpy(buffer.data(), deviceSerialNumber.c_str(), deviceSerialNumber.size());
        memcpy(buffer.data() + 16, asicSerialNumber.c_str(), asicSerialNumber.size());
        bool result = false;
        device_->writeFlash(
            G2R_SN_OFFSET, buffer.data(), buffer.size(),
            [&](OBDataTranState state, uint8_t percent) {
                if(state == OBDataTranState::DATA_TRAN_STAT_DONE) {
                    result = true;
                }
            },
            false);
        return result;
    }

    // usage: burnCalibParam(G2R_CALIB_PARAM_RECTIFY_KK_OFFSET, "./calibParamFile1.bin", callback);
    virtual bool burnCalibParam(uint32_t offset, std::string &calibParamFile, SetDataCallback callback = nullptr) {
        // read file
        std::ifstream file(calibParamFile, std::ios::binary);
        if(!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + calibParamFile);
        }
        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<char> buffer(fileSize);
        file.read(buffer.data(), fileSize);
        file.close();

        bool result = false;
        device_->writeFlash(
            offset, buffer.data(), buffer.size(),
            [&](OBDataTranState state, uint8_t percent) {
                if(state == OBDataTranState::DATA_TRAN_STAT_DONE) {
                    result = true;
                }
                if(callback) {
                    callback(state, percent);
                }
            },
            false);

        return result;
    }

private:
    std::shared_ptr<Device> device_;
};
}  // namespace ob