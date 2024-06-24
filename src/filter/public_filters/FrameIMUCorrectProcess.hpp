#pragma once
#include "FilterBase.hpp"
#include "libobsensor/h/ObTypes.h"
#include "imu_calibration_params.h"
#include "imu_calibration_params_parser.h"
#include <Eigen/Core>
#include <Eigen/LU>

namespace libobsensor {

class IMUCorrecter : public FilterBase {
public:
    static IMUCalibrateParams parserIMUCalibrationParamsRaw(uint8_t* filedata, uint32_t size);

    static float calculateAccelGravity(int16_t accelValue, uint8_t accelFSR);

    static float calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR);

    static float calculateRegisterTemperature(int16_t tempValue);

public:
    IMUCorrecter(const std::string &name);

    // Config
    virtual void               updateConfig(std::vector<std::string> &params) override;

    virtual const std::string &getConfigSchema() const override;

    void setAccelScaleRange(int scaleRange) {
        accelScaleRange_ = scaleRange;
    }

    void setGyroScaleRange(int scaleRange) {
        gyroScaleRange_ = scaleRange;
    }

    void setIMUCalibrationParam(IMUCalibrateParams param) {
        param_ = param;
    }

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    Eigen::Vector3d correctAccel(const Eigen::Vector3d& accelVec, IMUCalibrateParams *params);

    Eigen::Vector3d correctGyro(const Eigen::Vector3d& gyroVec, IMUCalibrateParams *params);

protected:
    IMUCalibrateParams param_;

    int accelScaleRange_;

    int gyroScaleRange_;

    static constexpr float GYRO_MAX =  32800 / 0.017453293f;

    static constexpr float ACCEL_MAX = 32768 / 9.80f;
};

}  // namespace libobsensor
