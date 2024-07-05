#pragma once
#include "FilterBase.hpp"
#include "libobsensor/h/ObTypes.h"
#include "IProperty.hpp"
#include "InternalTypes.hpp"

namespace libobsensor {

class IMUCorrecter : public FilterBase, public IPropertyPort {
public:
    static OBIMUCalibrateParams parserIMUCalibrationParamsRaw(uint8_t *filedata, uint32_t size);
    static float                calculateAccelGravity(int16_t accelValue, uint8_t accelFSR);
    static float                calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR);
    static float calculateRegisterTemperature(int16_t tempValue);

public:
    IMUCorrecter(const std::string &name);

    virtual ~IMUCorrecter() = default;

    // Config
    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    OBPoint3f correctAccel(const OBPoint3f &accelVec, OBAccelIntrinsic *intrinsic);
    OBPoint3f correctGyro(const OBPoint3f &gyroVec, OBGyroIntrinsic *intrinsic);

protected:
    static constexpr float GYRO_MAX  = 32800 / 0.017453293f;
    static constexpr float ACCEL_MAX = 32768 / 9.80f;
};

}  // namespace libobsensor
