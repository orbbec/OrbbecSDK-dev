#pragma once
#include "FilterBase.hpp"
#include "libobsensor/h/ObTypes.h"
#include "imu_calibration_params.h"
#include "imu_calibration_params_parser.h"
#include "IProperty.hpp"
#include <Eigen/Core>
#include <Eigen/LU>

namespace libobsensor {

class IMUCorrecter : public FilterBase,public IPropertyPort {
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

    void setIMUCalibrationParam(IMUCalibrateParams param) {
        param_ = param;
    }

    void setPropertyValue(uint32_t propertyId, OBPropertyValue value) override{
        if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL){
            throw invalid_value_exception("Not support this property");
        }
        FilterBase::enable(static_cast<bool>(value.intValue));
    }

    void getPropertyValue(uint32_t propertyId, OBPropertyValue *value) override{
        if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL){
            throw invalid_value_exception("Not support this property");
        }
        value->intValue = static_cast<int32_t>(isEnabled());
    }

    void getPropertyRange(uint32_t propertyId, OBPropertyRange *range) override{
        if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL){
            throw invalid_value_exception("Not support this property");
        }
        range->cur.intValue = static_cast<int32_t>(isEnabled());
        range->def.intValue = 1;
        range->max.intValue = 1;
        range->min.intValue = 0;
        range->step.intValue = 1;
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
