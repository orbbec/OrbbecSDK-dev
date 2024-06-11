#pragma once
#include "parameter/IMUParam.hpp"

namespace libobsensor {

class IMUCorrecter : public FilterBase {
public:
    IMUCorrecter();
    ~IMUCorrecter() noexcept {
        reset();
    };

    virtual void reset() override;

    void setAccelScaleRange(int scaleRange) {
        accelScaleRange_ = scaleRange;
    }

    void setGyroScaleRange(int scaleRange) {
        gyroScaleRange_ = scaleRange;
    }

    void setIMUCalibrationParam(IMUCalibrateParams param) {
        param_ = param;
    }

    void setProcessFuncEnable(bool enable) {
        processFuncEnable_ = enable;
    }

    bool getProcessFuncEnable() {
        return processFuncEnable_;
    }

    void setIMUDataUnitTransformOnly(bool transform) {
        uintTransformOnly_ = transform;
    }


private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    IMUCalibrateParams param_;
    
    int accelScaleRange_;

    int gyroScaleRange_;

    bool processFuncEnable_;

    bool uintTransformOnly_;
};

}  // namespace libobsensor
