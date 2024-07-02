#pragma once

#include "IDevice.hpp"
#include "IProperty.hpp"
#include "InternalTypes.hpp"
#include "libobsensor/h/ObTypes.h"
#include "imu_calibration_params.h"
#include "component/DeviceComponentBase.hpp"

#include <vector>
#include <memory>

namespace libobsensor {

class G330AlgParamManager : public DeviceComponentBase {
public:
    G330AlgParamManager(std::shared_ptr<IDevice> owner);
    virtual ~G330AlgParamManager() = default;

    void bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);
    void bindDisparityParam(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);

    OBDisparityParam getCurrentDisparityProcessParam();                                      // get from device
    OBDisparityParam getDisparityParam(std::shared_ptr<const StreamProfile> profile) const;  // get from cache

    const std::vector<OBD2CProfile> &getD2CProfileList() const {
        return fixedD2cProfileList_;
    }

    const std::vector<OBCameraParam> &getCalibrationCameraParamList() const {
        return fixedCalibrationCameraParamList_;
    }

    const IMUCalibrateParams &getIMUCalibrationParam() const {
        return imuCalibParam_;
    }

private:
    void fetchParams();
    void fixD2CParmaList();
    void registerBasicExtrinsics();
    void bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);
    void bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);

private:
    // using empty stream profile to initialize and register extrinsic to GlobalStreamExtrinsicsManager
    std::shared_ptr<const StreamProfile> colorEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> depthEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> leftIrEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> rightIrEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> accelEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> gyroEmptyStreamProfile_;

    std::vector<OBDepthCalibrationParam> depthCalibParamList_;
    IMUCalibrateParams                   imuCalibParam_;
    std::vector<OBCameraParam>           calibrationCameraParamList_;
    std::vector<OBD2CProfile>            d2cProfileList_;
    std::vector<OBCameraParam>           fixedCalibrationCameraParamList_;
    std::vector<OBD2CProfile>            fixedD2cProfileList_;
};

}  // namespace libobsensor