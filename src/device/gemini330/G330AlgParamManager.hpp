#pragma once

#include "IDevice.hpp"
#include "IProperty.hpp"
#include "InternalTypes.hpp"
#include "libobsensor/h/ObTypes.h"
#include "DeviceComponentBase.hpp"

#include <vector>
#include <memory>

namespace libobsensor {

class G330AlgParamManager : public DeviceComponentBase {
public:
    G330AlgParamManager(IDevice *owner);
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

    const OBIMUCalibrateParams &getIMUCalibrationParam() const {
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
    std::shared_ptr<const StreamProfile> colorBasicStreamProfile_;
    std::shared_ptr<const StreamProfile> depthBasicStreamProfile_;
    std::shared_ptr<const StreamProfile> leftIrBasicStreamProfile_;
    std::shared_ptr<const StreamProfile> rightIrBasicStreamProfile_;
    std::shared_ptr<const StreamProfile> accelBasicStreamProfile_;
    std::shared_ptr<const StreamProfile> gyroBasicStreamProfile_;

    std::vector<OBDepthCalibrationParam> depthCalibParamList_;
    OBIMUCalibrateParams                 imuCalibParam_;
    std::vector<OBCameraParam>           calibrationCameraParamList_;
    std::vector<OBD2CProfile>            d2cProfileList_;
    std::vector<OBCameraParam>           fixedCalibrationCameraParamList_;
    std::vector<OBD2CProfile>            fixedD2cProfileList_;
};

}  // namespace libobsensor