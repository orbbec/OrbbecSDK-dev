#pragma once
#include "IDevice.hpp"
#include "IProperty.hpp"
#include "InternalTypes.hpp"
#include "libobsensor/h/ObTypes.h"
#include "component/DeviceComponentBase.hpp"

#include <vector>
#include <memory>

namespace libobsensor {
class FemtoBoltAlgParamManager : public DeviceComponentBase {
public:
    FemtoBoltAlgParamManager(IDevice *owner);
    virtual ~FemtoBoltAlgParamManager() noexcept = default;

    void bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);

    const std::vector<OBD2CProfile> &getD2CProfileList() const {
        return d2cProfileList_;
    }

    const OBIMUCalibrateParams &getIMUCalibrateParams() const {
        return imuCalibParam_;
    }

private:
    void fetchParams();
    void registerBasicExtrinsics();
    void bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);
    void bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);

private:
    OBIMUCalibrateParams                 imuCalibParam_;
    std::vector<OBCameraParam>           calibrationCameraParamList_;
    std::vector<OBD2CProfile>            d2cProfileList_;
    std::shared_ptr<const StreamProfile> colorEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> depthEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> accelEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> gyroEmptyStreamProfile_;
};
}  // namespace libobsensor