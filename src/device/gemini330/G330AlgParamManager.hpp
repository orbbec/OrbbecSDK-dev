#pragma once

#include "IDevice.hpp"
#include "IProperty.hpp"

namespace libobsensor {


class G330AlgParamManager {
public:
    G330AlgParamManager(const std::shared_ptr<IPropertyAccessor> &propertyAccessor, uint16_t pid);
    virtual ~G330AlgParamManager() = default;

    OBCameraIntrinsic  getCameraIntrinsic(std::shared_ptr<const StreamProfile> profile) const override;
    void               registerCameraIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBCameraIntrinsic &intrinsic) override;
    OBCameraDistortion getCameraDistortion(std::shared_ptr<const StreamProfile> profile) const override;
    void               registerCameraDistortion(std::shared_ptr<const StreamProfile> profile, const OBCameraDistortion &distortion) override;
    OBAccelIntrinsic   getAccelIntrinsic(std::shared_ptr<const StreamProfile> profile) const override;
    void               registerAccelIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBAccelIntrinsic &intrinsic) override;
    OBGyroIntrinsic    getGyroIntrinsic(std::shared_ptr<const StreamProfile> profile) const override;
    void               registerGyroIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBGyroIntrinsic &intrinsic) override;

    OBExtrinsic getExtrinsic(std::shared_ptr<const StreamProfile> source, std::shared_ptr<const StreamProfile> target) const override;
    void registerExtrinsic(std::shared_ptr<const StreamProfile> source, std::shared_ptr<const StreamProfile> target, const OBExtrinsic &extrinsic) override;
    void registerSameExtrinsic(std::shared_ptr<const StreamProfile> profile, std::shared_ptr<const StreamProfile> target) override;
    void bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList);

    OBDisparityProcessParam getCurrentDisparityProcessParam();
    OBDisparityProcessParam getDisparityProcessParam(std::shared_ptr<const StreamProfile> profile) const override;
    void                    registerDisparityProcessParam(std::shared_ptr<const StreamProfile> profile, const OBDisparityProcessParam &param) override;
    bool                    isBinocularCamera() const override;

    std::vector<OBD2CProfile> getD2CProfileList() const {
        return fixedD2cProfileList_;
    }

    std::vector<OBCameraParam> getCalibrationCameraParamList() const {
        return fixedCalibrationCameraParamList_;
    }

    const IMUCalibrateParams &getIMUCalibrationParam() const {
        return imuCalibParam_;
    }

private:
    void fetchParams();
    void fixD2CParmaList();
    void registerBasicExtrinsics();

private:
    uint16_t devicePid_;

    // using empty stream profile to initialize and register extrinsic to GlobalStreamExtrinsicsManager
    std::shared_ptr<const StreamProfile> colorEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> depthEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> leftIrEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> rightIrEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> accelEmptyStreamProfile_;
    std::shared_ptr<const StreamProfile> gyroEmptyStreamProfile_;

    std::shared_ptr<IPropertyAccessor>      devCommand_;
    std::vector<OBDepthCalibrationParam> depthCalibParamList_;
    IMUCalibrateParams                   imuCalibParam_;

    std::vector<OBCameraParam> calibrationCameraParamList_;
    std::vector<OBD2CProfile>  d2cProfileList_;
    std::vector<OBCameraParam> fixedCalibrationCameraParamList_;
    std::vector<OBD2CProfile>  fixedD2cProfileList_;

    std::map<const StreamProfile *, std::pair<std::weak_ptr<const StreamProfile>, OBCameraIntrinsic>>       cameraIntrinsicMap_;
    std::map<const StreamProfile *, std::pair<std::weak_ptr<const StreamProfile>, OBCameraDistortion>>      cameraDistortionMap_;
    std::map<const StreamProfile *, std::pair<std::weak_ptr<const StreamProfile>, OBAccelIntrinsic>>        accelIntrinsicMap_;
    std::map<const StreamProfile *, std::pair<std::weak_ptr<const StreamProfile>, OBGyroIntrinsic>>         gyroIntrinsicMap_;
    std::map<const StreamProfile *, std::pair<std::weak_ptr<const StreamProfile>, OBDisparityProcessParam>> disparityProcessParamMap_;
};


}  // namespace libobsensor