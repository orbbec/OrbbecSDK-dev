#include "G330AlgParamManager.hpp"
#include "core/streamprofile/StreamProfile.hpp"
#include "common/exception/ObException.hpp"
#include "common/parameter/Mx6600CalibParamParser.hpp"
#include "common/parameter/IMUParam.hpp"
#include "common/logger/Logger.hpp"
#include "core/device/component/GlobalStreamExtrinsicsManager.hpp"

#include <vector>
#include <sstream>
namespace libobsensor {
namespace g330 {

bool findBestMatchedCameraParam(const std::vector<OBCameraParam> &cameraParamList, const std::shared_ptr<const VideoStreamProfile> &profile,
                                OBCameraParam &result) {
    bool found = false;
    // match same resolution
    for(auto &param: cameraParamList) {
        auto streamType = profile->getType();
        if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
           && param.depthIntrinsic.width == profile->getWidth() && param.depthIntrinsic.height == profile->getHeight()) {
            found  = true;
            result = param;
            break;
        }
        else if(streamType == OB_STREAM_COLOR && param.rgbIntrinsic.width == profile->getWidth() && param.rgbIntrinsic.height == profile->getHeight()) {
            found  = true;
            result = param;
            break;
        }
    }

    if(!found) {
        // match same ratio
        float ratio = (float)profile->getWidth() / profile->getHeight();
        for(auto &param: cameraParamList) {
            auto streamType = profile->getType();
            if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
               && (float)param.depthIntrinsic.width / param.depthIntrinsic.height == ratio) {
                found  = true;
                result = param;
                break;
            }
            else if(streamType == OB_STREAM_COLOR && (float)param.rgbIntrinsic.width / param.rgbIntrinsic.height == ratio) {
                found  = true;
                result = param;
                break;
            }
        }
    }

    return found;
}

G330AlgParamManager::G330AlgParamManager(const std::shared_ptr<IDeviceCommand> &cmd, uint16_t pid) : devCommand_(cmd), devicePid_(pid) {
    if(!devCommand_) {
        throw libobsensor::not_implemented_exception("Can not get command from device!");
    }

    fetchParams();
    fixD2CParmaList();
    registerBasicExtrinsics();
}

void G330AlgParamManager::fetchParams() {
    // d2c param
    std::vector<uint8_t> data;
    uint32_t             size            = 0;
    OBCmdVersion         localCmdVersion = OB_CMD_VERSION_INVALID;

    // depth calib param
    BEGIN_TRY_EXECUTE(devCommand_->getStructDataList(
        OB_RAW_DATA_DEPTH_CALIB_PARAM,
        [&](OBDataTranState state, OBCmdVersion cmdVersion, OBDataChunk *dataChunk) {
            if(state == DATA_TRAN_STAT_TRANSFERRING) {
                data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                localCmdVersion = cmdVersion;
            }
        },
        false);)
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get depth calibration params failed!");
        data.clear();
    })

    if(!data.empty() && localCmdVersion != OB_CMD_VERSION_INVALID) {
        depthCalibParamList_ = depthCalibParamParse(localCmdVersion, data.data(), data.size());
        LOG_DEBUG("Get depth calibration params success! num={}", depthCalibParamList_.size());
        for(auto &&param: depthCalibParamList_) {
            std::stringstream ss;
            ss << param;
            LOG_DEBUG(" - {}", ss.str());
        }
    }

    // align param
    data.clear();
    BEGIN_TRY_EXECUTE(devCommand_->getStructDataList(
        OB_RAW_DATA_ALIGN_CALIB_PARAM,
        [&](OBDataTranState state, OBCmdVersion cmdVersion, OBDataChunk *dataChunk) {
            if(state == DATA_TRAN_STAT_TRANSFERRING) {
                data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                localCmdVersion = cmdVersion;
            }
        },
        false))
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get align calibration camera params failed!");
        data.clear();
    })
    if(!data.empty() && localCmdVersion != OB_CMD_VERSION_INVALID) {
        calibrationCameraParamList_ = alignCalibParamParse(localCmdVersion, data.data(), data.size());
        LOG_DEBUG("Get align calibration camera params success! num={}", calibrationCameraParamList_.size());
        for(auto &&cameraParam: calibrationCameraParamList_) {
            std::stringstream ss;
            ss << cameraParam;
            LOG_DEBUG("- {}", ss.str());
        }
    }

    // d2c align profile
    data.clear();
    BEGIN_TRY_EXECUTE(devCommand_->getStructDataList(
        OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST,
        [&](OBDataTranState state, OBCmdVersion cmdVersion, OBDataChunk *dataChunk) {
            if(state == DATA_TRAN_STAT_TRANSFERRING) {
                data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                localCmdVersion = cmdVersion;
            }
        },
        false);)
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get depth to color profile list failed!");
        data.clear();
    })
    if(!data.empty() && localCmdVersion != OB_CMD_VERSION_INVALID) {
        d2cProfileList_ = d2cProfileInfoParse(data.data(), data.size());
        LOG_DEBUG("Get depth to color profile list success! num={}", d2cProfileList_.size());
        for(auto &&profile: d2cProfileList_) {
            std::stringstream ss;
            ss << profile;
            LOG_DEBUG(" - {}", ss.str());
        }
    }

    // imu param
    data.clear();
    BEGIN_TRY_EXECUTE(devCommand_->getRawData(
        OB_RAW_DATA_IMU_CALIB_PARAM,
        [&](OBDataTranState state, OBDataChunk *dataChunk) {
            if(state == DATA_TRAN_STAT_TRANSFERRING) {
                data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
            }
        },
        false))
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get imu calibration params failed!");
        data.clear();
    })
    if(!data.empty()) {
        imuCalibParam_ = parserIMUCalibrationParamsRaw(data.data(), data.size());
        LOG_DEBUG("Get imu calibration params success!");
    }
}

void G330AlgParamManager::registerBasicExtrinsics() {
    auto extrinsicMgr          = GlobalStreamExtrinsicsManager::getInstance();
    depthEmptyStreamProfile_   = std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_DEPTH, OB_FORMAT_UNKNOWN, 0, 0, 0);
    colorEmptyStreamProfile_   = std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_COLOR, OB_FORMAT_UNKNOWN, 0, 0, 0);
    leftIrEmptyStreamProfile_  = std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_IR_LEFT, OB_FORMAT_UNKNOWN, 0, 0, 0);
    rightIrEmptyStreamProfile_ = std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_IR_RIGHT, OB_FORMAT_UNKNOWN, 0, 0, 0);
    accelEmptyStreamProfile_ =
        std::make_shared<AccelStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_ACCEL, OB_FORMAT_UNKNOWN, OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    gyroEmptyStreamProfile_ =
        std::make_shared<GyroStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_GYRO, OB_FORMAT_UNKNOWN, OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);

    if(!calibrationCameraParamList_.empty()) {
        auto d2cExtrinsic = calibrationCameraParamList_.front().transform;
        extrinsicMgr->registerExtrinsics(depthEmptyStreamProfile_, colorEmptyStreamProfile_, d2cExtrinsic);
    }
    extrinsicMgr->registerSameExtrinsics(leftIrEmptyStreamProfile_, depthEmptyStreamProfile_);

    auto left_to_right     = IdentityExtrinsic;
    left_to_right.trans[0] = depthCalibParamList_.front().baseline * depthCalibParamList_.front().unit;
    extrinsicMgr->registerExtrinsics(leftIrEmptyStreamProfile_, rightIrEmptyStreamProfile_, left_to_right);

    //    auto       &imuExtr = imuCalibParam_.singleIMUParams[0].TCamToIMU;  // error on raspberry pi, replace with below code
    double imuExtr[16] = { 0 };
    memcpy(imuExtr, imuCalibParam_.singleIMUParams[0].TCamToIMU, sizeof(imuExtr));

    OBExtrinsic imu_to_depth;
    imu_to_depth.rot[0] = (float)imuExtr[0];
    imu_to_depth.rot[1] = (float)imuExtr[1];

    imu_to_depth.rot[2]   = (float)imuExtr[2];
    imu_to_depth.rot[3]   = (float)imuExtr[4];
    imu_to_depth.rot[4]   = (float)imuExtr[5];
    imu_to_depth.rot[5]   = (float)imuExtr[6];
    imu_to_depth.rot[6]   = (float)imuExtr[8];
    imu_to_depth.rot[7]   = (float)imuExtr[9];
    imu_to_depth.rot[8]   = (float)imuExtr[10];
    imu_to_depth.trans[0] = (float)imuExtr[3];
    imu_to_depth.trans[1] = (float)imuExtr[7];
    imu_to_depth.trans[2] = (float)imuExtr[11];
    extrinsicMgr->registerExtrinsics(accelEmptyStreamProfile_, depthEmptyStreamProfile_, imu_to_depth);
    extrinsicMgr->registerSameExtrinsics(gyroEmptyStreamProfile_, accelEmptyStreamProfile_);
}

typedef struct {
    uint32_t width;
    uint32_t height;
} Resolution;

void G330AlgParamManager::fixD2CParmaList() {
    std::vector<Resolution> appendColorResolutions = { { 1920, 1080 }, { 1280, 800 }, { 1280, 720 }, { 960, 540 }, { 848, 480 }, { 640, 480 },
                                                       { 640, 400 },   { 640, 360 },  { 424, 270 },  { 424, 240 }, { 320, 240 }, { 320, 180 } };

    const std::vector<Resolution> depthResolutions = { { 1280, 800 }, { 1280, 720 }, { 848, 480 }, { 640, 480 }, { 640, 400 },
                                                       { 640, 360 },  { 480, 270 },  { 424, 240 }, { 848, 100 } };

    auto iter = std::find(gG330LPids.begin(), gG330LPids.end(), devicePid_);

    if(iter != gG330LPids.end()) {
        appendColorResolutions.push_back({ 480, 270 });
    }

    if(d2cProfileList_.empty()) {
        return;
    }

    fixedD2cProfileList_ = d2cProfileList_;
    for(auto &profile: fixedD2cProfileList_) {
        profile.alignType = ALIGN_D2C_HW;
    }

    fixedCalibrationCameraParamList_ = calibrationCameraParamList_;
    for(const auto &colorRes: appendColorResolutions) {
        std::shared_ptr<VideoStreamProfile> colorProfile =
            std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_COLOR, OB_FORMAT_UNKNOWN, colorRes.width, colorRes.height, 30);
        OBCameraParam colorAlignParam;
        if(!findBestMatchedCameraParam(calibrationCameraParamList_, colorProfile, colorAlignParam)) {
            continue;
        }
        OBCameraIntrinsic colorIntrinsic = colorAlignParam.rgbIntrinsic;
        float             ratio          = (float)colorProfile->getWidth() / colorIntrinsic.width;
        colorIntrinsic.fx *= ratio;
        colorIntrinsic.fy *= ratio;
        colorIntrinsic.cx *= ratio;
        colorIntrinsic.cy *= ratio;
        colorIntrinsic.width  = colorProfile->getWidth();
        colorIntrinsic.height = (float)colorIntrinsic.height * ratio;
        for(const auto &depthRes: depthResolutions) {
            std::shared_ptr<VideoStreamProfile> depthProfile =
                std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_DEPTH, OB_FORMAT_UNKNOWN, depthRes.width, depthRes.height, 30);
            OBCameraParam depthAlignParam;
            if(!findBestMatchedCameraParam(calibrationCameraParamList_, depthProfile, depthAlignParam)) {
                continue;
            }
            OBCameraIntrinsic depthIntrinsic = depthAlignParam.depthIntrinsic;
            ratio                            = (float)depthProfile->getWidth() / depthIntrinsic.width;
            depthIntrinsic.fx *= ratio;
            depthIntrinsic.fy *= ratio;
            depthIntrinsic.cx *= ratio;
            depthIntrinsic.cy *= ratio;
            depthIntrinsic.width  = depthProfile->getWidth();
            depthIntrinsic.height = (float)depthIntrinsic.height * ratio;

            auto index = fixedCalibrationCameraParamList_.size();
            fixedCalibrationCameraParamList_.push_back({ depthIntrinsic, colorIntrinsic, depthAlignParam.depthDistortion, depthAlignParam.rgbDistortion,
                                                         depthAlignParam.transform, depthAlignParam.isMirrored });

            OBD2CProfile d2cProfile;
            d2cProfile.alignType        = ALIGN_D2C_SW;
            d2cProfile.postProcessParam = { 1.0f, 0, 0, 0, 0 };
            d2cProfile.colorWidth       = colorProfile->getWidth();
            d2cProfile.colorHeight      = colorProfile->getHeight();
            d2cProfile.depthWidth       = depthProfile->getWidth();
            d2cProfile.depthHeight      = depthProfile->getHeight();
            d2cProfile.paramIndex       = (uint8_t)index;
            fixedD2cProfileList_.push_back(d2cProfile);
        }
    }

    // fix depth intrinsic according to post process param and rgb intrinsic
    for(auto &profile: fixedD2cProfileList_) {
        if(profile.alignType != ALIGN_D2C_HW || profile.paramIndex >= fixedCalibrationCameraParamList_.size()) {
            continue;
        }
        const auto &cameraParam = fixedCalibrationCameraParamList_.at(profile.paramIndex);

        std::shared_ptr<VideoStreamProfile> colorProfile =
            std::make_shared<VideoStreamProfile>(std::weak_ptr<ISensor>(), OB_STREAM_COLOR, OB_FORMAT_UNKNOWN, profile.colorWidth, profile.colorHeight, 30);

        OBCameraParam fixedCameraParam = cameraParam;
        OBCameraParam matchedCameraParam;
        if(!findBestMatchedCameraParam(calibrationCameraParamList_, colorProfile, matchedCameraParam)) {
            continue;
        }
        fixedCameraParam.rgbIntrinsic = matchedCameraParam.rgbIntrinsic;

        // scale the rgb(target) intrinsic to match the profile
        auto ratio = (float)profile.colorWidth / fixedCameraParam.rgbIntrinsic.width;
        fixedCameraParam.rgbIntrinsic.fx *= ratio;
        fixedCameraParam.rgbIntrinsic.fy *= ratio;
        fixedCameraParam.rgbIntrinsic.cx *= ratio;
        fixedCameraParam.rgbIntrinsic.cy *= ratio;
        fixedCameraParam.rgbIntrinsic.width  = (float)fixedCameraParam.rgbIntrinsic.width * ratio;
        fixedCameraParam.rgbIntrinsic.height = (float)fixedCameraParam.rgbIntrinsic.height * ratio;

        // Scale the rgb(target) intrinsic parameters by the same factor as the depth intrinsic scale.
        auto depthRatio = (float)fixedCameraParam.depthIntrinsic.width / profile.depthWidth;
        fixedCameraParam.rgbIntrinsic.fx *= depthRatio;
        fixedCameraParam.rgbIntrinsic.fy *= depthRatio;
        fixedCameraParam.rgbIntrinsic.cx *= depthRatio;
        fixedCameraParam.rgbIntrinsic.cy *= depthRatio;
        fixedCameraParam.rgbIntrinsic.width  = (float)fixedCameraParam.rgbIntrinsic.width * depthRatio;
        fixedCameraParam.rgbIntrinsic.height = (float)fixedCameraParam.rgbIntrinsic.height * depthRatio;

        // according to post process param to reverse process the rgb(target) intrinsic
        fixedCameraParam.rgbIntrinsic.fx     = fixedCameraParam.rgbIntrinsic.fx / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.fy     = fixedCameraParam.rgbIntrinsic.fy / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.cx     = (fixedCameraParam.rgbIntrinsic.cx - profile.postProcessParam.alignLeft) / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.cy     = (fixedCameraParam.rgbIntrinsic.cy - profile.postProcessParam.alignTop) / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.width  = profile.depthWidth;
        fixedCameraParam.rgbIntrinsic.height = profile.depthHeight;

        auto index = fixedCalibrationCameraParamList_.size();
        fixedCalibrationCameraParamList_.push_back(fixedCameraParam);
        auto oldProfile    = profile;
        profile.paramIndex = (uint8_t)index;

        std::stringstream ss;
        ss << "Fix align calibration camera params:" << std::endl;
        ss << oldProfile << std::endl;
        ss << cameraParam.rgbIntrinsic << std::endl;
        ss << "to:" << std::endl;
        ss << profile << std::endl;
        ss << fixedCameraParam.rgbIntrinsic << std::endl;
        LOG_INFO("- {}", ss.str());
    }

    // LOG_DEBUG("Fixed align calibration camera params success! num={}", fixedCalibrationCameraParamList_.size());
    // for(auto &&profile: fixedD2cProfileList_) {
    //     if(profile.paramIndex >= fixedCalibrationCameraParamList_.size()) {
    //         continue;
    //     }

    //     std::stringstream ss;
    //     ss << profile;
    //     ss << std::endl;
    //     ss << fixedCalibrationCameraParamList_[profile.paramIndex];
    //     LOG_DEBUG("- {}", ss.str());
    // }
}

void G330AlgParamManager::registerCameraIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBCameraIntrinsic &intrinsic) {
    // clear expired items
    for(auto iter = cameraIntrinsicMap_.begin(); iter != cameraIntrinsicMap_.end();) {
        if(iter->second.first.expired()) {
            iter = cameraIntrinsicMap_.erase(iter);
        }
        else {
            ++iter;
        }
    }
    cameraIntrinsicMap_[profile.get()] = { std::weak_ptr<const StreamProfile>(profile), intrinsic };
}

OBCameraIntrinsic G330AlgParamManager::getCameraIntrinsic(std::shared_ptr<const StreamProfile> profile) const {
    auto it = cameraIntrinsicMap_.find(profile.get());
    if(it != cameraIntrinsicMap_.end()) {
        return it->second.second;
    }

    OBCameraIntrinsic intrinsic;
    if(!profile || !profile->is<VideoStreamProfile>()) {
        throw libobsensor::invalid_value_exception("Invalid profile!");
    }
    OBCameraParam param;
    auto          vsp = profile->as<VideoStreamProfile>();
    if(!findBestMatchedCameraParam(calibrationCameraParamList_, vsp, param)) {
        throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
    }
    switch(profile->getType()) {
    case OB_STREAM_COLOR:
        intrinsic = param.rgbIntrinsic;
        break;
    case OB_STREAM_DEPTH:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
        intrinsic = param.depthIntrinsic;
        break;
    default:
        break;
    }
    auto ratio = (float)vsp->getWidth() / (float)intrinsic.width;
    intrinsic.fx *= ratio;
    intrinsic.fy *= ratio;
    intrinsic.cx *= ratio;
    intrinsic.cy *= ratio;
    intrinsic.width = vsp->getWidth();
    intrinsic.height *= ratio;

    return intrinsic;
}

OBCameraDistortion G330AlgParamManager::getCameraDistortion(std::shared_ptr<const StreamProfile> profile) const {
    auto it = cameraDistortionMap_.find(profile.get());
    if(it != cameraDistortionMap_.end()) {
        return it->second.second;
    }

    if(!profile || !profile->is<VideoStreamProfile>()) {
        throw libobsensor::invalid_value_exception("Invalid profile!");
    }
    OBCameraParam param;
    if(!findBestMatchedCameraParam(calibrationCameraParamList_, profile->as<VideoStreamProfile>(), param)) {
        throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
    }
    switch(profile->getType()) {
    case OB_STREAM_COLOR:
        return param.rgbDistortion;
    case OB_STREAM_DEPTH:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
        return param.depthDistortion;
    default:
        break;
    }

    return {};
}

void G330AlgParamManager::registerCameraDistortion(std::shared_ptr<const StreamProfile> profile, const OBCameraDistortion &distortion) {
    // clear expired items
    for(auto iter = cameraDistortionMap_.begin(); iter != cameraDistortionMap_.end();) {
        if(iter->second.first.expired()) {
            iter = cameraDistortionMap_.erase(iter);
        }
        else {
            ++iter;
        }
    }

    cameraDistortionMap_[profile.get()] = { std::weak_ptr<const StreamProfile>(profile), distortion };
}

OBAccelIntrinsic G330AlgParamManager::getAccelIntrinsic(std::shared_ptr<const StreamProfile> profile) const {
    auto iter = accelIntrinsicMap_.find(profile.get());
    if(iter != accelIntrinsicMap_.end()) {
        return iter->second.second;
    }

    OBAccelIntrinsic intrinsic;
    memset(&intrinsic, 0, sizeof(intrinsic));
    if(!profile->is<AccelStreamProfile>()) {
        throw libobsensor::invalid_value_exception("Invalid profile!");
    }
    return *(OBAccelIntrinsic *)&(imuCalibParam_.singleIMUParams[0].acc);
}

void G330AlgParamManager::registerAccelIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBAccelIntrinsic &intrinsic) {
    // clear expired items
    for(auto iter = accelIntrinsicMap_.begin(); iter != accelIntrinsicMap_.end();) {
        if(iter->second.first.expired()) {
            iter = accelIntrinsicMap_.erase(iter);
        }
        else {
            ++iter;
        }
    }

    accelIntrinsicMap_[profile.get()] = { std::weak_ptr<const StreamProfile>(profile), intrinsic };
}

OBGyroIntrinsic G330AlgParamManager::getGyroIntrinsic(std::shared_ptr<const StreamProfile> profile) const {
    auto iter = gyroIntrinsicMap_.find(profile.get());
    if(iter != gyroIntrinsicMap_.end()) {
        return iter->second.second;
    }

    OBAccelIntrinsic intrinsic;
    memset(&intrinsic, 0, sizeof(intrinsic));
    if(!profile->is<GyroStreamProfile>()) {
        throw libobsensor::invalid_value_exception("Invalid profile!");
    }
    return *(OBGyroIntrinsic *)&(imuCalibParam_.singleIMUParams[0].gyro);
}

void G330AlgParamManager::registerGyroIntrinsic(std::shared_ptr<const StreamProfile> profile, const OBGyroIntrinsic &intrinsic) {
    // clear expired items
    for(auto iter = gyroIntrinsicMap_.begin(); iter != gyroIntrinsicMap_.end();) {
        if(iter->second.first.expired()) {
            iter = gyroIntrinsicMap_.erase(iter);
        }
        else {
            ++iter;
        }
    }

    gyroIntrinsicMap_[profile.get()] = { std::weak_ptr<const StreamProfile>(profile), intrinsic };
}

OBExtrinsic G330AlgParamManager::getExtrinsic(std::shared_ptr<const StreamProfile> source, std::shared_ptr<const StreamProfile> target) const {
    return GlobalStreamExtrinsicsManager::getInstance()->getExtrinsics(source, target);
}

void G330AlgParamManager::registerExtrinsic(std::shared_ptr<const StreamProfile> source, std::shared_ptr<const StreamProfile> target,
                                           const OBExtrinsic &extrinsic) {
    GlobalStreamExtrinsicsManager::getInstance()->registerExtrinsics(source, target, extrinsic);
}

void G330AlgParamManager::registerSameExtrinsic(std::shared_ptr<const StreamProfile> source, std::shared_ptr<const StreamProfile> target) {
    GlobalStreamExtrinsicsManager::getInstance()->registerSameExtrinsics(source, target);
}

void G330AlgParamManager::bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto extrinsicMgr            = GlobalStreamExtrinsicsManager::getInstance();
    auto matchEmptyStreamProfile = [&](std::shared_ptr<const StreamProfile> profile) {
        auto spType = profile->getType();
        switch(spType) {
        case OB_STREAM_DEPTH:
            return depthEmptyStreamProfile_;
        case OB_STREAM_COLOR:
            return colorEmptyStreamProfile_;
        case OB_STREAM_IR_LEFT:
            return leftIrEmptyStreamProfile_;
        case OB_STREAM_IR_RIGHT:
            return rightIrEmptyStreamProfile_;
        case OB_STREAM_ACCEL:
            return accelEmptyStreamProfile_;
        case OB_STREAM_GYRO:
            return gyroEmptyStreamProfile_;
        default:
            return std::shared_ptr<const StreamProfile>(nullptr);
        }
    };
    for(auto &sp: streamProfileList) {
        auto src = matchEmptyStreamProfile(sp);
        extrinsicMgr->registerSameExtrinsics(sp, src);
    }
}

OBDisparityProcessParam G330AlgParamManager::getCurrentDisparityProcessParam() {
    std::vector<uint8_t> data;
    OBCmdVersion         localCmdVersion = OB_CMD_VERSION_INVALID;
    // depth calib param
    BEGIN_TRY_EXECUTE(devCommand_->getStructDataList(
        OB_RAW_DATA_DEPTH_CALIB_PARAM,
        [&](OBDataTranState state, OBCmdVersion cmdVersion, OBDataChunk *dataChunk) {
            if(state == DATA_TRAN_STAT_TRANSFERRING) {
                data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                localCmdVersion = cmdVersion;
            }
        },
        false);)
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get depth calibration params failed!");
        data.clear();
    })

    if(!data.empty() && localCmdVersion != OB_CMD_VERSION_INVALID) {
        depthCalibParamList_ = depthCalibParamParse(localCmdVersion, data.data(), data.size());
        LOG_DEBUG("Get depth calibration params success! num={}", depthCalibParamList_.size());
        for(auto &&param: depthCalibParamList_) {
            std::stringstream ss;
            ss << param;
            LOG_DEBUG(" - {}", ss.str());
        }
    }

    OBDisparityProcessParam param      = { 0 };
    const auto             &depthCalib = depthCalibParamList_.front();
    param.baseline                     = depthCalib.baseline;
    param.zpd                          = depthCalib.z0;
    param.fx                           = depthCalib.focalPix;
    param.zpps                         = depthCalib.z0 / depthCalib.focalPix;
    param.bitSize                      = 14;  // low 14 bit
    param.dispIntPlace                 = 8;
    param.unit                         = depthCalib.unit;
    param.dispOffset                   = depthCalib.dispOffset;
    param.invalidDisp                  = depthCalib.invalidDisp;
    param.packMode                     = OB_DISP_PACK_ORIGINAL_NEW;
    return param;
}

OBDisparityProcessParam G330AlgParamManager::getDisparityProcessParam(std::shared_ptr<const StreamProfile> profile) const {
    auto it = disparityProcessParamMap_.find(profile.get());
    if(it != disparityProcessParamMap_.end()) {
        return it->second.second;
    }

    // LOG_DEBUG("Can not find disparity process param for profile, read current param from device instead.");

    OBDisparityProcessParam param      = { 0 };
    const auto             &depthCalib = depthCalibParamList_.front();
    param.baseline                     = depthCalib.baseline;
    param.zpd                          = depthCalib.z0;
    param.fx                           = depthCalib.focalPix;
    param.zpps                         = depthCalib.z0 / depthCalib.focalPix;
    param.bitSize                      = 14;  // low 14 bit
    param.dispIntPlace                 = 8;
    param.unit                         = depthCalib.unit;
    param.dispOffset                   = depthCalib.dispOffset;
    param.invalidDisp                  = depthCalib.invalidDisp;
    param.packMode                     = OB_DISP_PACK_ORIGINAL_NEW;

    return param;
}

void G330AlgParamManager::registerDisparityProcessParam(std::shared_ptr<const StreamProfile> profile, const OBDisparityProcessParam &param) {
    // clear expired items
    for(auto iter = disparityProcessParamMap_.begin(); iter != disparityProcessParamMap_.end();) {
        if(iter->second.first.expired()) {
            iter = disparityProcessParamMap_.erase(iter);
        }
        else {
            ++iter;
        }
    }

    disparityProcessParamMap_[profile.get()] = { std::weak_ptr<const StreamProfile>(profile), param };
}

bool G330AlgParamManager::isBinocularCamera() const {
    const auto &depthCalib = depthCalibParamList_.front();
    return depthCalib.depthMode == (uint32_t)DEPTH_MODE_STEREO;
}

}  // namespace g330
}  // namespace libobsensor