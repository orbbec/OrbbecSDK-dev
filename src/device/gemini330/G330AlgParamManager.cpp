#include "G330AlgParamManager.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "property/InternalProperty.hpp"
#include "DevicePids.hpp"
#include "exception/ObException.hpp"
#include "publicfilters/IMUCorrector.hpp"

#include <vector>
#include <sstream>
namespace libobsensor {

bool findBestMatchedCameraParam(const std::vector<OBCameraParam> &cameraParamList, const std::shared_ptr<const VideoStreamProfile> &profile,
                                OBCameraParam &result) {
    bool found = false;
    // match same resolution
    for(auto &param: cameraParamList) {
        auto streamType = profile->getType();
        if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
           && static_cast<uint32_t>(param.depthIntrinsic.width) == profile->getWidth()
           && static_cast<uint32_t>(param.depthIntrinsic.height) == profile->getHeight()) {
            found  = true;
            result = param;
            break;
        }
        else if(streamType == OB_STREAM_COLOR && static_cast<uint32_t>(param.rgbIntrinsic.width) == profile->getWidth()
                && static_cast<uint32_t>(param.rgbIntrinsic.height) == profile->getHeight()) {
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

G330AlgParamManager::G330AlgParamManager(IDevice *owner) : DeviceComponentBase(owner) {
    fetchParams();
    fixD2CParmaList();
    registerBasicExtrinsics();
}

void G330AlgParamManager::fetchParams() {

    try {
        auto owner           = getOwner();
        auto propServer      = owner->getPropertyServer();
        depthCalibParamList_ = propServer->getStructureDataListProtoV1_1_T<OBDepthCalibrationParam, 1>(OB_RAW_DATA_DEPTH_CALIB_PARAM);
    }
    catch(const std::exception &e) {
        LOG_ERROR("Get depth calibration params failed! {}", e.what());
    }

    try {
        auto owner           = getOwner();
        auto propServer      = owner->getPropertyServer();
        auto cameraParamList = propServer->getStructureDataListProtoV1_1_T<OBCameraParam_Internal_V0, 0>(OB_RAW_DATA_ALIGN_CALIB_PARAM);
        for(auto &cameraParam: cameraParamList) {
            OBCameraParam param;
            param.depthIntrinsic = cameraParam.depthIntrinsic;
            param.rgbIntrinsic   = cameraParam.rgbIntrinsic;
            memcpy(&param.depthDistortion, &cameraParam.depthDistortion, sizeof(param.depthDistortion));
            param.depthDistortion.model = OB_DISTORTION_BROWN_CONRADY;
            memcpy(&param.rgbDistortion, &cameraParam.rgbDistortion, sizeof(param.rgbDistortion));
            param.rgbDistortion.model = OB_DISTORTION_BROWN_CONRADY;
            param.transform           = cameraParam.transform;
            param.isMirrored          = false;
            calibrationCameraParamList_.emplace_back(param);

            std::stringstream ss;
            ss << param;
            LOG_DEBUG("-{}", ss.str());
        }
    }
    catch(const std::exception &e) {
        LOG_ERROR("Get depth calibration params failed! {}", e.what());
    }

    try {
        auto owner        = getOwner();
        auto propServer   = owner->getPropertyServer();
        d2cProfileList_   = propServer->getStructureDataListProtoV1_1_T<OBD2CProfile, 0>(OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST);
    }
    catch(const std::exception &e) {
        LOG_ERROR("Get depth to color profile list failed! {}", e.what());
    }

    // imu param
    std::vector<uint8_t> data;
    BEGIN_TRY_EXECUTE({
        auto owner        = getOwner();
        auto propServer   = owner->getPropertyServer();
        propServer->getRawData(
            OB_RAW_DATA_IMU_CALIB_PARAM,
            [&](OBDataTranState state, OBDataChunk *dataChunk) {
                if(state == DATA_TRAN_STAT_TRANSFERRING) {
                    data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                }
            },
            PROP_ACCESS_INTERNAL);
    })
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get imu calibration params failed!");
        data.clear();
    })
    if(!data.empty()) {
        imuCalibParam_ = IMUCorrector::parserIMUCalibrationParamsRaw(data.data(), static_cast<uint32_t>(data.size()));
        LOG_DEBUG("Get imu calibration params success!");
    }
}

void G330AlgParamManager::registerBasicExtrinsics() {
    auto extrinsicMgr          = StreamExtrinsicsManager::getInstance();
    depthBasicStreamProfile_   = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_DEPTH, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    colorBasicStreamProfile_   = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    leftIrBasicStreamProfile_  = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_IR_LEFT, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    rightIrBasicStreamProfile_ = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_IR_RIGHT, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    accelBasicStreamProfile_   = StreamProfileFactory::createAccelStreamProfile(OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    gyroBasicStreamProfile_    = StreamProfileFactory::createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);

    if(!calibrationCameraParamList_.empty()) {
        auto d2cExtrinsic = calibrationCameraParamList_.front().transform;
        extrinsicMgr->registerExtrinsics(depthBasicStreamProfile_, colorBasicStreamProfile_, d2cExtrinsic);
    }
    extrinsicMgr->registerSameExtrinsics(leftIrBasicStreamProfile_, depthBasicStreamProfile_);

    if(!depthCalibParamList_.empty()) {
        auto left_to_right     = IdentityExtrinsic;
        left_to_right.trans[0] = depthCalibParamList_.front().baseline * depthCalibParamList_.front().unit;
        extrinsicMgr->registerExtrinsics(leftIrBasicStreamProfile_, rightIrBasicStreamProfile_, left_to_right);
    }

    // todo: use shared_ptr to manage imuCalibParam_
    double imuExtr[16] = { 0 };
    memcpy(imuExtr, imuCalibParam_.singleIMUParams[0].imu_to_cam_extrinsics, sizeof(imuExtr));

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
    extrinsicMgr->registerExtrinsics(accelBasicStreamProfile_, depthBasicStreamProfile_, imu_to_depth);
    extrinsicMgr->registerSameExtrinsics(gyroBasicStreamProfile_, accelBasicStreamProfile_);
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

    auto owner      = getOwner();
    auto deviceInfo = owner->getInfo();
    auto iter       = std::find(G330LDevPids.begin(), G330LDevPids.end(), deviceInfo->pid_);

    if(iter != G330LDevPids.end()) {
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
        auto          colorProfile = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_UNKNOWN, colorRes.width, colorRes.height, 30);
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
        colorIntrinsic.width  = static_cast<int16_t>(colorProfile->getWidth());
        colorIntrinsic.height = static_cast<int16_t>((float)colorIntrinsic.height * ratio);
        for(const auto &depthRes: depthResolutions) {
            auto depthProfile = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_DEPTH, OB_FORMAT_UNKNOWN, depthRes.width, depthRes.height, 30);
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
            depthIntrinsic.width  = static_cast<int16_t>(depthProfile->getWidth());
            depthIntrinsic.height = static_cast<int16_t>((float)depthIntrinsic.height * ratio);

            auto index = fixedCalibrationCameraParamList_.size();
            fixedCalibrationCameraParamList_.push_back({ depthIntrinsic, colorIntrinsic, depthAlignParam.depthDistortion, depthAlignParam.rgbDistortion,
                                                         depthAlignParam.transform, depthAlignParam.isMirrored });

            OBD2CProfile d2cProfile;
            d2cProfile.alignType        = ALIGN_D2C_SW;
            d2cProfile.postProcessParam = { 1.0f, 0, 0, 0, 0 };
            d2cProfile.colorWidth       = static_cast<int16_t>(colorProfile->getWidth());
            d2cProfile.colorHeight      = static_cast<int16_t>(colorProfile->getHeight());
            d2cProfile.depthWidth       = static_cast<int16_t>(depthProfile->getWidth());
            d2cProfile.depthHeight      = static_cast<int16_t>(depthProfile->getHeight());
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

        auto colorProfile = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_UNKNOWN, profile.colorWidth, profile.colorHeight, 30);

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
        fixedCameraParam.rgbIntrinsic.width  = static_cast<int16_t>((float)fixedCameraParam.rgbIntrinsic.width * ratio);
        fixedCameraParam.rgbIntrinsic.height = static_cast<int16_t>((float)fixedCameraParam.rgbIntrinsic.height * ratio);

        // Scale the rgb(target) intrinsic parameters by the same factor as the depth intrinsic scale.
        auto depthRatio = (float)fixedCameraParam.depthIntrinsic.width / profile.depthWidth;
        fixedCameraParam.rgbIntrinsic.fx *= depthRatio;
        fixedCameraParam.rgbIntrinsic.fy *= depthRatio;
        fixedCameraParam.rgbIntrinsic.cx *= depthRatio;
        fixedCameraParam.rgbIntrinsic.cy *= depthRatio;
        fixedCameraParam.rgbIntrinsic.width  = static_cast<int16_t>((float)fixedCameraParam.rgbIntrinsic.width * depthRatio);
        fixedCameraParam.rgbIntrinsic.height = static_cast<int16_t>((float)fixedCameraParam.rgbIntrinsic.height * depthRatio);

        // according to post process param to reverse process the rgb(target) intrinsic
        fixedCameraParam.rgbIntrinsic.fx     = fixedCameraParam.rgbIntrinsic.fx / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.fy     = fixedCameraParam.rgbIntrinsic.fy / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.cx     = (fixedCameraParam.rgbIntrinsic.cx - profile.postProcessParam.alignLeft) / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.cy     = (fixedCameraParam.rgbIntrinsic.cy - profile.postProcessParam.alignTop) / profile.postProcessParam.depthScale;
        fixedCameraParam.rgbIntrinsic.width  = profile.depthWidth;
        fixedCameraParam.rgbIntrinsic.height = profile.depthHeight;

        auto index = fixedCalibrationCameraParamList_.size();
        fixedCalibrationCameraParamList_.push_back(fixedCameraParam);
        // auto oldProfile    = profile;
        profile.paramIndex = (uint8_t)index;

        // std::stringstream ss;
        // ss << "Fix align calibration camera params:" << std::endl;
        // ss << oldProfile << std::endl;
        // ss << cameraParam.rgbIntrinsic << std::endl;
        // ss << "to:" << std::endl;
        // ss << profile << std::endl;
        // ss << fixedCameraParam.rgbIntrinsic << std::endl;
        // LOG_INFO("- {}", ss.str());
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

void G330AlgParamManager::bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    bindExtrinsic(streamProfileList);
    bindIntrinsic(streamProfileList);
    bindDisparityParam(streamProfileList);
}

void G330AlgParamManager::bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto extrinsicMgr            = StreamExtrinsicsManager::getInstance();
    auto matchBasicStreamProfile = [&](std::shared_ptr<const StreamProfile> profile) {
        auto spType = profile->getType();
        switch(spType) {
        case OB_STREAM_DEPTH:
            return depthBasicStreamProfile_;
        case OB_STREAM_COLOR:
            return colorBasicStreamProfile_;
        case OB_STREAM_IR_LEFT:
            return leftIrBasicStreamProfile_;
        case OB_STREAM_IR_RIGHT:
            return rightIrBasicStreamProfile_;
        case OB_STREAM_ACCEL:
            return accelBasicStreamProfile_;
        case OB_STREAM_GYRO:
            return gyroBasicStreamProfile_;
        default:
            return std::shared_ptr<const StreamProfile>(nullptr);
        }
    };
    for(auto &sp: streamProfileList) {
        auto src = matchBasicStreamProfile(sp);
        extrinsicMgr->registerSameExtrinsics(sp, src);
    }
}

void G330AlgParamManager::bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    for(const auto &sp: streamProfileList) {
        if(sp->is<AccelStreamProfile>()) {
            OBAccelIntrinsic accIntrinsic;
            memcpy(&accIntrinsic, &imuCalibParam_.singleIMUParams[0].acc, sizeof(OBAccelIntrinsic));
            intrinsicMgr->registerAccelStreamIntrinsics(sp, accIntrinsic);
        }
        else if(sp->is<GyroStreamProfile>()) {
            OBGyroIntrinsic gyroIntrinsic;
            memcpy(&gyroIntrinsic, &imuCalibParam_.singleIMUParams[0].gyro, sizeof(OBGyroIntrinsic));
            intrinsicMgr->registerGyroStreamIntrinsics(sp, gyroIntrinsic);
        }
        else {
            OBCameraIntrinsic  intrinsic  = { 0 };
            OBCameraDistortion distortion = { 0 };
            OBCameraParam      param      {};
            auto               vsp        = sp->as<VideoStreamProfile>();
            if(!findBestMatchedCameraParam(calibrationCameraParamList_, vsp, param)) {
                // throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
                continue;
            }
            switch(sp->getType()) {
            case OB_STREAM_COLOR:
                intrinsic  = param.rgbIntrinsic;
                distortion = param.rgbDistortion;
                break;
            case OB_STREAM_DEPTH:
            case OB_STREAM_IR:
            case OB_STREAM_IR_LEFT:
            case OB_STREAM_IR_RIGHT:
                intrinsic  = param.depthIntrinsic;
                distortion = param.depthDistortion;
                break;
            default:
                break;
            }
            auto ratio = (float)vsp->getWidth() / (float)intrinsic.width;
            intrinsic.fx *= ratio;
            intrinsic.fy *= ratio;
            intrinsic.cx *= ratio;
            intrinsic.cy *= ratio;
            intrinsic.width  = static_cast<int16_t>(vsp->getWidth());
            intrinsic.height = static_cast<int16_t>((float)intrinsic.height * ratio);

            intrinsicMgr->registerVideoStreamIntrinsics(sp, intrinsic);
            intrinsicMgr->registerVideoStreamDistortion(sp, distortion);
        }
    }
}

void G330AlgParamManager::bindDisparityParam(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto dispParam    = getCurrentDisparityProcessParam();
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    for(const auto &sp: streamProfileList) {
        if(!sp->is<DisparityBasedStreamProfile>()) {
            continue;
        }
        intrinsicMgr->registerDisparityBasedStreamDisparityParam(sp, dispParam);
    }
}

OBDisparityParam G330AlgParamManager::getCurrentDisparityProcessParam() {
    try {
        auto owner           = getOwner();
        auto propServer      = owner->getPropertyServer();
        depthCalibParamList_ = propServer->getStructureDataListProtoV1_1_T<OBDepthCalibrationParam, 1>(OB_RAW_DATA_DEPTH_CALIB_PARAM);
    }
    catch(const std::exception &e) {
        LOG_ERROR("Get depth calibration params failed! {}", e.what());
    }

    OBDisparityParam param      = { 0 };
    const auto      &depthCalib = depthCalibParamList_.front();
    param.baseline              = depthCalib.baseline;
    param.zpd                   = depthCalib.z0;
    param.fx                    = depthCalib.focalPix;
    param.zpps                  = depthCalib.z0 / depthCalib.focalPix;
    param.bitSize               = 14;  // low 14 bit
    param.dispIntPlace          = 8;
    param.unit                  = depthCalib.unit;
    param.dispOffset            = depthCalib.dispOffset;
    param.invalidDisp           = depthCalib.invalidDisp;
    param.packMode              = OB_DISP_PACK_ORIGINAL_NEW;
    param.isDualCamera          = true;
    return param;
}

// bool G330AlgParamManager::isBinocularCamera() const {
//     // const auto &depthCalib = depthCalibParamList_.front();
//     // return depthCalib.depthMode == (uint32_t)DEPTH_MODE_STEREO;
//     return true;
// }

}  // namespace libobsensor