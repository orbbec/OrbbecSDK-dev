#include "AlgParamManager.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "property/InternalProperty.hpp"
#include "DevicePids.hpp"
#include "exception/ObException.hpp"
#include "publicfilters/IMUCorrector.hpp"

#include <vector>
#include <sstream>
#include <utility>

namespace libobsensor {

bool findBestMatchedD2CProfile(const std::vector<OBD2CProfile> &d2cProfileList, const std::shared_ptr<const VideoStreamProfile> &profile,
                               OBD2CProfile &result) {
    bool found = false;
    // match same resolution
    for(auto &d2cProfile: d2cProfileList) {
        // using software align profile
        if(d2cProfile.alignType != ALIGN_D2C_SW && d2cProfile.alignType != ALIGN_D2C_HW_SW_BOTH) {
            continue;
        }

        auto streamType = profile->getType();
        if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
           && static_cast<uint32_t>(d2cProfile.depthWidth) == profile->getWidth() && static_cast<uint32_t>(d2cProfile.depthWidth) == profile->getHeight()) {
            found  = true;
            result = d2cProfile;
            break;
        }
        else if(streamType == OB_STREAM_COLOR && static_cast<uint32_t>(d2cProfile.colorWidth) == profile->getWidth()
                && static_cast<uint32_t>(d2cProfile.colorHeight) == profile->getHeight()) {
            found  = true;
            result = d2cProfile;
            break;
        }
    }

    if(!found) {
        // match same ratio
        float ratio = (float)profile->getWidth() / profile->getHeight();
        for(auto &d2cProfile: d2cProfileList) {
            // using software align profile
            if(d2cProfile.alignType != ALIGN_D2C_SW && d2cProfile.alignType != ALIGN_D2C_HW_SW_BOTH) {
                continue;
            }

            auto streamType = profile->getType();
            if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
               && (float)d2cProfile.depthWidth / d2cProfile.depthHeight == ratio) {
                found  = true;
                result = d2cProfile;
                break;
            }
            else if(streamType == OB_STREAM_COLOR && (float)d2cProfile.colorWidth / d2cProfile.colorHeight == ratio) {
                found  = true;
                result = d2cProfile;
                break;
            }
        }
    }

    return found;
}

AlgParamManager::AlgParamManager(IDevice *owner) : DeviceComponentBase(owner) {
    fetchParams();
    registerBasicExtrinsics();
}

void AlgParamManager::fetchParams() {

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
            param.depthDistortion.model = OB_DISTORTION_BROWN_CONRADY_K6;
            memcpy(&param.rgbDistortion, &cameraParam.rgbDistortion, sizeof(param.rgbDistortion));
            param.rgbDistortion.model = OB_DISTORTION_BROWN_CONRADY_K6;
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
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        d2cProfileList_ = propServer->getStructureDataListProtoV1_1_T<OBD2CProfile, 0>(OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST);
    }
    catch(const std::exception &e) {
        LOG_ERROR("Get depth to color profile list failed! {}", e.what());
    }

    // imu param
    std::vector<uint8_t> data;
    try {
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        propServer->getRawData(
            OB_RAW_DATA_IMU_CALIB_PARAM,
            [&](OBDataTranState state, OBDataChunk *dataChunk) {
                if(state == DATA_TRAN_STAT_TRANSFERRING) {
                    data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                }
            },
            PROP_ACCESS_INTERNAL);
    }
    catch(...) {
        LOG_ERROR("Get imu calibration params failed!");
        data.clear();
    }
    if(!data.empty()) {
        imuCalibParam_ = IMUCorrector::parserIMUCalibrationParamsRaw(data.data(), static_cast<uint32_t>(data.size()));
        LOG_DEBUG("Get imu calibration params success!");
    }
}

void AlgParamManager::registerBasicExtrinsics() {
    auto extrinsicMgr          = StreamExtrinsicsManager::getInstance();
    depthEmptyStreamProfile_   = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_DEPTH, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    colorEmptyStreamProfile_   = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    leftIrEmptyStreamProfile_  = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_IR_LEFT, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    rightIrEmptyStreamProfile_ = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_IR_RIGHT, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    accelEmptyStreamProfile_   = StreamProfileFactory::createAccelStreamProfile(OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    gyroEmptyStreamProfile_    = StreamProfileFactory::createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);

    if(!calibrationCameraParamList_.empty()) {
        auto d2cExtrinsic = calibrationCameraParamList_.front().transform;
        bool mirrored     = false;
        {
            auto owner      = getOwner();
            auto propServer = owner->getPropertyServer();
            if(propServer->isPropertySupported(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
                mirrored = propServer->getPropertyValueT<bool>(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL);
            }
        }
        if(mirrored) {
            (d2cExtrinsic.rot)[1] *= -1.0;
            (d2cExtrinsic.rot)[2] *= -1.0;
            (d2cExtrinsic.rot)[3] *= -1.0;
            (d2cExtrinsic.rot)[6] *= -1.0;
            (d2cExtrinsic.trans)[0] *= -1.0;
        }
        extrinsicMgr->registerExtrinsics(depthEmptyStreamProfile_, colorEmptyStreamProfile_, d2cExtrinsic);
    }
    extrinsicMgr->registerSameExtrinsics(leftIrEmptyStreamProfile_, depthEmptyStreamProfile_);

    if(!depthCalibParamList_.empty()) {
        auto left_to_right     = IdentityExtrinsic;
        left_to_right.trans[0] = depthCalibParamList_.front().baseline * depthCalibParamList_.front().unit;
        extrinsicMgr->registerExtrinsics(leftIrEmptyStreamProfile_, rightIrEmptyStreamProfile_, left_to_right);
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
    extrinsicMgr->registerExtrinsics(accelEmptyStreamProfile_, depthEmptyStreamProfile_, imu_to_depth);
    extrinsicMgr->registerSameExtrinsics(gyroEmptyStreamProfile_, accelEmptyStreamProfile_);
}

typedef struct {
    uint32_t width;
    uint32_t height;
} Resolution;

void AlgParamManager::bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    bindExtrinsic(streamProfileList);
    bindIntrinsic(streamProfileList);
    bindDisparityParam(streamProfileList);
}

void AlgParamManager::bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto extrinsicMgr            = StreamExtrinsicsManager::getInstance();
    auto matchEmptyStreamProfile = [&](std::shared_ptr<const StreamProfile> profile) {
        auto spType = profile->getType();
        switch(spType) {
        case OB_STREAM_DEPTH:
            return depthEmptyStreamProfile_;
        case OB_STREAM_COLOR:
            return colorEmptyStreamProfile_;
        case OB_STREAM_IR:
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

void AlgParamManager::bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    bool mirrored     = false;

    {
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        if(propServer->isPropertySupported(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
            mirrored = propServer->getPropertyValueT<bool>(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL);
        }
    }

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
            OBD2CProfile       d2cProfile{};
            auto               vsp = sp->as<VideoStreamProfile>();

            if(!findBestMatchedD2CProfile(d2cProfileList_, vsp, d2cProfile)) {
                throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
            }
            const auto &param            = calibrationCameraParamList_.at(d2cProfile.paramIndex);
            auto        postProcessParam = d2cProfile.postProcessParam;

            auto streamType = sp->getType();
            if(streamType == OB_STREAM_COLOR) {
                intrinsic  = param.rgbIntrinsic;
                distortion = param.rgbDistortion;
            }
            else {
                intrinsic  = param.depthIntrinsic;
                distortion = param.depthDistortion;
            }

            if(mirrored) {
                // mirror intrinsic
                intrinsic.cx  = (float)1.0 * intrinsic.width - intrinsic.cx - 1;
                distortion.p2 = -distortion.p2;
                std::swap(postProcessParam.alignLeft, postProcessParam.alignRight);
            }

            if(streamType == OB_STREAM_COLOR) {
                intrinsic.fx = postProcessParam.depthScale * intrinsic.fx;
                intrinsic.fy = postProcessParam.depthScale * intrinsic.fy;
                intrinsic.cx = postProcessParam.depthScale * intrinsic.cx + postProcessParam.alignLeft;
                intrinsic.cy = postProcessParam.depthScale * intrinsic.cy + postProcessParam.alignTop;
                intrinsic.width =
                    static_cast<int16_t>(postProcessParam.depthScale * intrinsic.width + postProcessParam.alignLeft + postProcessParam.alignRight);
                intrinsic.height =
                    static_cast<int16_t>(postProcessParam.depthScale * intrinsic.height + postProcessParam.alignTop + postProcessParam.alignBottom);
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

void AlgParamManager::bindDisparityParam(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto dispParam    = getCurrentDisparityProcessParam();
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    for(const auto &sp: streamProfileList) {
        if(!sp->is<DisparityBasedStreamProfile>()) {
            continue;
        }
        intrinsicMgr->registerDisparityBasedStreamDisparityParam(sp, dispParam);
    }
}

OBDisparityParam AlgParamManager::getCurrentDisparityProcessParam() {
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

// bool AlgParamManager::isBinocularCamera() const {
//     // const auto &depthCalib = depthCalibParamList_.front();
//     // return depthCalib.depthMode == (uint32_t)DEPTH_MODE_STEREO;
//     return true;
// }

}  // namespace libobsensor