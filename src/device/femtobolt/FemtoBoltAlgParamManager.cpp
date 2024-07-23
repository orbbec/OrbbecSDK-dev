#include "FemtoBoltAlgParamManager.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "property/InternalProperty.hpp"
#include "DevicePids.hpp"
#include "exception/ObException.hpp"
#include "publicfilters/IMUCorrector.hpp"

#include <vector>
#include <sstream>
#define D2C_PARAMS_ITEM_SIZE 0xB0

namespace libobsensor {
FemtoBoltAlgParamManager::FemtoBoltAlgParamManager(IDevice *owner) : DeviceComponentBase(owner) {
    fetchParams();
   // registerBasicExtrinsics();
}

std::vector<OBCameraParam> alignCalibParamParse(uint8_t *filedata, uint32_t size) {
    std::vector<OBCameraParam> output;
    for(int i = 0; i < static_cast<int>(size / D2C_PARAMS_ITEM_SIZE); i++) {
        output.push_back(*(OBCameraParam *)(filedata + i * D2C_PARAMS_ITEM_SIZE));
    }
    return output;
}

std::vector<OBD2CProfile> d2cProfileInfoParse(uint8_t *filedata, uint32_t size) {
    std::vector<OBD2CProfile> output;
    // int                       strustSize = sizeof(OBD2CProfile);
    for(int i = 0; i < static_cast<int>(size / sizeof(OBD2CProfile)); i++) {
        output.push_back(*(OBD2CProfile *)(filedata + i * sizeof(OBD2CProfile)));
    }
    return output;
}

void FemtoBoltAlgParamManager::fetchParams() {

    std::vector<uint8_t> data;
    // uint32_t             size = 0;
    // align param
    data.clear();
    BEGIN_TRY_EXECUTE({
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        propServer->getRawData(
            OB_RAW_DATA_ALIGN_CALIB_PARAM,
            [&](OBDataTranState state, OBDataChunk *dataChunk) {
                if(state == DATA_TRAN_STAT_TRANSFERRING) {
                    data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                }
            },
            PROP_ACCESS_INTERNAL);
    })
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get align calibration params failed!");
        data.clear();
    })

    if(!data.empty()) {
        calibrationCameraParamList_ = alignCalibParamParse(data.data(), static_cast<uint32_t>(data.size()));
        LOG_DEBUG("Get align calibration camera params success! num={}", calibrationCameraParamList_.size());
        for(auto &&cameraParam: calibrationCameraParamList_) {
            std::stringstream ss;
            ss << cameraParam;
            LOG_DEBUG("- {}", ss.str());
        }
    }

    // d2c align profile
    data.clear();
    BEGIN_TRY_EXECUTE({
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        propServer->getRawData(
            OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST,
            [&](OBDataTranState state, OBDataChunk *dataChunk) {
                if(state == DATA_TRAN_STAT_TRANSFERRING) {
                    data.insert(data.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                }
            },
            PROP_ACCESS_INTERNAL);
    })
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get align calibration params failed!");
        data.clear();
    })

    if(!data.empty()) {
        d2cProfileList_ = d2cProfileInfoParse(data.data(), static_cast<uint32_t>(data.size()));
        auto iter       = d2cProfileList_.begin();
        while(iter != d2cProfileList_.end()) {
            if((*iter).alignType == ALIGN_D2C_HW_MODE) {
                iter = d2cProfileList_.erase(iter);
                continue;
            }
            iter++;
        }
        LOG_DEBUG("Get depth to color profile list success! num={}", d2cProfileList_.size());
        // for(auto &&profile: d2cProfileList_) {
        //     std::stringstream ss;
        //     ss << profile;
        //     LOG_DEBUG(" - {}", ss.str());
        // }
    }

    // imu param
    data.clear();
    BEGIN_TRY_EXECUTE({
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
    })
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_ERROR("Get align calibration params failed!");
        data.clear();
    })

    if(!data.empty()) {
        imuCalibParam_ = IMUCorrector::parserIMUCalibrationParamsRaw(data.data(), static_cast<uint32_t>(data.size()));
        LOG_DEBUG("Get imu calibration params success!");
    }
    else {
        LOG_WARN("Get imu calibration param failed!load default param.");
        // auto fs        = cmrc::ob::get_filesystem();
        // auto defConfig = fs.open("config/imu_calib_dft_0.2.yaml");
        // imuCalibParam_ = parserIMUCalibrationParams(defConfig.begin());
    }
}

void FemtoBoltAlgParamManager::registerBasicExtrinsics() {
    auto extrinsicMgr        = StreamExtrinsicsManager::getInstance();
    depthEmptyStreamProfile_ = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_DEPTH, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    colorEmptyStreamProfile_ = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    accelEmptyStreamProfile_ = StreamProfileFactory::createAccelStreamProfile(OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    gyroEmptyStreamProfile_  = StreamProfileFactory::createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);

    if(!calibrationCameraParamList_.empty()) {
        auto d2cExtrinsic = calibrationCameraParamList_.front().transform;
        extrinsicMgr->registerExtrinsics(depthEmptyStreamProfile_, colorEmptyStreamProfile_, d2cExtrinsic);
    }

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

void FemtoBoltAlgParamManager::bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    bindExtrinsic(streamProfileList);
    bindExtrinsic(streamProfileList);
}

void FemtoBoltAlgParamManager::bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto extrinsicMgr            = StreamExtrinsicsManager::getInstance();
    auto matchEmptyStreamProfile = [&](std::shared_ptr<const StreamProfile> profile) {
        auto spType = profile->getType();
        switch(spType) {
        case OB_STREAM_DEPTH:
            return depthEmptyStreamProfile_;
            break;
        case OB_STREAM_COLOR:
            return colorEmptyStreamProfile_;
            break;
        case OB_STREAM_ACCEL:
            return accelEmptyStreamProfile_;
            break;
        case OB_STREAM_GYRO:
            return gyroEmptyStreamProfile_;
            break;
        default:
            return std::shared_ptr<const StreamProfile>(nullptr);
            break;
        }
    };

    for(auto &sp: streamProfileList) {
        auto src = matchEmptyStreamProfile(sp);
        extrinsicMgr->registerSameExtrinsics(sp, src);
    }
}

void FemtoBoltAlgParamManager::bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
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
            OBCameraParam      param{};
            auto               vsp = sp->as<VideoStreamProfile>();
            // if(!findBestMatchedCameraParam(calibrationCameraParamList_, vsp, param)) {
            //     // throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
            //     continue;
            // }
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
}  // namespace libobsensor