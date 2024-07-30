#include "FemtoBoltAlgParamManager.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "property/InternalProperty.hpp"
#include "DevicePids.hpp"
#include "exception/ObException.hpp"
#include "publicfilters/IMUCorrector.hpp"
#include "param/AlgParseHelper.hpp"

#include <vector>
#include <sstream>


namespace libobsensor {
FemtoBoltAlgParamManager::FemtoBoltAlgParamManager(IDevice *owner) : AlgParamManagerBase(owner) {
    fetchParams();
    registerBasicExtrinsics();
}

// std::vector<OBCameraParam> alignCalibParamParse(uint8_t *data, uint32_t size) {
//     std::vector<OBCameraParam> output;
//     for(int i = 0; i < static_cast<int>(size / D2C_PARAMS_ITEM_SIZE); i++) {
//         output.push_back(*(OBCameraParam *)(data + i * D2C_PARAMS_ITEM_SIZE));
//     }
//     return output;
// }

// std::vector<OBD2CProfile> d2cProfileInfoParse(uint8_t *data, uint32_t size) {
//     std::vector<OBD2CProfile> output;
//     for(int i = 0; i < static_cast<int>(size / sizeof(OBD2CProfile)); i++) {
//         output.push_back(*(OBD2CProfile *)(data + i * sizeof(OBD2CProfile)));
//     }
//     return output;
// }

void FemtoBoltAlgParamManager::fetchParams() {
    std::vector<uint8_t> data;
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
        calibrationCameraParamList_ = AlgParseHelper::alignCalibParamParse(data.data(), static_cast<uint32_t>(data.size()));
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
        d2cProfileList_ = AlgParseHelper::d2cProfileInfoParse(data.data(), static_cast<uint32_t>(data.size()));
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
        // 由于保证读写文件保持一致性，而IMU标定参数文件携带了校验头数据，需要SDK端进行偏移，此做法与MX6600系列不一致。
        auto realData  = data.data() + IMU_CALIBRATION_FILE_OFFSET;
        auto realSize  = static_cast<uint32_t>(data.size()) - IMU_CALIBRATION_FILE_OFFSET;
        imuCalibParam_ = IMUCorrector::parserIMUCalibParamRaw(realData, realSize);
        LOG_DEBUG("Get imu calibration params success!");
    }
    else {
        LOG_WARN("Get imu calibration param failed!load default param.");
        imuCalibParam_ = IMUCorrector::getDefaultImuCalibParam();
    }
}

void FemtoBoltAlgParamManager::registerBasicExtrinsics() {
    auto extrinsicMgr            = StreamExtrinsicsManager::getInstance();
    auto depthBasicStreamProfile = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_DEPTH, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    auto colorBasicStreamProfile = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_COLOR, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    auto irBasicStreamProfile    = StreamProfileFactory::createVideoStreamProfile(OB_STREAM_IR, OB_FORMAT_ANY, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY);
    auto accelBasicStreamProfile = StreamProfileFactory::createAccelStreamProfile(OB_ACCEL_FS_2g, OB_SAMPLE_RATE_1_5625_HZ);
    auto gyroBasicStreamProfile  = StreamProfileFactory::createGyroStreamProfile(OB_GYRO_FS_16dps, OB_SAMPLE_RATE_1_5625_HZ);

    if(!calibrationCameraParamList_.empty()) {
        auto d2cExtrinsic = calibrationCameraParamList_.front().transform;
        extrinsicMgr->registerExtrinsics(depthBasicStreamProfile, colorBasicStreamProfile, d2cExtrinsic);
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
    extrinsicMgr->registerExtrinsics(accelBasicStreamProfile, depthBasicStreamProfile, imu_to_depth);
    extrinsicMgr->registerSameExtrinsics(gyroBasicStreamProfile, accelBasicStreamProfile);

    basicStreamProfileList_.emplace_back(depthBasicStreamProfile);
    basicStreamProfileList_.emplace_back(colorBasicStreamProfile);
    basicStreamProfileList_.emplace_back(irBasicStreamProfile);
    basicStreamProfileList_.emplace_back(accelBasicStreamProfile);
    basicStreamProfileList_.emplace_back(gyroBasicStreamProfile);
}
}  // namespace libobsensor
