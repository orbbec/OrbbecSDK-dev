#include "IMUCorrector.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "InternalTypes.hpp"

namespace libobsensor {

OBIMUCalibrateParams IMUCorrector::parserIMUCalibrationParamsRaw(uint8_t *data, uint32_t size) {
    auto                 singleIMUParamSize = sizeof(OBSingleIMUParams);
    uint8_t              paramCount         = static_cast<uint8_t>(size / singleIMUParamSize);
    OBIMUCalibrateParams params{};

    params.validNum = (paramCount > 3) ? 3 : paramCount;

    for(int i = 0; i < params.validNum; i++) {
        auto singleParam          = *((OBSingleIMUParams *)(data + (i * singleIMUParamSize)));
        params.singleIMUParams[i] = singleParam;
    }
    return params;
}

float IMUCorrector::calculateAccelGravity(int16_t accelValue, uint8_t accelFSR) {
    float sensitivity = 0.f;

    switch(accelFSR) {
    case OB_ACCEL_FS_2g: /*!< 2g*/
        sensitivity = ACCEL_MAX / 2.0f;
        break;
    case OB_ACCEL_FS_4g: /*!< 4g*/
        sensitivity = ACCEL_MAX / 4.0f;
        break;
    case OB_ACCEL_FS_8g: /*!< 8g*/
        sensitivity = ACCEL_MAX / 8.0f;
        break;
    case OB_ACCEL_FS_16g: /*!< 16g*/
        sensitivity = ACCEL_MAX / 16.0f;
        break;
    }

    return (accelValue / sensitivity);
}

float IMUCorrector::calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR) {
    float sensitivity = 0.f;

    switch(gyroFSR) {
    case OB_GYRO_FS_16dps: /*!< 16dps*/
        sensitivity = GYRO_MAX / 16.0f;
        break;
    case OB_GYRO_FS_31dps: /*!< 31dps*/
        sensitivity = GYRO_MAX / 31.0f;
        break;
    case OB_GYRO_FS_62dps: /*!< 62dps*/
        sensitivity = GYRO_MAX / 62.0f;
        break;
    case OB_GYRO_FS_125dps: /*!< 125dps*/
        sensitivity = GYRO_MAX / 125.0f;
        break;
    case OB_GYRO_FS_250dps: /*!< 250dps*/
        sensitivity = GYRO_MAX / 250.0f;
        break;
    case OB_GYRO_FS_500dps: /*!< 500dps*/
        sensitivity = GYRO_MAX / 500.0f;
        break;
    case OB_GYRO_FS_1000dps: /*!< 1000dps*/
        sensitivity = GYRO_MAX / 1000.0f;
        break;
    case OB_GYRO_FS_2000dps: /*!< 2000dps*/
        sensitivity = GYRO_MAX / 2000.0f;
        break;
    }

    return (gyroValue / sensitivity);
}

float IMUCorrector::calculateRegisterTemperature(int16_t tempValue) {
    float ret = static_cast<float>(tempValue / 132.48 + 25);
    return ret;
}

IMUCorrector::IMUCorrector(const std::string &name) : FilterBase(name) {}

void IMUCorrector::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("IMUCorrector update config error: unsupported operation.");
    }
}

const std::string &IMUCorrector::getConfigSchema() const {
    static const std::string schema = "";
    return schema;
}

std::shared_ptr<Frame> IMUCorrector::processFunc(std::shared_ptr<const Frame> frame) {
    if(frame == nullptr) {
        return nullptr;
    }

    auto newFrame = FrameFactory::createFrameFromOtherFrame(frame, true);
    if(!frame->is<FrameSet>()) {
        return newFrame;
    }

    auto frameSet   = newFrame->as<FrameSet>();
    auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
    if(accelFrame) {
        auto sp       = accelFrame->getStreamProfile();
        auto accelSp  = sp->as<AccelStreamProfile>();
        auto intrisic = accelSp->getIntrinsic();

        auto      frameData     = (AccelFrame::OBAccelFrameData *)accelFrame->getData();
        OBPoint3f accelValue    = { frameData->accelData[0], frameData->accelData[1], frameData->accelData[2] };
        accelValue              = correctAccel(accelValue, &intrisic);
        frameData->accelData[0] = accelValue.x;
        frameData->accelData[1] = accelValue.y;
        frameData->accelData[2] = accelValue.z;
        frameData->temp         = calculateRegisterTemperature(static_cast<int16_t>(frameData->temp));
    }

    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if(gyroFrame) {
        auto sp       = gyroFrame->getStreamProfile();
        auto gyroSp   = sp->as<GyroStreamProfile>();
        auto intrisic = gyroSp->getIntrinsic();

        auto      frameData    = (GyroFrame::OBGyroFrameData *)gyroFrame->getData();
        OBPoint3f gyroValue    = { frameData->gyroData[0], frameData->gyroData[1], frameData->gyroData[2] };
        gyroValue              = correctGyro(gyroValue, &intrisic);
        frameData->gyroData[0] = gyroValue.x;
        frameData->gyroData[1] = gyroValue.y;
        frameData->gyroData[2] = gyroValue.z;
    }

    return newFrame;
}

OBPoint3f IMUCorrector::correctAccel(const OBPoint3f &accelVec, OBAccelIntrinsic *intrinsic) {
    double M_acc[3][3];
    double bias_acc[3];

    for(int i = 0; i < 3; i++) {
        M_acc[i][0] = intrinsic->scaleMisalignment[3 * i];
        M_acc[i][1] = intrinsic->scaleMisalignment[3 * i + 1];
        M_acc[i][2] = intrinsic->scaleMisalignment[3 * i + 2];
    }

    bias_acc[0] = intrinsic->bias[0];
    bias_acc[1] = intrinsic->bias[1];
    bias_acc[2] = intrinsic->bias[2];

    double correctedAccel[3];
    for(int i = 0; i < 3; i++) {
        correctedAccel[i] = M_acc[i][0] * (accelVec.x - bias_acc[0]) + M_acc[i][1] * (accelVec.y - bias_acc[1]) + M_acc[i][2] * (accelVec.z - bias_acc[2]);
    }

    return { static_cast<float>(correctedAccel[0]), static_cast<float>(correctedAccel[1]), static_cast<float>(correctedAccel[2]) };
}

OBPoint3f IMUCorrector::correctGyro(const OBPoint3f &gyroVec, OBGyroIntrinsic *intrinsic) {
    double M_gyro[3][3];
    double bias_gyro[3];

    for(int i = 0; i < 3; i++) {
        M_gyro[i][0] = intrinsic->scaleMisalignment[3 * i];
        M_gyro[i][1] = intrinsic->scaleMisalignment[3 * i + 1];
        M_gyro[i][2] = intrinsic->scaleMisalignment[3 * i + 2];
    }

    bias_gyro[0] = intrinsic->bias[0];
    bias_gyro[1] = intrinsic->bias[1];
    bias_gyro[2] = intrinsic->bias[2];

    double correctedGyro[3];
    for(int i = 0; i < 3; i++) {
        correctedGyro[i] = M_gyro[i][0] * (gyroVec.x - bias_gyro[0]) + M_gyro[i][1] * (gyroVec.y - bias_gyro[1]) + M_gyro[i][2] * (gyroVec.z - bias_gyro[2]);
    }

    return { static_cast<float>(correctedGyro[0]), static_cast<float>(correctedGyro[1]), static_cast<float>(correctedGyro[2]) };
}

}  // namespace libobsensor
