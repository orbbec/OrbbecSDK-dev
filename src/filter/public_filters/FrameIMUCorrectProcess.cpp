#include "FrameIMUCorrectProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "InternalTypes.hpp"

namespace libobsensor {

OBIMUCalibrateParams IMUCorrecter::parserIMUCalibrationParamsRaw(uint8_t *filedata, uint32_t size) {
    auto                 singleIMUParamSize = sizeof(OBSingleIMUParams);
    uint8_t            paramCount         = static_cast<uint8_t>(size / singleIMUParamSize);
    OBIMUCalibrateParams    params{};

    params.validNum = (paramCount > 3) ? 3 : paramCount;

    for(int i = 0; i < params.validNum; i++) {
        auto singleParam          = *((OBSingleIMUParams *)(filedata + (i * singleIMUParamSize)));
        params.singleIMUParams[i] = singleParam;
    }
    return params;
}

float IMUCorrecter::calculateAccelGravity(int16_t accelValue, uint8_t accelFSR) {
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

float IMUCorrecter::calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR) {
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

float IMUCorrecter::calculateRegisterTemperature(int16_t tempValue) {
    float ret = static_cast<float>(tempValue / 132.48 + 25);
    return ret;
}

IMUCorrecter::IMUCorrecter(const std::string &name) : FilterBase(name) {}

void IMUCorrecter::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL) {
        throw invalid_value_exception("Not support this property");
    }
    FilterBase::enable(static_cast<bool>(value.intValue));
}

void IMUCorrecter::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL) {
        throw invalid_value_exception("Not support this property");
    }
    value->intValue = static_cast<int32_t>(isEnabled());
}

void IMUCorrecter::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    if(propertyId != OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL && propertyId != OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL) {
        throw invalid_value_exception("Not support this property");
    }
    range->cur.intValue  = static_cast<int32_t>(isEnabled());
    range->def.intValue  = 1;
    range->max.intValue  = 1;
    range->min.intValue  = 0;
    range->step.intValue = 1;
}

void IMUCorrecter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("IMUCorrecter update config error: unsupported operation.");
    }
}

const std::string &IMUCorrecter::getConfigSchema() const {
    static const std::string schema = "";
    return schema;
}

std::shared_ptr<Frame> IMUCorrecter::processFunc(std::shared_ptr<const Frame> frame) {
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

        auto            frameData = (AccelFrame::OBAccelFrameData *)accelFrame->getData();
        OBPoint3f       accelValue = { frameData->accelData[0], frameData->accelData[1], frameData->accelData[2] };
        accelValue                 = correctAccel(accelValue, &intrisic);
        frameData->accelData[0]    = accelValue.x;
        frameData->accelData[1]    = accelValue.y;
        frameData->accelData[2]    = accelValue.z;
        frameData->temp         = calculateRegisterTemperature(static_cast<int16_t>(frameData->temp));
    }

    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if(gyroFrame) {
        auto sp       = gyroFrame->getStreamProfile();
        auto gyroSp   = sp->as<GyroStreamProfile>();
        auto intrisic = gyroSp->getIntrinsic();

        auto            frameData = (GyroFrame::OBGyroFrameData *)gyroFrame->getData();
        OBPoint3f       gyroValue = { frameData->gyroData[0], frameData->gyroData[1], frameData->gyroData[2] };
        gyroValue                 = correctGyro(gyroValue, &intrisic);
        frameData->gyroData[0]    = gyroValue.x;
        frameData->gyroData[1]    = gyroValue.y;
        frameData->gyroData[2]    = gyroValue.z;
    }

    return newFrame;
}

OBPoint3f IMUCorrecter::correctAccel(const OBPoint3f &accelVec, OBAccelIntrinsic *intrinsic) {
    float M_acc[3][3];
    float bias_acc[3];

    for(int i = 0; i < 3; i++) {
        M_acc[i][0] = intrinsic->scaleMisalignment[3 * i];
        M_acc[i][1] = intrinsic->scaleMisalignment[3 * i + 1];
        M_acc[i][2] = intrinsic->scaleMisalignment[3 * i + 2];
    }

    bias_acc[0] = intrinsic->bias[0];
    bias_acc[1] = intrinsic->bias[1];
    bias_acc[2] = intrinsic->bias[2];

    float correctedAccel[3];
    for(int i = 0; i < 3; i++) {
        correctedAccel[i] = M_acc[i][0] * (accelVec.x - bias_acc[0]) + M_acc[i][1] * (accelVec.y - bias_acc[1]) + M_acc[i][2] * (accelVec.z - bias_acc[2]);
    }

    return { correctedAccel[0], correctedAccel[1], correctedAccel[2] };
}

OBPoint3f IMUCorrecter::correctGyro(const OBPoint3f &gyroVec, OBGyroIntrinsic *intrinsic) {
    float M_gyro[3][3];
    float bias_gyro[3];

    for(int i = 0; i < 3; i++) {
        M_gyro[i][0] = intrinsic->scaleMisalignment[3 * i];
        M_gyro[i][1] = intrinsic->scaleMisalignment[3 * i + 1];
        M_gyro[i][2] = intrinsic->scaleMisalignment[3 * i + 2];
    }

    bias_gyro[0] = intrinsic->bias[0];
    bias_gyro[1] = intrinsic->bias[1];
    bias_gyro[2] = intrinsic->bias[2];

    float correctedGyro[3];
    for(int i = 0; i < 3; i++) {
        correctedGyro[i] = M_gyro[i][0] * (gyroVec.x - bias_gyro[0]) + M_gyro[i][1] * (gyroVec.y - bias_gyro[1]) + M_gyro[i][2] * (gyroVec.z - bias_gyro[2]);
    }

    return { correctedGyro[0], correctedGyro[1], correctedGyro[2] };
}

}  // namespace libobsensor
