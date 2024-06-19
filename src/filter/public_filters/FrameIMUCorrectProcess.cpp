#include "FrameIMUCorrectProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"

namespace libobsensor {


IMUCalibrateParams IMUCorrecter::parserIMUCalibrationParamsRaw(uint8_t* filedata, uint32_t size){
    auto           singleIMUParamSize = sizeof(SingleIMUParams);
    uint8_t            paramCount         = static_cast<uint8_t>(size / singleIMUParamSize);
    IMUCalibrateParams params{};

    params.validNum = (paramCount > 3) ? 3 : paramCount;

    for(int i = 0; i < params.validNum; i++) {
        auto singleParam          = *((SingleIMUParams *)(filedata + (i * singleIMUParamSize)));
        params.singleIMUParams[i] = singleParam;
    }
    return params;
}

float IMUCorrecter::calculateAccelGravity(int16_t accelValue, uint8_t accelFSR){
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

float IMUCorrecter::calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR){
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

float IMUCorrecter::calculateRegisterTemperature(int16_t tempValue){
    float ret = static_cast<float>(tempValue / 132.48 + 25);
    return ret;
}

IMUCorrecter::IMUCorrecter(const std::string &name): FilterBase(name) {

}

void IMUCorrecter::updateConfig(std::vector<std::string> &params){
    if(params.size() != 0) {
        throw unsupported_operation_exception("IMUCorrecter update config error: unsupported operation.");
    }
}

const std::string &IMUCorrecter::getConfigSchema() const {
    throw unsupported_operation_exception("IMUCorrecter get config schema error: unsupported operation.");
}

std::shared_ptr<Frame> IMUCorrecter::processFunc(std::shared_ptr<const Frame> frame) {
    if(frame == nullptr) {
        return nullptr;
    }

    auto newFrame = FrameFactory::cloneFrame(frame, true);
    if(!frame->is<FrameSet>()){
        return newFrame;
    }

    auto frameSet   = newFrame->as<FrameSet>();
    auto accelFrame = frameSet->getFrame(OB_FRAME_ACCEL);
    if(accelFrame){
        auto frameData = (AccelFrame::OBAccelFrameData *)accelFrame->getData();
        Eigen::Vector3d accelEigenVec(frameData->accelData[0], frameData->accelData[1], frameData->accelData[2]);
        accelEigenVec           = correctAccel(accelEigenVec, &param_);
        frameData->accelData[0] = static_cast<float>(accelEigenVec.x());
        frameData->accelData[1] = static_cast<float>(accelEigenVec.y());
        frameData->accelData[2] = static_cast<float>(accelEigenVec.z());
        frameData->temp         = calculateRegisterTemperature(static_cast<int16_t>(frameData->temp));
    }

    auto gyroFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if(gyroFrame){
        auto frameData = (GyroFrame::OBGyroFrameData *)gyroFrame->getData();
        Eigen::Vector3d gyroEigenVec(frameData->gyroData[0], frameData->gyroData[1], frameData->gyroData[2]);
        gyroEigenVec = correctGyro(gyroEigenVec, &param_);
        frameData->gyroData[0] = static_cast<float>(gyroEigenVec.x());
        frameData->gyroData[1] = static_cast<float>(gyroEigenVec.y());
        frameData->gyroData[2] = static_cast<float>(gyroEigenVec.z());
        frameData->temp = calculateRegisterTemperature(static_cast<int16_t>(frameData->temp));
    }

    return newFrame;
}

Eigen::Vector3d IMUCorrecter::correctAccel(const Eigen::Vector3d& accelVec, IMUCalibrateParams *params){
    Eigen::Matrix3d M_acc;
    Eigen::Vector3d bias_acc;

    for(int i = 0; i < 3; i++) {
        M_acc(i, 0) = params->singleIMUParams->acc.scaleMisalignment[3 * i];
        M_acc(i, 1) = params->singleIMUParams->acc.scaleMisalignment[3 * i + 1];
        M_acc(i, 2) = params->singleIMUParams->acc.scaleMisalignment[3 * i + 2];
    }

    double bias0 = params->singleIMUParams->acc.bias[0];
    double bias1 = params->singleIMUParams->acc.bias[1];
    double bias2 = params->singleIMUParams->acc.bias[2];
    bias_acc << Eigen::Vector3d(bias0, bias1, bias2);

    Eigen::Vector3d accE;
    accE = M_acc * (accelVec - bias_acc);

    return accE;
}

Eigen::Vector3d IMUCorrecter::correctGyro(const Eigen::Vector3d& gyroVec, IMUCalibrateParams *params){
    Eigen::Matrix3d M_gyr;
    Eigen::Vector3d bias_gyr;

    for(int i = 0; i < 3; i++) {
        M_gyr(i, 0) = params->singleIMUParams->gyro.scaleMisalignment[3 * i];
        M_gyr(i, 1) = params->singleIMUParams->gyro.scaleMisalignment[3 * i + 1];
        M_gyr(i, 2) = params->singleIMUParams->gyro.scaleMisalignment[3 * i + 2];
    }
    // gyr estimate
    Eigen::Vector3d gyrE;
    double bias0 = params->singleIMUParams->gyro.bias[0];
    double bias1 = params->singleIMUParams->gyro.bias[1];
    double bias2 = params->singleIMUParams->gyro.bias[2];
    bias_gyr << Eigen::Vector3d(bias0, bias1, bias2);
    gyrE = M_gyr * (gyroVec - bias_gyr);
    
    return gyrE;
}

}  // namespace libobsensor
