#include "IMUParam.hpp"
namespace libobsensor {
float calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR) {
    float sensitivity = 0.f;

    switch(gyroFSR) {
    case OB_GYRO_FS_16dps: /*!< 16dps*/
        sensitivity = GYRO_MAX / 16.0;
        break;
    case OB_GYRO_FS_31dps: /*!< 31dps*/
        sensitivity = GYRO_MAX / 31.0;
        break;
    case OB_GYRO_FS_62dps: /*!< 62dps*/
        sensitivity = GYRO_MAX / 62.0;
        break;
    case OB_GYRO_FS_125dps: /*!< 125dps*/
        sensitivity = GYRO_MAX / 125.0;
        break;
    case OB_GYRO_FS_250dps: /*!< 250dps*/
        sensitivity = GYRO_MAX / 250.0;
        break;
    case OB_GYRO_FS_500dps: /*!< 500dps*/
        sensitivity = GYRO_MAX / 500.0;
        break;
    case OB_GYRO_FS_1000dps: /*!< 1000dps*/
        sensitivity = GYRO_MAX / 1000.0;
        break;
    case OB_GYRO_FS_2000dps: /*!< 2000dps*/
        sensitivity = GYRO_MAX / 2000.0;
        break;
    }

    return (gyroValue / sensitivity);
}

float calculateAccelGravi(int16_t accelValue, uint8_t accelFSR) {
    float sensitivity = 0.f;

    switch(accelFSR) {
    case OB_ACCEL_FS_2g: /*!< 2g*/
        sensitivity = ACCEL_MAX / 2.0;
        break;
    case OB_ACCEL_FS_4g: /*!< 4g*/
        sensitivity = ACCEL_MAX / 4.0;
        break;
    case OB_ACCEL_FS_8g: /*!< 8g*/
        sensitivity = ACCEL_MAX / 8.0;
        break;
    case OB_ACCEL_FS_16g: /*!< 16g*/
        sensitivity = ACCEL_MAX / 16.0;
        break;
    }

    return (accelValue / sensitivity);
}

float calculateRegisterTemperature(int16_t tempValue) {
    return (tempValue / 132.48 + 25);
}

// IMUCalibrateParams parserIMUCalibrationParams(const char *data){
IMUCalibrateParams parserIMUCalibrationParams(std::string fileStream) {
    IMUParamParseErr status = IMUParamParseErr::IMU_PARAM_PARSE_OK;

    IMUCalibrateParams load_params{};
    // status = loadCalibrationParamsMulti(data, &load_params, IMU_CALIB_PARAM_FORMAT_YAML);
    status = loadCalibrationIMUParamsMulti(fileStream.c_str(), &load_params, IMU_CALIB_PARAM_FORMAT_YAML);

    return load_params;
}

IMUCalibrateParams parserIMUCalibrationParams(const char *data) {
    IMUParamParseErr status = IMUParamParseErr::IMU_PARAM_PARSE_OK;

    IMUCalibrateParams load_params{};
    status = loadCalibrationIMUParamsMultiFromData(data, &load_params, IMU_CALIB_PARAM_FORMAT_YAML);

    return load_params;
}

IMUCalibrateParams parserIMUCalibrationParamsRaw(uint8_t *imudata, uint32_t size) {
    auto               singleIMUParamSize = sizeof(SingleIMUParams);
    uint8_t            paramCount         = size / singleIMUParamSize;
    IMUCalibrateParams params{};

    params.validNum = (paramCount > 3) ? 3 : paramCount;

    for(int i = 0; i < params.validNum; i++) {
        auto singleParam          = *((SingleIMUParams *)(imudata + (i * singleIMUParamSize)));
        params.singleIMUParams[i] = singleParam;
    }
    return params;
}

Eigen::Vector3d correctAccel(const Eigen::Vector3d& accelVec, IMUCalibrateParams *params) {
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

Eigen::Vector3d correctGyro(const Eigen::Vector3d& gyroVec, IMUCalibrateParams *params) {
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