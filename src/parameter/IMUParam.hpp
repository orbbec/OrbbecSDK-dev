#pragma once
#include "ObTypes.h"
#include "imu_calibration_params.h"
#include "imu_calibration_params_parser.h"
#include <Eigen/Core>
#include <Eigen/LU>

#define GYRO_MAX 32800 / 0.017453293
#define ACCEL_MAX 32768 / 9.80

namespace libobsensor {

    /**
     * @brief Calculates the angular velocity from the gyroscope value in degrees per second (dps).
     *
     * @param[in] gyroValue The raw value from the gyroscope.
     * @param[in] gyroFSR The full-scale range of the gyroscope in degrees per second (dps).
     * @return The calculated angular velocity in dps.
     */
    float calculateGyroDPS(int16_t gyroValue, uint8_t gyroFSR);

    /**
     * @brief Converts the temperature register value to a temperature in degrees Celsius.
     *
     * @param[in] tempValue The raw temperature register value.
     * @return The calculated temperature in degrees Celsius.
     */
    float calculateRegisterTemperature(int16_t tempValue); 

    /**
     * @brief Calculates the acceleration from the accelerometer value in units of gravity (g).
     *
     * @param[in] accelValue The raw value from the accelerometer.
     * @param[in] accelFSR The full-scale range of the accelerometer in g.
     * @return The calculated acceleration in g.
     */
    float calculateAccelGravi(int16_t accelValue, uint8_t accelFSR);

    IMUCalibrateParams parserIMUCalibrationParams(const char* data);
    IMUCalibrateParams parserIMUCalibrationParams(std::string fileStream);
    IMUCalibrateParams parserIMUCalibrationParamsRaw(uint8_t* filedata, uint32_t size);


    /**
     * @brief Corrects the accelerometer readings.
     *
     * This function takes the raw accelerometer vector and IMU calibration parameters,
     * and returns the corrected acceleration vector.
     *
     * @param[in] accelVec The raw acceleration vector.
     * @param[in] params Pointer to the IMU calibration parameters.
     * @return The corrected acceleration vector.
     */
    Eigen::Vector3d correctAccel(const Eigen::Vector3d& accelVec, IMUCalibrateParams *params);
    
    /**
     * @brief Corrects the gyroscope readings.
     *
     * This function applies calibration to the raw gyroscope vector to compensate for
     * any biases or scale factors, ensuring more accurate angular velocity measurements.
     *
     * @param[in] gyroVec The raw gyroscope vector containing angular velocity measurements.
     * @param[in] params Pointer to the IMU calibration parameters used for correction.
     * @return The corrected gyroscope vector.
     */
    Eigen::Vector3d correctGyro(const Eigen::Vector3d& gyroVec, IMUCalibrateParams *params);

}  // namespace libobsensor