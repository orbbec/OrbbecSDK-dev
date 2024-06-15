#ifndef __VCAL_IMU_CALIBRATION_PARAMS_H__
#define __VCAL_IMU_CALIBRATION_PARAMS_H__

#include <stdint.h>

#pragma pack(1)

// Orbbec Accelerometer model
typedef struct {
    double noiseDensity;          ///< Noise density
    double randomWalk;            ///< Random walk
    double referenceTemp;         ///< Reference temperature
    double bias[3];               ///< Bias
    double gravity[3];            ///< Gravity vector
    double scaleMisalignment[9];  ///< Scale and misalignment matrix
    double tempSlope[9];          ///< Temperature slope (linear thermal drift coefficient)
} Orbb_Accelerometer;

// Orbbec Gyroscope model
typedef struct {
    double noiseDensity;          ///< Noise density
    double randomWalk;            ///< Random walk
    double referenceTemp;         ///< Reference temperature
    double bias[3];               ///< Bias
    double scaleMisalignment[9];  ///< Scale and misalignment matrix
    double tempSlope[9];          ///< Temperature slope (linear thermal drift coefficient)
} Orbb_Gyroscope;

// Orbbec Magnetometer model
typedef struct {
    double referenceTemp;      ///< Reference temperature
    double tempSlope[9];       ///< Temperature slope (linear thermal drift coefficient)
    double misalignment[9];    ///< Misalignment matrix
    double softIron[9];        ///< Soft iron effect matrix
    double scale[3];           ///< Scale vector
    double hardIron[3];        ///< Hard iron bias
} Orbb_Magnetometer;

// Single IMU parameters
typedef struct {
    char name[12];             ///< IMU name
    uint16_t version;          ///< IMU calibration library version number
    uint16_t imuModel;         ///< IMU model
    double body_to_gyroscope[9]; ///< Rotation from body coordinate system to gyroscope coordinate system
    double acc_to_gyro_factor[9]; ///< Influence factor of accelerometer measurements on gyroscope measurements
    Orbb_Accelerometer acc;    ///< Accelerometer model
    Orbb_Gyroscope gyro;       ///< Gyroscope model
    Orbb_Magnetometer mag;     ///< Magnetometer model
    double timeshift_cam_to_imu;  ///< Time offset between camera and IMU
    double imu_to_cam_extrinsics[16];    ///< Extrinsic parameters from IMU to Cam(Depth)
} SingleIMUParams;

// IMU Calibration Parameters
typedef struct {
    uint8_t validNum;          ///< Number of valid IMUs
    SingleIMUParams singleIMUParams[3]; ///< Array of single IMU parameter models
} IMUCalibrateParams;


#pragma pack()
#endif //__VCAL_CALIBRATION_PARAMS_H__
