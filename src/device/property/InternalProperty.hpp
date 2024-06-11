#pragma once
#include "openobsdk/h/Property.h"

namespace libobsensor {

typedef enum {
    OB_PROP_GYRO_SWITCH_BOOL     = 2019, /**< Gyroscope switch*/
    OB_PROP_ACCEL_SWITCH_BOOL    = 2020, /**< Accelerometer switch*/
    OB_PROP_GYRO_ODR_INT         = 2021, /**< get/set current gyroscope sampling rate*/
    OB_PROP_ACCEL_ODR_INT        = 2022, /**< get/set the current sampling rate of the accelerometer*/
    OB_PROP_GYRO_FULL_SCALE_INT  = 2023, /**< get/set current gyroscope range*/
    OB_PROP_ACCEL_FULL_SCALE_INT = 2024, /**< get/set current accelerometer range*/

    OB_STRUCT_VERSION                           = 1000, /**< version information*/
    OB_STRUCT_GET_GYRO_PRESETS_ODR_LIST         = 1031, /**< Get the list of sampling rates supported by the gyroscope*/
    OB_STRUCT_GET_ACCEL_PRESETS_ODR_LIST        = 1032, /**< Get the list of sampling rates supported by the accelerometer*/
    OB_STRUCT_GET_GYRO_PRESETS_FULL_SCALE_LIST  = 1033, /**< Get the range list supported by the gyroscope*/
    OB_STRUCT_GET_ACCEL_PRESETS_FULL_SCALE_LIST = 1034, /**< Get the range list supported by the accelerometer*/
} OBInternalPropertyID;

}  // namespace libobsensor
