#include "ImplTypes.hpp"
#include "utils/Utils.hpp"
#include "exception/ObException.hpp"

#include "libobsensor/h/TypeHelper.h"

const char* ob_format_type_to_string(OBFormat type){
    return libobsensor::utils::obFormatToStr(type).c_str();
}

const char* ob_frame_type_to_string(OBFrameType type){
    return libobsensor::utils::obFrameToStr(type).c_str();
}

const char* ob_stream_type_to_string(OBStreamType type){
    return libobsensor::utils::obStreamToStr(type).c_str();
}

const char* ob_sensor_type_to_string(OBSensorType type){
    return libobsensor::utils::obSensorToStr(type).c_str();
}


const char* ob_imu_rate_type_to_string(OBIMUSampleRate type){
    return libobsensor::utils::obImuRateToStr(type).c_str();
}

const char* ob_gyro_range_type_to_string(OBGyroFullScaleRange type){
    return libobsensor::utils::GyroFullScaleRangeToStr(type).c_str();
}

const char* ob_accel_range_type_to_string(OBAccelFullScaleRange type){
    return libobsensor::utils::AccelFullScaleRangeToStr(type).c_str();
}