#pragma once

#include <string>
#include <iostream>
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/TypeHelper.h"

#include <functional>

/**
 * @brief Convert OBFormat to " char* " type and then return.
 *
 * @param[in] type OBFormat type.
 * @return OBFormat of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBFormat &type) {
    os << ob_format_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBFrameType to " char* " type and then return.
 *
 * @param[in] type OBFrameType type.
 * @return OBFrameType of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBFrameType &type) {
    os << ob_frame_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBStreamType to " char* " type and then return.
 *
 * @param[in] type OBStreamType type.
 * @return OBStreamType of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBStreamType &type) {
    os << ob_stream_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBSensorType to " char* " type and then return.
 *
 * @param[in] type OBSensorType type.
 * @return OBSensorType of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBSensorType &type) {
    os << ob_sensor_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBIMUSampleRate to " char* " type and then return.
 *
 * @param[in] type OBIMUSampleRate type.
 * @return OBIMUSampleRate of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBIMUSampleRate &type) {
    os << ob_imu_rate_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBGyroFullScaleRange to " char* " type and then return.
 *
 * @param[in] type OBGyroFullScaleRange type.
 * @return OBGyroFullScaleRange of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBGyroFullScaleRange &type) {
    os << ob_gyro_range_type_to_string(type);
    return os;
}

/**
 * @brief Convert OBAccelFullScaleRange to " char* " type and then return.
 *
 * @param[in] type OBAccelFullScaleRange type.
 * @return OBAccelFullScaleRange of "char*" type.
 */
std::ostream &operator<<(std::ostream &os, const OBAccelFullScaleRange &type) {
    os << ob_accel_range_type_to_string(type);
    return os;
}