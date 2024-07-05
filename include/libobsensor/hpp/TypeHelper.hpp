#pragma once

#include <string>
#include <iostream>
#include "libobsensor/h/ObTypes.h"
#include "libobsensor/h/TypeHelper.h"

#include <functional>

namespace ob {
class TypeHelper {
public:
    /**
     * @brief Convert OBFormat to " string " type and then return.
     *
     * @param[in] type OBFormat type.
     * @return OBFormat of "string" type.
     */
    static std::string convertOBFormatTypeToString(const OBFormat &type) {
        return ob_format_type_to_string(type);
    }

    /**
     * @brief Convert OBFrameType to " string " type and then return.
     *
     * @param[in] type OBFrameType type.
     * @return OBFrameType of "string" type.
     */
    static std::string convertOBFrameTypeToString(const OBFrameType &type) {
        return ob_frame_type_to_string(type);
    }

    /**
     * @brief Convert OBStreamType to " string " type and then return.
     *
     * @param[in] type OBStreamType type.
     * @return OBStreamType of "string" type.
     */
    static std::string convertOBStreamTypeToString(const OBStreamType &type) {
        return ob_stream_type_to_string(type);
    }

    /**
     * @brief Convert OBSensorType to " string " type and then return.
     *
     * @param[in] type OBSensorType type.
     * @return OBSensorType of "string" type.
     */
    static std::string convertOBSensorTypeToString(const OBSensorType &type) {
        return ob_sensor_type_to_string(type);
    }

    /**
     * @brief Convert OBIMUSampleRate to " string " type and then return.
     *
     * @param[in] type OBIMUSampleRate type.
     * @return OBIMUSampleRate of "string" type.
     */
    static std::string convertOBIMUSampleRateTypeToString(const OBIMUSampleRate &type) {
        return ob_imu_rate_type_to_string(type);
    }

    /**
     * @brief Convert OBGyroFullScaleRange to " string " type and then return.
     *
     * @param[in] type OBGyroFullScaleRange type.
     * @return OBGyroFullScaleRange of "string" type.
     */
    static std::string convertOBGyroFullScaleRangeTypeToString(const OBGyroFullScaleRange &type) {
        return ob_gyro_range_type_to_string(type);
    }

    /**
     * @brief Convert OBAccelFullScaleRange to " string " type and then return.
     *
     * @param[in] type OBAccelFullScaleRange type.
     * @return OBAccelFullScaleRange of "string" type.
     */
    static std::string convertOBAccelFullScaleRangeTypeToString(const OBAccelFullScaleRange &type) {
        return ob_accel_range_type_to_string(type);
    }

    /**
     * @brief Convert OBFrameMetadataType to " string " type and then return.
     *
     * @param[in] type OBFrameMetadataType type.
     * @return OBFrameMetadataType of "string" type.
     */
    static std::string convertOBFrameMetadataTypeToString(const OBFrameMetadataType &type) {
        return ob_meta_data_type_to_string(type);
    }

    /**
     * @brief Convert OBSensorType to OBStreamType type and then return.
     *
     * @param[in] type OBSensorType type.
     * @return OBStreamType type.
     */
    static OBStreamType convertSensorTypeToStreamType(OBSensorType type) {
        return ob_sensor_type_to_stream_type(type);
    }
};
}  // namespace ob
