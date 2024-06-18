/*****************************************************************************
 *  Orbbec Vision Core Library
 *  Copyright (C) 2017 by ORBBEC Technology., Inc.
 *
 *  This file is part of Orbbec Vision Core Library 2.0
 *
 *  This file belongs to ORBBEC Technology., Inc.
 *  It is considered a trade secret, and is not to be divulged or used by
 * parties who have NOT received written authorization from the owner.
 *
 *  Description
 ****************************************************************************/

#ifndef __VCAL_CALIBRATION_PARAMS_PARSER_H__
#define __VCAL_CALIBRATION_PARAMS_PARSER_H__

#include "imu_calibration_params.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#ifdef VCAL_EXPORTS
#define VCAL_API __declspec(dllexport)
#else
#define VCAL_API __declspec(dllimport)
#endif
#else
#define VCAL_API
#endif

    /**
     * @brief Enumeration for IMU parameter file formats.
     */
    typedef enum {
        IMU_CALIB_PARAM_FORMAT_YAML = 0, /**< YAML format */
    } IMUParamsFormatE;

    /**
     * @brief Enumeration for IMU parameter parsing error codes.
     */
    typedef enum {
        IMU_PARAM_PARSE_OK = 0,          /**< Parsing succeeded */
        IMU_PARAM_PARSE_INVALID_ARG = 4096, /**< Invalid input argument */
        IMU_PARAM_PARSE_CORRUPT = 4097,  /**< Corrupt data */
        IMU_PARAM_PARSE_NOSUPP = 4098,   /**< Unsupported feature */
    } IMUParamParseErr;

    /**
     * @brief Loads IMU calibration parameters from a file.
     * @param[in] fname The file name.
     * @param[in] params Pointer to the IMU calibration parameters structure.
     * @param[in] format The file format enumeration value.
     * @return An error code indicating the result of the operation.
     */
    VCAL_API IMUParamParseErr loadCalibrationIMUParamsMulti(const char* fname,
        IMUCalibrateParams* params,
        IMUParamsFormatE format);

    /**
     * @brief Loads IMU calibration parameters from raw data.
     * @param[in] data The raw data string.
     * @param[in] params Pointer to the IMU calibration parameters structure.
     * @param[in] format The file format enumeration value.
     * @return An error code indicating the result of the operation.
     */
    VCAL_API IMUParamParseErr loadCalibrationIMUParamsMultiFromData(const char* data,
        IMUCalibrateParams* params,
        IMUParamsFormatE format);

    /**
     * @brief Retrieves the version information of the IMU parameters parser.
     * @param[out] version_info The array to store version information string.
     * @return Boolean value indicating whether version information was successfully retrieved.
     */
    VCAL_API bool getVersionIMUParamsParser(char version_info[]);

#ifdef __cplusplus
}
#endif

#endif //__VCAL_CALIBRATION_PARAMS_PARSER_H__
