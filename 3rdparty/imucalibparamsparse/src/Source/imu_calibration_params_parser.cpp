#include "imu_calibration_params_parser.h"
#include "imu_calibration_params_yaml.h"

#include "version.h"

#include <errno.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace std;

static IMUParamParseErr
loadCalibrationIMUParamsYaml(const char* fname,
    IMUCalibrateParams* params)
{
    YAML::Node reader;
    YAML::Node root;
    std::ifstream ifs;
    ifs.open(fname);
    if (!ifs.is_open()) {
        return (IMUParamParseErr)errno;
    }

    reader = YAML::LoadFile(fname);
    root = reader;
    if (!reader.IsNull()) {
        loadOrbbecIMUParams(root, params);
    } else
        return IMU_PARAM_PARSE_CORRUPT;
    return IMU_PARAM_PARSE_OK;
}

static IMUParamParseErr loadCalibrationIMUParamsYamlFromData(const char* data, IMUCalibrateParams* params) {
    YAML::Node reader;
    YAML::Node root;

    reader = YAML::Load(data);
    root = reader;
    if (!reader.IsNull()) {
        loadOrbbecIMUParams(root, params);
    }
    else
        return IMU_PARAM_PARSE_CORRUPT;
    return IMU_PARAM_PARSE_OK;
}


IMUParamParseErr loadCalibrationIMUParamsMulti(const char* fname,
    IMUCalibrateParams* params,
    IMUParamsFormatE format)
{
    if (nullptr == fname)
        return IMU_PARAM_PARSE_INVALID_ARG;

    if (nullptr == params)
        return IMU_PARAM_PARSE_INVALID_ARG;

    if (IMU_CALIB_PARAM_FORMAT_YAML == format)
        return loadCalibrationIMUParamsYaml(fname, params);

    else
        return IMU_PARAM_PARSE_NOSUPP;
}

IMUParamParseErr loadCalibrationIMUParamsMultiFromData(const char* data, IMUCalibrateParams* params, IMUParamsFormatE format) {
    if (nullptr == data)
        return IMU_PARAM_PARSE_INVALID_ARG;

    if (nullptr == params)
        return IMU_PARAM_PARSE_INVALID_ARG;

    if (IMU_CALIB_PARAM_FORMAT_YAML == format)
        return loadCalibrationIMUParamsYamlFromData(data, params);

    else
        return IMU_PARAM_PARSE_NOSUPP;
}

bool getVersionIMUParamsParser(char version_info[])
{
    std::string ver = std::string(CALIB_PARAMS_PARSER_VERSION);
    strcpy(version_info, ver.c_str());

    return true;
}
