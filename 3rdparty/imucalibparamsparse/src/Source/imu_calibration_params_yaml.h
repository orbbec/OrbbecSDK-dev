#ifndef __VCAL_CALIBRATION_PARAMS_YAML_H__
#define __VCAL_CALIBRATION_PARAMS_YAML_H__

#include "imu_calibration_params.h"
#include <yaml-cpp/yaml.h>

void loadOrbbecIMUParams(const YAML::Node root, IMUCalibrateParams* params);
#endif //__VCAL_CALIBRATION_PARAMS_YAML_H__
