#include "imu_calibration_params_yaml.h"
#include "libcrc-2.0/checksum.h"

#include <iostream>
#include <string.h>

void loadOrbbecIMUParams(const YAML::Node root,
                         IMUCalibrateParams *params) {
    params->validNum = 1;
  //------------- ACC params -------------------
  params->singleIMUParams->acc.noiseDensity =
      root["imu0"]["accelerometer"]["noise_density"].as<double>();
  params->singleIMUParams->acc.randomWalk =
      root["imu0"]["accelerometer"]["random_walk"].as<double>();

  //------------- gravity -------------------
  std::vector<double> gravity = root["gravity"].as<std::vector<double>>();
  for (int i = 0; i < gravity.size(); i++) {
    params->singleIMUParams->acc.gravity[i] = gravity[i];
  }

  //------------- acc bias -------------------
  std::vector<double> acc_bias = root["imu0"]["AccBias"].as<std::vector<double>>();
  for (int i = 0; i < acc_bias.size(); i++) {
    params->singleIMUParams->acc.bias[i] = acc_bias[i];
  }

  //------------- gyro bias -------------------
  std::vector<double> gyro_bias = root["imu0"]["GyrBias"].as<std::vector<double>>();
  for (int i = 0; i < gyro_bias.size(); i++) {
    params->singleIMUParams->gyro.bias[i] = gyro_bias[i];
  }

  //------------- bodyToGyroscope params -------------------
  for (int j = 0; j < 3; j++) {
    std::vector<double> bodyToGyroscope =
        root["imu0"]["C_gyro_i"][j].as<std::vector<double>>();
    for (int i = 3 * j; i < bodyToGyroscope.size() + 3 * j; i++) {
      params->singleIMUParams->body_to_gyroscope[i] = bodyToGyroscope[i - 3 * j];
    }
  }

  //------------- AccToGyroFactor params -------------------
  for (int j = 0; j < 3; j++) {
    std::vector<double> AccToGyroFactor =
        root["imu0"]["Ma_gyr"][j].as<std::vector<double>>();
    for (int i = 3 * j; i < AccToGyroFactor.size() + 3 * j; i++) {
      params->singleIMUParams->acc_to_gyro_factor[i] = AccToGyroFactor[i - 3 * j];
    }
  }

  //------------- timeshiftCamToIMU params -------------------
  double timeshiftCamToIMU = root["cam0"]["timeshift_cam_imu"].as<double>();
  params->singleIMUParams->timeshift_cam_to_imu = timeshiftCamToIMU;

  //------------- Acc scaleMisalignment params -------------------
  for (int j = 0; j < 3; j++) {
    std::vector<double> scaleMisalignment =
        root["imu0"]["M_acc"][j].as<std::vector<double>>();
    for (int i = 3 * j; i < scaleMisalignment.size() + 3 * j; i++) {
      params->singleIMUParams->acc.scaleMisalignment[i] =
          scaleMisalignment[i - 3 * j];
    }
  }

  //------------- Gyro params -------------------
  params->singleIMUParams->gyro.noiseDensity =
      root["imu0"]["gyroscope"]["noise_density"].as<double>();
  params->singleIMUParams->gyro.randomWalk =
      root["imu0"]["gyroscope"]["random_walk"].as<double>();

  //------------- Gyro scaleMisalignment params -------------------
  for (int j = 0; j < 3; j++) {
    std::vector<double> scaleMisalignment =
        root["imu0"]["M_gyr"][j].as<std::vector<double>>();
    for (int i = 3 * j; i < scaleMisalignment.size() + 3 * j; i++) {
      params->singleIMUParams->gyro.scaleMisalignment[i] =
          scaleMisalignment[i - 3 * j];
    }
  }

  //------------- T_cam_imu params -------------------
  for (int j = 0; j < 4; j++) {
    std::vector<double> T_cam_imu =
        root["cam0"]["T_cam_imu"][j].as<std::vector<double>>();
    for (int i = 4 * j; i < T_cam_imu.size() + 4 * j; i++) {
      params->singleIMUParams->imu_to_cam_extrinsics[i] = T_cam_imu[i - 4 * j];
    }
  }
}
