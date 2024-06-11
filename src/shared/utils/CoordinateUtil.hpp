#pragma once
#include "openobsdk/h/ObTypes.h"
//#include "core/device/IDevice.hpp"
//#include "core/frame/Frame.hpp"
#include <memory>
#include <cmath>

namespace libobsensor {
#define EPS 1e-4

class CoordinateUtil {
public:
    static bool calibration3dTo3d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    static bool calibration2dTo3d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    static bool calibration2dTo3dUndistortion(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                              const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f);

    static bool calibration3dTo2d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                  const OBSensorType targetSensorType, OBPoint2f *targetPoint2f);

    static bool calibration2dTo2d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                  const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint2f *targetPoint2f);

    /*static std::shared_ptr<Frame> transformationDepthFrameToColorCamera(std::shared_ptr<IDevice> device, std::shared_ptr<Frame> depthFrame,
                                                                        uint32_t targetColorCameraWidth, uint32_t targetColorCameraHeight);*/

    // data 的内存需要在外部申请，在初始化的时候，外部申请的大小是 data_size，例如datasize = 1920*1080 *2 (1920*1080表示图像分辨率，2表示有两个lut，
    // 分别为x和y的坐标)
    static bool transformationInitXYTables(const OBCalibrationParam calibrationParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                           OBXYTables *xyTables);

    static bool transformationInitXYTables(const OBCameraParam cameraParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                           OBXYTables *xyTables);

    // static bool transformationInitAddDistortionUVTables(const OBCameraParam cameraParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
    //                                                     OBXYTables *uvTables);
    static bool transformationInitAddDistortionUVTables(const OBCameraParam cameraParam, float *data, uint32_t *dataSize,
                                                        OBXYTables *uvTables);

    static void transformationDepthToPointCloud(OBXYTables *xyTables, const void *depthImageData, void *pointCloudData, float positionDataScale = 1.0f,
                                                OBCoordinateSystemType type = OB_RIGHT_HAND_COORDINATE_SYSTEM);

    static void transformationDepthToRGBDPointCloud(OBXYTables *xyTables, const void *depthImageData, const void *colorImageData, void *pointCloudData,
                                                    float positionDataScale = 1.0f, OBCoordinateSystemType type = OB_RIGHT_HAND_COORDINATE_SYSTEM,
                                                    bool colorDataNormalization = false);

    static void transformationDepthToRGBDPointCloudByUVTables(const OBCameraParam cameraParam, OBXYTables *uvTables, const void *depthImageData,
                                                              const void *colorImageData, void *pointCloudData, float positionDataScale = 1.0f,
                                                              OBCoordinateSystemType type                   = OB_RIGHT_HAND_COORDINATE_SYSTEM,
                                                              bool                   colorDataNormalization = false);
};
}  // namespace libobsensor