#include "CoordinateUtil.hpp"
#include "logger/Logger.hpp"
#include "logger/LoggerInterval.hpp"
//#include "core/device/AbstractDevice.hpp"
//#include "core/frame/process/advance/D2CFilter.hpp"
//#include "core/frame/process/advance/PostProcessFilter.hpp"

#include <cmath>

namespace libobsensor {
static bool judgeTransformValid(OBCameraParam cameraParam) {
    // 旋转矩阵的正交性
    // r1 .* r2 = 0 ;
    float r1r2 = cameraParam.transform.rot[0] * cameraParam.transform.rot[3] + cameraParam.transform.rot[1] * cameraParam.transform.rot[4]
                 + cameraParam.transform.rot[2] * cameraParam.transform.rot[5];

    float r1r3 = cameraParam.transform.rot[0] * cameraParam.transform.rot[6] + cameraParam.transform.rot[1] * cameraParam.transform.rot[7]
                 + cameraParam.transform.rot[2] * cameraParam.transform.rot[8];

    if(fabsf(r1r2) < EPS && fabs(r1r3) < EPS) {
        return true;
    }
    return false;
}

static void undistortIterativeUnproject(OBCameraIntrinsic K, OBCameraDistortion disto, float x_pixel, float y_pixel, float *xn, float *yn, int *valid) {
    double r2, r4, r6;
    double x = 0;
    double y = 0;
    double kr_inv, xd, yd, dx, dy;
    xd = (x_pixel - K.cx) / K.fx;
    yd = (y_pixel - K.cy) / K.fy;

    x               = xd;
    y               = yd;
    double best_err = 99999;
    *valid          = 1;

    for(int i = 0; i < 20; i++) {
        // 迭代去畸变
        r2     = x * x + y * y;
        r4     = r2 * r2;
        r6     = r4 * r2;
        kr_inv = (1 + disto.k4 * r2 + disto.k5 * r4 + disto.k6 * r6) / (1 + disto.k1 * r2 + disto.k2 * r4 + disto.k3 * r6);
        dx     = disto.p1 * 2 * x * y + disto.p2 * (r2 + 2 * x * x);
        dy     = disto.p2 * 2 * x * y + disto.p1 * (r2 + 2 * y * y);
        x      = (xd - dx) * kr_inv;
        y      = (yd - dy) * kr_inv;

        // 无畸变点进行加畸变，
        // double a1, a2, a3, cdist, icdist2;
        double a1, a2, a3, cdist;
        double xd0, yd0;
        r2            = x * x + y * y;
        r4            = r2 * r2;
        r6            = r4 * r2;
        a1            = 2 * x * y;
        a2            = r2 + 2 * x * x;
        a3            = r2 + 2 * y * y;
        cdist         = (1 + disto.k1 * r2 + disto.k2 * r4 + disto.k3 * r6) / (1 + disto.k4 * r2 + disto.k5 * r4 + disto.k6 * r6);
        xd0           = x * cdist + disto.p1 * a1 + disto.p2 * a2;
        yd0           = y * cdist + disto.p1 * a3 + disto.p2 * a1;
        double x_proj = xd0 * K.fx + K.cx;
        double y_proj = yd0 * K.fy + K.cy;
        double error  = sqrt(pow(x_proj - x_pixel, 2) + pow(y_proj - y_pixel, 2));

        if(error > best_err)
            break;

        best_err = error;

        if(error < 0.01) {
            break;
        }
    }

    if(best_err > 0.1) {
        *valid = 0;
        *xn    = (float)xd;
        *yn    = (float)yd;
        return;
    }

    *xn = (float)x, *yn = (float)y;
}

bool CoordinateUtil::calibration3dTo3d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                       const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
    OBCameraParam cameraParam;
    memcpy(&cameraParam.rgbIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_COLOR], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.depthIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_DEPTH], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.transform, &calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], sizeof(OBD2CTransform));

    // step 1 ： 数据有效性判断
    if(!judgeTransformValid(cameraParam)) {
        return false;
    }

    if(fabsf(sourcePoint3f.z) < EPS) {
        return false;
    }

    // step 2: 计算转换关系
    if(sourceSensorType == targetSensorType)  // 源相机与目标相机为同一相机
    {
        *targetPoint3f = sourcePoint3f;
    }
    else if((OB_SENSOR_DEPTH == sourceSensorType) && (OB_SENSOR_COLOR == targetSensorType))  // 从DEPTH 到rgb的转换
    {
        // R * X + t
        float rx =
            cameraParam.transform.rot[0] * sourcePoint3f.x + cameraParam.transform.rot[1] * sourcePoint3f.y + cameraParam.transform.rot[2] * sourcePoint3f.z;
        float ry =
            cameraParam.transform.rot[3] * sourcePoint3f.x + cameraParam.transform.rot[4] * sourcePoint3f.y + cameraParam.transform.rot[5] * sourcePoint3f.z;
        float rz =
            cameraParam.transform.rot[6] * sourcePoint3f.x + cameraParam.transform.rot[7] * sourcePoint3f.y + cameraParam.transform.rot[8] * sourcePoint3f.z;

        (*targetPoint3f).x = rx + cameraParam.transform.trans[0];
        (*targetPoint3f).y = ry + cameraParam.transform.trans[1];
        (*targetPoint3f).z = rz + cameraParam.transform.trans[2];
    }
    else if((OB_SENSOR_COLOR == sourceSensorType) && (OB_SENSOR_DEPTH == targetSensorType))  // 从rgb到DEPTH的转换
    {
        // R_inv *(X-b)
        float tx = sourcePoint3f.x - cameraParam.transform.trans[0];
        float ty = sourcePoint3f.y - cameraParam.transform.trans[1];
        float tz = sourcePoint3f.z - cameraParam.transform.trans[2];

        (*targetPoint3f).x = cameraParam.transform.rot[0] * tx + cameraParam.transform.rot[3] * ty + cameraParam.transform.rot[6] * tz;
        (*targetPoint3f).y = cameraParam.transform.rot[1] * tx + cameraParam.transform.rot[4] * ty + cameraParam.transform.rot[7] * tz;
        (*targetPoint3f).z = cameraParam.transform.rot[2] * tx + cameraParam.transform.rot[5] * ty + cameraParam.transform.rot[8] * tz;
    }
    else {
        return false;
    }

    return true;
}

bool CoordinateUtil::calibration2dTo3d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                       const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
    OBCameraParam cameraParam;
    memcpy(&cameraParam.rgbIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_COLOR], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.depthIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_DEPTH], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.transform, &calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], sizeof(OBD2CTransform));

    // step 1: 判断有效性
    // sourcePoint2f 为图像的像素坐标，或者亚像素坐标
    if(sourcePoint2f.x < 0 || sourcePoint2f.y < 0) {
        return false;
    }

    if(OB_SENSOR_DEPTH == sourceSensorType) {
        if(sourcePoint2f.x > (cameraParam.depthIntrinsic.width - 1) || sourcePoint2f.y > ((cameraParam.depthIntrinsic.height - 1))) {
            return false;
        }
    }

    if(OB_SENSOR_COLOR == sourceSensorType) {
        if(sourcePoint2f.x > (cameraParam.rgbIntrinsic.width - 1) || sourcePoint2f.y > ((cameraParam.rgbIntrinsic.height - 1))) {
            return false;
        }
    }

    // step 2: 将2D 转3d点
    OBPoint3f source_3f;
    source_3f.z = sourceDepthPixelValue;  // z方向赋值
    // 2d 转3d （和转点云一样）
    if(OB_SENSOR_DEPTH == sourceSensorType) {
        source_3f.x = sourceDepthPixelValue * (sourcePoint2f.x - cameraParam.depthIntrinsic.cx) / cameraParam.depthIntrinsic.fx;
        source_3f.y = sourceDepthPixelValue * (sourcePoint2f.y - cameraParam.depthIntrinsic.cy) / cameraParam.depthIntrinsic.fy;
    }
    else if(OB_SENSOR_COLOR == sourceSensorType) {
        source_3f.x = sourceDepthPixelValue * (sourcePoint2f.x - cameraParam.rgbIntrinsic.cx) / cameraParam.rgbIntrinsic.fx;
        source_3f.y = sourceDepthPixelValue * (sourcePoint2f.y - cameraParam.rgbIntrinsic.cy) / cameraParam.rgbIntrinsic.fy;
    }
    else  // 2d 点仅存在depth 或者rgb 上
    {
        return false;
    }

    // step 3: 将source 下的3d点转换到target相机坐标下
    bool ret = calibration3dTo3d(calibrationParam, source_3f, sourceSensorType, targetSensorType, targetPoint3f);

    return ret;
}

bool CoordinateUtil::calibration2dTo3dUndistortion(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                                   const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint3f *targetPoint3f) {
    OBCameraParam cameraParam;
    memcpy(&cameraParam.rgbIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_COLOR], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.depthIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_DEPTH], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.rgbDistortion, &calibrationParam.distortion[OB_SENSOR_COLOR], sizeof(OBCameraDistortion));
    memcpy(&cameraParam.depthDistortion, &calibrationParam.distortion[OB_SENSOR_DEPTH], sizeof(OBCameraDistortion));
    memcpy(&cameraParam.transform, &calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], sizeof(OBD2CTransform));

    // step 1: 判断有效性
    // sourcePoint2f 为图像的像素坐标，或者亚像素坐标
    if(sourcePoint2f.x < 0 || sourcePoint2f.y < 0) {
        return false;
    }

    if(OB_SENSOR_DEPTH == sourceSensorType) {
        if(sourcePoint2f.x > (cameraParam.depthIntrinsic.width - 1) || sourcePoint2f.y > ((cameraParam.depthIntrinsic.height - 1))) {
            return false;
        }
    }

    if(OB_SENSOR_COLOR == sourceSensorType) {
        if(sourcePoint2f.x > (cameraParam.rgbIntrinsic.width - 1) || sourcePoint2f.y > ((cameraParam.rgbIntrinsic.height - 1))) {
            return false;
        }
    }

    // step 2: 将2D 转3d点
    OBPoint3f source_3f;
    source_3f.z = sourceDepthPixelValue;  // z方向赋值
    int valid   = 0;
    // 2d 转3d （和转点云一样）
    if(OB_SENSOR_DEPTH == sourceSensorType) {
        undistortIterativeUnproject(cameraParam.depthIntrinsic, cameraParam.depthDistortion, sourcePoint2f.x, sourcePoint2f.y, &(source_3f.x), &(source_3f.y),
                                    &valid);
    }
    else if(OB_SENSOR_COLOR == sourceSensorType) {
        undistortIterativeUnproject(cameraParam.rgbIntrinsic, cameraParam.rgbDistortion, sourcePoint2f.x, sourcePoint2f.y, &(source_3f.x), &(source_3f.y),
                                    &valid);
    }
    else  // 2d 点仅存在depth 或者rgb 上
    {
        return false;
    }

    if(valid == 0) {
        return false;
    }

    source_3f.x *= source_3f.z;
    source_3f.y *= source_3f.z;
    // step 3: 将source 下的3d点转换到target相机坐标下
    bool ret = calibration3dTo3d(calibrationParam, source_3f, sourceSensorType, targetSensorType, targetPoint3f);

    return ret;
}

bool CoordinateUtil::calibration3dTo2d(const OBCalibrationParam calibrationParam, const OBPoint3f sourcePoint3f, const OBSensorType sourceSensorType,
                                       const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
    OBCameraParam cameraParam;
    memcpy(&cameraParam.rgbIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_COLOR], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.depthIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_DEPTH], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.transform, &calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], sizeof(OBD2CTransform));

    // step1: 将source 下的3d点转换到target相机坐标下
    OBPoint3f targetPoint3f;
    bool      ret = calibration3dTo3d(calibrationParam, sourcePoint3f, sourceSensorType, targetSensorType, &targetPoint3f);
    if(!ret) {
        return ret;
    }

    // step 2： 将target相机坐标下的3d点转2d点 (小孔成像的原理)
    if(OB_SENSOR_DEPTH == targetSensorType) {
        (*targetPoint2f).x = cameraParam.depthIntrinsic.fx * targetPoint3f.x / targetPoint3f.z + cameraParam.depthIntrinsic.cx;
        (*targetPoint2f).y = cameraParam.depthIntrinsic.fy * targetPoint3f.y / targetPoint3f.z + cameraParam.depthIntrinsic.cy;
        ret = ((*targetPoint2f).x > (cameraParam.depthIntrinsic.width - 1) || (*targetPoint2f).y > (cameraParam.depthIntrinsic.height - 1)) ? false : true;
    }
    else if(OB_SENSOR_COLOR == targetSensorType) {
        (*targetPoint2f).x = cameraParam.rgbIntrinsic.fx * targetPoint3f.x / targetPoint3f.z + cameraParam.rgbIntrinsic.cx;
        (*targetPoint2f).y = cameraParam.rgbIntrinsic.fy * targetPoint3f.y / targetPoint3f.z + cameraParam.rgbIntrinsic.cy;
        ret = ((*targetPoint2f).x > (cameraParam.rgbIntrinsic.width - 1) || (*targetPoint2f).y > (cameraParam.rgbIntrinsic.height - 1)) ? false : true;
    }
    else  // 2d 点仅存在depth 或者rgb 上
    {
        return false;
    }

    ret = ((*targetPoint2f).x < 0 || (*targetPoint2f).y < 0) ? false : true;

    return ret;
}

bool CoordinateUtil::calibration2dTo2d(const OBCalibrationParam calibrationParam, const OBPoint2f sourcePoint2f, const float sourceDepthPixelValue,
                                       const OBSensorType sourceSensorType, const OBSensorType targetSensorType, OBPoint2f *targetPoint2f) {
    OBCameraParam cameraParam;
    memcpy(&cameraParam.rgbIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_COLOR], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.depthIntrinsic, &calibrationParam.intrinsics[OB_SENSOR_DEPTH], sizeof(OBCameraIntrinsic));
    memcpy(&cameraParam.transform, &calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], sizeof(OBD2CTransform));

    // step 1: 先将source的2d转成target下的 3d
    OBPoint3f targetPoint3f;
    bool      ret = calibration2dTo3dUndistortion(calibrationParam, sourcePoint2f, sourceDepthPixelValue, sourceSensorType, targetSensorType, &targetPoint3f);
    if(!ret) {
        return ret;
    }

    // step 2 ：将target相机坐标下的3d点转图像下的2d点 (小孔成像的原理)
    if(OB_SENSOR_DEPTH == targetSensorType) {
        (*targetPoint2f).x = cameraParam.depthIntrinsic.fx * targetPoint3f.x / targetPoint3f.z + cameraParam.depthIntrinsic.cx;
        (*targetPoint2f).y = cameraParam.depthIntrinsic.fy * targetPoint3f.y / targetPoint3f.z + cameraParam.depthIntrinsic.cy;
        ret = ((*targetPoint2f).x > (cameraParam.depthIntrinsic.width - 1) || (*targetPoint2f).y > (cameraParam.depthIntrinsic.height - 1)) ? false : true;
    }
    else if(OB_SENSOR_COLOR == targetSensorType) {
        (*targetPoint2f).x = cameraParam.rgbIntrinsic.fx * targetPoint3f.x / targetPoint3f.z + cameraParam.rgbIntrinsic.cx;
        (*targetPoint2f).y = cameraParam.rgbIntrinsic.fy * targetPoint3f.y / targetPoint3f.z + cameraParam.rgbIntrinsic.cy;
        ret = ((*targetPoint2f).x > (cameraParam.rgbIntrinsic.width - 1) || (*targetPoint2f).y > (cameraParam.rgbIntrinsic.height - 1)) ? false : true;
    }
    else  // 2d 点仅存在depth 或者rgb 上
    {
        return false;
    }

    ret = ((*targetPoint2f).x < 0 || (*targetPoint2f).y < 0) ? false : true;

    return ret;
}

//std::shared_ptr<Frame> CoordinateUtil::transformationDepthFrameToColorCamera(std::shared_ptr<IDevice> device, std::shared_ptr<Frame> depthFrame,
//                                                                             uint32_t targetColorCameraWidth, uint32_t targetColorCameraHeight) {
//    auto absDevice = std::dynamic_pointer_cast<AbstractDevice>(device);
//    if(absDevice == nullptr) {
//        throw std::runtime_error("Device is invalid!");
//    }
//
//    auto cameraParamList = absDevice->getCalibrationCameraParamList();
//    if(cameraParamList.empty()) {
//        LOG_WARN("Get calibration param failed,cameraParamList is empty!");
//        return nullptr;
//    }
//
//    auto profileInfo =
//        absDevice->getSupportedProfileInfo(targetColorCameraWidth, targetColorCameraHeight, depthFrame->getWidth(), depthFrame->getHeight(), ALIGN_D2C_SW_MODE);
//    if(!profileInfo.valid()) {
//        LOG_ERROR("Input invalid");
//        return nullptr;
//    }
//
//    auto cameraParam = cameraParamList[profileInfo.paramIndex];
//    auto depthWidth = depthFrame->getWidth();
//    auto depthHeight = depthFrame->getHeight();
//    // 由于模组存储的默认参数与当前图像状态可能对不上，例如MX6600项目参数是镜像状态下标定的，但是输出的图像是非镜像的，因此需要对参数进行处理。
//    cameraParam = absDevice->preProcessCameraParam(cameraParam);
//    // soft D2C filter
//    auto d2cFilter = std::make_shared<D2CFilter>();
//    d2cFilter->setCameraParam(cameraParam);
//    OBD2CAlignParam alignParam = absDevice->getD2CAlignParam();
//    d2cFilter->setD2CAlignParam(&alignParam);
//    d2cFilter->setColorSize(targetColorCameraWidth, targetColorCameraHeight);
//
//    // post process filter
//    auto postProcessFilter = std::make_shared<PostProcessFilter>();
//    postProcessFilter->setPostProcessParam(profileInfo.postProcessParam);
//    postProcessFilter->setDepthScaleRequire(true);
//    postProcessFilter->setDepthSize(depthWidth, depthHeight);
//
//    depthFrame = d2cFilter->process(depthFrame);
//    if(depthFrame == nullptr) {
//        LOG_ERROR("D2C process failed.");
//        return nullptr;
//    }
//
//    bool isParamTransform = absDevice->getCameraParamTransformState();
//    cameraParam = absDevice->transformCameraParamToD2CDstParam(cameraParam, profileInfo.postProcessParam, targetColorCameraWidth, targetColorCameraHeight, depthWidth, depthHeight);
//    cameraParam = CameraParamProcessor::postProcessCameraParam(profileInfo.postProcessParam, cameraParam, isParamTransform, ALIGN_D2C_SW_MODE,targetColorCameraWidth, targetColorCameraHeight, depthWidth, depthHeight, true);
//    postProcessFilter->setTargetIntrinsic(cameraParam.depthIntrinsic);
//
//    depthFrame = postProcessFilter->process(depthFrame);
//    if(depthFrame == nullptr) {
//        LOG_ERROR("Post process failed.");
//        return nullptr;
//    }
//
//    return depthFrame;
//}

bool CoordinateUtil::transformationInitXYTables(const OBCalibrationParam calibrationParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                                OBXYTables *xyTables) {
    int width  = 0;
    int height = 0;
    switch(sensorType) {
    case OB_SENSOR_DEPTH:
        width  = calibrationParam.intrinsics[OB_SENSOR_DEPTH].width;
        height = calibrationParam.intrinsics[OB_SENSOR_DEPTH].height;
        break;
    case OB_SENSOR_COLOR:
        width  = calibrationParam.intrinsics[OB_SENSOR_COLOR].width;
        height = calibrationParam.intrinsics[OB_SENSOR_COLOR].height;
        break;
    default:
        return false;
    }

    size_t tableSize = (size_t)(width * height);
    if(data == NULL)  // 如果外部没有申请内存，就报错
    {
        (*dataSize) = 2 * (uint32_t)tableSize;
        return false;
    }
    else {
        // 如果外部申请的内存大小不够，就报错
        if(*dataSize < 2 * tableSize) {
            LOG_ERROR("Unexpected xy table size {}, should be larger or equal than {}.", int(*dataSize), int(2 * tableSize));
            return false;
        }

        xyTables->width  = width;
        xyTables->height = height;
        xyTables->xTable = data;
        xyTables->yTable = data + tableSize;

        OBPoint2f point2d;
        OBPoint3f point3d;
        //int       valid = 1;

        for(int y = 0, idx = 0; y < height; y++) {
            point2d.y = (float)y;
            for(int x = 0; x < width; x++, idx++) {
                point2d.x = (float)x;

                if(!calibration2dTo3dUndistortion(calibrationParam, point2d, 1.0, sensorType, sensorType, &point3d)) {
                    // x table value of NAN marks invalid
                    xyTables->xTable[idx] = NAN;
                    // set y table value to 0 to speed up SSE implementation
                    xyTables->yTable[idx] = 0.f;
                }
                else {
                    xyTables->xTable[idx] = point3d.x;
                    xyTables->yTable[idx] = point3d.y;
                }
            }
        }
        (*dataSize) = 2 * (uint32_t)tableSize;
    }
    return true;
}

bool CoordinateUtil::transformationInitXYTables(const OBCameraParam cameraParam, const OBSensorType sensorType, float *data, uint32_t *dataSize,
                                                OBXYTables *xyTables) {
    OBCalibrationParam calibrationParam = {};
    memcpy(&calibrationParam.intrinsics[OB_SENSOR_COLOR], &cameraParam.rgbIntrinsic, sizeof(OBCameraIntrinsic));
    memcpy(&calibrationParam.intrinsics[OB_SENSOR_DEPTH], &cameraParam.depthIntrinsic, sizeof(OBCameraIntrinsic));
    memcpy(&calibrationParam.distortion[OB_SENSOR_COLOR], &cameraParam.rgbDistortion, sizeof(OBCameraDistortion));
    memcpy(&calibrationParam.distortion[OB_SENSOR_DEPTH], &cameraParam.depthDistortion, sizeof(OBCameraDistortion));
    memcpy(&calibrationParam.extrinsics[OB_SENSOR_DEPTH][OB_SENSOR_COLOR], &cameraParam.transform, sizeof(OBTransform));
    return transformationInitXYTables(calibrationParam, sensorType, data, dataSize, xyTables);
}

bool CoordinateUtil::transformationInitAddDistortionUVTables(const OBCameraParam cameraParam, float *data, uint32_t *dataSize,
                                                             OBXYTables *uvTables) {
    OBCameraDistortion disto     = cameraParam.rgbDistortion;
    OBCameraIntrinsic  intrinsic = cameraParam.rgbIntrinsic;
    int                width     = intrinsic.width;
    int                height    = intrinsic.height;

    size_t tableSize = (size_t)(width * height);
    if(data == NULL)  // 如果外部没有申请内存，就报错
    {
        (*dataSize) = 2 * (uint32_t)tableSize;
        return false;
    }
    else {
        // 如果外部申请的内存大小不够，就报错
        if(*dataSize < 2 * tableSize) {
            LOG_ERROR("Unexpected xy table size {}, should be larger or equal than {}.", int(*dataSize), int(2 * tableSize));
            return false;
        }

        // data 与 ob_xy_tables_t 共用内存
        uvTables->width  = width;
        uvTables->height = height;
        uvTables->xTable = data;
        uvTables->yTable = data + tableSize;


        for(int row = 0, idx = 0; row < height; row++) {
            double y = (double)(row - intrinsic.cy) / intrinsic.fy;
            for(int col = 0; col < width; col++, idx++) {
                double x = (double)(col - intrinsic.cx) / intrinsic.fx;

                // 加畸变，仅支持布朗模型，k2,k3,k6模型， 不支持KB
                // double a1, a2, a3, cdist, icdist2;
                double a1, a2, a3, cdist;
                double xd, yd;
                double r2, r4, r6;
                r2           = x * x + y * y;
                r4           = r2 * r2;
                r6           = r4 * r2;
                a1           = 2 * x * y;
                a2           = r2 + 2 * x * x;
                a3           = r2 + 2 * y * y;
                cdist        = (1 + disto.k1 * r2 + disto.k2 * r4 + disto.k3 * r6) / (1 + disto.k4 * r2 + disto.k5 * r4 + disto.k6 * r6);
                xd           = x * cdist + disto.p1 * a1 + disto.p2 * a2;
                yd           = y * cdist + disto.p1 * a3 + disto.p2 * a1;
                float x_proj = (float)xd * intrinsic.fx + intrinsic.cx;
                float y_proj = (float)yd * intrinsic.fy + intrinsic.cy;

                // 判断加畸变后的点，是否超过图像范围，如果超过，这个点无效
                if(x_proj < 0 || x_proj > (width - 1) || y_proj < 0 || y_proj > (height - 1)) {
                    // 无效数据输出
                    // x table value of NAN marks invalid
                    uvTables->xTable[idx] = NAN;
                    // set y table value to 0 to speed up SSE implementation
                    uvTables->yTable[idx] = 0.f;
                }
                else  // 有效数据输出
                {
                    uvTables->xTable[idx] = x_proj;
                    uvTables->yTable[idx] = y_proj;
                }
            }
        }
        (*dataSize) = 2 * (uint32_t)tableSize;
    }
    return true;
}

void CoordinateUtil::transformationDepthToPointCloud(OBXYTables *xyTables, const void *depthImageData, void *pointCloudData, float positionDataScale,
                                                     OBCoordinateSystemType type) {
    const uint16_t *imageData = (const uint16_t *)depthImageData;
    float          *xyzData   = (float *)pointCloudData;
    float           x, y, z;
    int             coordinateSystemCoefficient = type == OB_LEFT_HAND_COORDINATE_SYSTEM ? -1 : 1;

    for(int i = 0; i < xyTables->width * xyTables->height; i++) {
        float x_tab = xyTables->xTable[i];

        uint16_t depthValue = imageData[i];
        if(!std::isnan(x_tab) && depthValue != 65535) {
            z = (float)depthValue;
            x = x_tab * (float)z;
            y = xyTables->yTable[i] * (float)z * coordinateSystemCoefficient;

            z *= positionDataScale;
            x *= positionDataScale;
            y *= positionDataScale;
        }
        else {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }

        xyzData[3 * i + 0] = x;
        xyzData[3 * i + 1] = y;
        xyzData[3 * i + 2] = z;
    }
}

void CoordinateUtil::transformationDepthToRGBDPointCloud(OBXYTables *xyTables, const void *depthImageData, const void *colorImageData, void *pointCloudData,
                                                         float positionDataScale, OBCoordinateSystemType type, bool colorDataNormalization) {
    const uint16_t *dImageData = (const uint16_t *)depthImageData;
    const uint8_t  *cImageData = (const uint8_t *)colorImageData;
    float          *xyzrgbData = (float *)pointCloudData;
    float           x, y, z;
    float           r, g, b;
    int             coordinateSystemCoefficient = type == OB_LEFT_HAND_COORDINATE_SYSTEM ? -1 : 1;
    float           colorDivCoeff               = colorDataNormalization ? 255.0f : 1.0f;

    for(int i = 0; i < xyTables->width * xyTables->height; i++) {
        float x_tab = xyTables->xTable[i];

        uint16_t depthValue = dImageData[i];
        if(!std::isnan(x_tab) && depthValue != 65535) {
            z = (float)depthValue;
            x = x_tab * (float)z;
            y = xyTables->yTable[i] * (float)z * coordinateSystemCoefficient;

            z *= positionDataScale;
            x *= positionDataScale;
            y *= positionDataScale;

            r = cImageData[3 * i + 0] / colorDivCoeff;
            g = cImageData[3 * i + 1] / colorDivCoeff;
            b = cImageData[3 * i + 2] / colorDivCoeff;
        }
        else {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            r = 0.0;
            g = 0.0;
            b = 0.0;
        }

        xyzrgbData[6 * i + 0] = x;
        xyzrgbData[6 * i + 1] = y;
        xyzrgbData[6 * i + 2] = z;
        xyzrgbData[6 * i + 3] = r;
        xyzrgbData[6 * i + 4] = g;
        xyzrgbData[6 * i + 5] = b;
    }
}

void CoordinateUtil::transformationDepthToRGBDPointCloudByUVTables(const OBCameraParam cameraParam, OBXYTables *uvTables, const void *depthImageData,
                                                                   const void *colorImageData, void *pointCloudData, float positionDataScale,
                                                                   OBCoordinateSystemType type, bool colorDataNormalization) {
    const uint16_t *dImageData = (const uint16_t *)depthImageData;
    const uint8_t  *cImageData = (const uint8_t *)colorImageData;
    float          *xyzrgbData = (float *)pointCloudData;
    float           x, y, z;
    float           r, g, b;
    int             coordinateSystemCoefficient = type == OB_LEFT_HAND_COORDINATE_SYSTEM ? -1 : 1;
    float           colorDivCoeff               = colorDataNormalization ? 255.0f : 1.0f;

    OBCameraIntrinsic rgbIntrinsic = cameraParam.rgbIntrinsic;
    for(int i = 0; i < uvTables->width * uvTables->height; i++) {
        //int u_tab = (int)uvTables->xTable[i];
        //int v_tab = (int)uvTables->yTable[i];

        int xValue = i % uvTables->width;
        int yValue = i / uvTables->width;

        uint16_t depthValue = dImageData[i];
        if(!std::isnan(uvTables->xTable[i]) && depthValue != 65535) {
            z = (float)depthValue;
            x = ((xValue - rgbIntrinsic.cx) / rgbIntrinsic.fx) * (float)z;
            y = ((yValue - rgbIntrinsic.cy) / rgbIntrinsic.fy) * (float)z * coordinateSystemCoefficient;

            z *= positionDataScale;
            x *= positionDataScale;
            y *= positionDataScale;

            int u_rgb   = (int)round(uvTables->xTable[i]);
            int v_rgb   = (int)round(uvTables->yTable[i]);
            int idx_rgb = v_rgb * uvTables->width + u_rgb;

            r = cImageData[3 * idx_rgb + 0] / colorDivCoeff;
            g = cImageData[3 * idx_rgb + 1] / colorDivCoeff;
            b = cImageData[3 * idx_rgb + 2] / colorDivCoeff;
        }
        else {
            x = (int)0.0;
            y = (int)0.0;
            z = 0.0;
            r = 0.0;
            g = 0.0;
            b = 0.0;
        }

        xyzrgbData[6 * i + 0] = (float)x;
        xyzrgbData[6 * i + 1] = (float)y;
        xyzrgbData[6 * i + 2] = z;
        xyzrgbData[6 * i + 3] = r;
        xyzrgbData[6 * i + 4] = g;
        xyzrgbData[6 * i + 5] = b;
    }
}

}  // namespace libobsensor