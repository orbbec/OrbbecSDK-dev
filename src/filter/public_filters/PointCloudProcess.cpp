#include "PointCloudProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "openobsdk/h/ObTypes.h"
#include "utils/CoordinateUtil.hpp"

namespace libobsensor {

PointCloudFilter::PointCloudFilter(const std::string &name)
    : FilterBase(name),
      pointFormat_(OB_FORMAT_POINT),
      positionDataScale_(1.0f),
      coordinateSystemCoefficient_(1),
      isColorDataNormalization_(false),
      cachePointsBufferManagerWidth_(0),
      cachePointsBufferManagerHeight_(0),
      cacheRGBDPointsBufferManagerWidth_(0),
      cacheRGBDPointsBufferManagerHeight_(0),
      cacheColorFormat_(OB_FORMAT_UNKNOWN) {}

PointCloudFilter::~PointCloudFilter() noexcept {}

void PointCloudFilter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 4) {
        throw invalid_value_exception("PointCloudFilter config error: params size not match");
    }
    try {
        OBFormat type = (OBFormat)std::stoi(params[0]);
        if(type != OB_FORMAT_POINT && type != OB_FORMAT_RGB_POINT) {
            LOG_ERROR("Invalid type, the pointType must be OB_FORMAT_POINT or OB_FORMAT_RGB_POINT");
        }
        else {
            pointFormat_ = type;
        }

        float scale = std::stof(params[1]);
        if(scale >= 0.01 && scale <= 100) {
            positionDataScale_ = scale;
        }

        int colorDataNormalizationState = std::stoi(params[2]);
        isColorDataNormalization_       = colorDataNormalizationState == 0 ? false : true;

        int csType = std::stoi(params[3]);
        if(csType == OB_LEFT_HAND_COORDINATE_SYSTEM) {
            coordinateSystemCoefficient_ = -1;
        }
        else if(csType == OB_RIGHT_HAND_COORDINATE_SYSTEM) {
            coordinateSystemCoefficient_ = 1;
        }
    }
    catch(const std::exception &e) {
        throw invalid_value_exception("PointCloudFilter config error: " + std::string(e.what()));
    }
}

const std::string &PointCloudFilter::getConfigSchema() const {
    // csv format: name，type， min，max，step，default，description
    static const std::string schema = "pointFormat, int, 0, 1, 1, 0, create point type\n"
                                      "scale, float, 0.01, 100, 0.01, 1.0, create point type\n"
                                      "colorDataState, int, 0, 1, 1, 0, color data normal state\n"
                                      "coordinateSystemType, int, 0, 1, 1, 1, Coordinate system representation type\n";
    return schema;
}

std::shared_ptr<Frame> PointCloudFilter::createDepthPointCloud(std::shared_ptr<const Frame> frame) {
    std::shared_ptr<const Frame> depthFrame = frame;
    if(frame->is<FrameSet>()) {
        auto frameSet = frame->as<FrameSet>();
        depthFrame    = frameSet->getFrame(OB_FRAME_DEPTH);
    }

    if(depthFrame == nullptr) {
        LOG_ERROR_INTVL("No depth frame found in frameset!");
        return nullptr;
    }

    // Request depth point cloud frame
    auto     depthVideoFrame = depthFrame->as<VideoFrame>();
    uint32_t h               = depthVideoFrame->getHeight();
    uint32_t w               = depthVideoFrame->getWidth();
    if(cachePointsBufferManagerWidth_ != w || cachePointsBufferManagerHeight_ != h) {
        if(tablesData_) {
            tablesData_.reset();
        }
    }
    cachePointsBufferManagerWidth_  = w;
    cachePointsBufferManagerHeight_ = h;

    auto pointFrame = FrameFactory::createFrame(OB_FRAME_POINTS, OB_FORMAT_POINT, w * h * sizeof(OBPoint));
    if(pointFrame == nullptr) {
        LOG_ERROR_INTVL("Acquire frame from frameBufferManager failed!");
        return nullptr;
    }

    // TODO: cameraParam_ need to get from StreamProfile
    if(tablesData_ == nullptr) {
        uint32_t tablesSize = w * h * 2;
        tablesData_         = std::shared_ptr<float>(new float[tablesSize], std::default_delete<float[]>());

        OBCameraIntrinsic  depthIntrinsic;
        OBCameraDistortion depthDisto;      
        memcpy(&depthIntrinsic, &cameraParam_.depthIntrinsic, sizeof(OBCameraIntrinsic));
        memcpy(&depthDisto, &cameraParam_.depthDistortion, sizeof(OBCameraDistortion));
        if(!CoordinateUtil::transformationInitXYTables(depthIntrinsic, depthDisto, reinterpret_cast<float *>(tablesData_.get()), &tablesSize,
                                                       &xyTables_)) {
            LOG_ERROR_INTVL("Init transformation coordinate tables failed!");
            tablesData_.reset();
            return nullptr;
        }
    }

    auto streamProfile      = depthVideoFrame->getStreamProfile();
    auto videoStreamProfile = frame->getStreamProfile();
    if(!currentStreamProfile_ || currentStreamProfile_.get() != videoStreamProfile.get()) {
        tarStreamProfile_     = videoStreamProfile->clone();
        currentStreamProfile_ = videoStreamProfile;
    }

    CoordinateUtil::transformationDepthToPointCloud(&xyTables_, depthFrame->getData(), (void *)pointFrame->getData(), positionDataScale_,
                                                    coordinateSystemType_);

    float depthValueScale = depthFrame->as<DepthFrame>()->getValueScale();
    pointFrame->copyInfo(depthFrame);
    tarStreamProfile_->setFormat(OB_FORMAT_POINT);
    pointFrame->setStreamProfile(tarStreamProfile_);
    // Actual coordinate scaling = Depth scaling factor / Set coordinate scaling factor.
    pointFrame->as<PointsFrame>()->setCoordinateValueScale(depthValueScale / positionDataScale_);

    return pointFrame;
}

std::shared_ptr<Frame> PointCloudFilter::createRGBDPointCloud(std::shared_ptr<const Frame> frame) {
    std::shared_ptr<const FrameSet> frameSet = nullptr;

    if(frame->is<const FrameSet>()) {
        frameSet = frame->as<FrameSet>();
    }
    else {
        LOG_ERROR_INTVL("Input frame is not a frameset, can not convert to pointcloud!");
        return nullptr;
    }

    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
    if(depthFrame == nullptr) {
        LOG_ERROR_INTVL("No depth frame found in frameset!");
        return nullptr;
    }

    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
    if(colorFrame == nullptr) {
        LOG_ERROR_INTVL("no color frame found in frameset!");
        return nullptr;
    }
    return createRGBDPointCloud(depthFrame, colorFrame);
}

std::shared_ptr<Frame> PointCloudFilter::createRGBDPointCloud(std::shared_ptr<const Frame> depthFrame, std::shared_ptr<const Frame> colorFrame) {
    auto     depthVideoFrame = depthFrame->as<VideoFrame>();
    uint32_t dstHeight       = depthVideoFrame->getHeight();
    uint32_t dstWidth        = depthVideoFrame->getWidth();

    auto colorVideoFrame = colorFrame->as<VideoFrame>();
    if(cacheRGBDPointsBufferManagerWidth_ != dstWidth || cacheRGBDPointsBufferManagerHeight_ != dstHeight) {
        if(tablesData_) {
            tablesData_.reset();
        }
    }
    cacheRGBDPointsBufferManagerWidth_  = dstWidth;
    cacheRGBDPointsBufferManagerHeight_ = dstHeight;

    // Create an RGBD point cloud frame
    auto pointFrame = FrameFactory::createFrame(OB_FRAME_POINTS, OB_FORMAT_RGB_POINT, dstWidth * dstHeight * sizeof(OBColorPoint));
    if(pointFrame == nullptr) {
        LOG_WARN_INTVL("Acquire frame from frameBufferManager failed!");
        return nullptr;
    }

    // decode rgb frame
    auto colorFormat = colorFrame->getFormat();
    if(cacheColorFormat_ != colorFormat) {
        formatConverter_.reset();
    }

    if(formatConverter_ == nullptr) {
        cacheColorFormat_ = colorFormat;
        formatConverter_  = std::make_shared<FormatConverter>("FormatConverter");
    }

    std::shared_ptr<const Frame> tarFrame = nullptr;
    std::vector<std::string> params;
    switch(colorFrame->getFormat()) {
    case OB_FORMAT_YUYV:
        // FORMAT_YUYV_TO_RGB
        params.push_back("0");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_UYVY:
        // FORMAT_UYVY_TO_RGB888
        params.push_back("10");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_I420:
        // FORMAT_I420_TO_RGB888
        params.push_back("1");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_MJPG:
        // FORMAT_MJPG_TO_RGB888
        params.push_back("7");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_NV21:
        // FORMAT_NV21_TO_RGB888
        params.push_back("2");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_NV12:
        // FORMAT_NV12_TO_RGB888
        params.push_back("3");
        formatConverter_->updateConfig(params);
        break;
    case OB_FORMAT_RGB:
        tarFrame = colorFrame;
        break;
    default:
        throw unsupported_operation_exception("unsupported color format for RgbDepth pointCloud convert!");
    }
    uint8_t *colorData = nullptr;
    if(colorFrame->getFormat() != OB_FORMAT_RGB) {
        tarFrame = formatConverter_->process(colorFrame);
    }

    if(tarFrame) {
        colorData = (uint8_t *)tarFrame->getData();
    }
    else {
        LOG_ERROR_INTVL("get rgb data failed!");
        return nullptr;
    }

    if(tablesData_ == nullptr) {
        uint32_t tablesSize = dstWidth * dstHeight * 2;
        tablesData_         = std::shared_ptr<float>(new float[tablesSize], std::default_delete<float[]>());
        // TODO: need to get cameraParam
        if(distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_UN_DISTORTION_TYPE) {
            OBCameraIntrinsic rgbIntrinsic; //TODO
            OBCameraDistortion rgbDisto;    //TODO
            memcpy(&rgbIntrinsic, &cameraParam_.rgbIntrinsic, sizeof(OBCameraIntrinsic));
            memcpy(&rgbDisto, &cameraParam_.rgbDistortion, sizeof(OBCameraDistortion));
            if(!CoordinateUtil::transformationInitXYTables(rgbIntrinsic, rgbDisto, reinterpret_cast<float *>(tablesData_.get()), &tablesSize,
                                                           &xyTables_)) {
                LOG_ERROR_INTVL("Init transformation coordinate tables failed!");
                return nullptr;
            }
        }
        else if(distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_ADD_DISTORTION_TYPE) {
            OBCameraIntrinsic  rgbIntrinsic;  // TODO
            OBCameraDistortion rgbDisto;      // TODO
            memcpy(&rgbIntrinsic, &cameraParam_.rgbIntrinsic, sizeof(OBCameraIntrinsic));
            memcpy(&rgbDisto, &cameraParam_.rgbDistortion, sizeof(OBCameraDistortion));
            if(!CoordinateUtil::transformationInitAddDistortionUVTables(rgbIntrinsic, rgbDisto, reinterpret_cast<float *>(tablesData_.get()), &tablesSize,
                                                                        &xyTables_)) {
                LOG_ERROR_INTVL("Init add distortion transformation coordinate tables failed!");
                return nullptr;
            }
        }
        else if(distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_ZERO_DISTORTION_TYPE) {
            OBCameraIntrinsic  rgbIntrinsic;  // TODO
            OBCameraDistortion rgbDisto;      
            memcpy(&rgbIntrinsic, &cameraParam_.rgbIntrinsic, sizeof(OBCameraIntrinsic));
            memset(&rgbDisto, 0, sizeof(OBCameraDistortion));
            if(!CoordinateUtil::transformationInitXYTables(rgbIntrinsic, rgbDisto, reinterpret_cast<float *>(tablesData_.get()), &tablesSize,
                                                           &xyTables_)) {
                LOG_ERROR_INTVL("Init transformation coordinate tables failed!");
                return nullptr;
            }
        }
    }

    if(distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_UN_DISTORTION_TYPE
       || distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_ZERO_DISTORTION_TYPE) {
        CoordinateUtil::transformationDepthToRGBDPointCloud(&xyTables_, depthFrame->getData(), colorData, (void *)pointFrame->getData(), positionDataScale_,
                                                            coordinateSystemType_, isColorDataNormalization_);
    }
    else if(distortionType_ == OBPointcloudDistortionType::OB_POINTCLOUD_ADD_DISTORTION_TYPE) {
        OBCameraIntrinsic  rgbIntrinsic;  // TODO
        memcpy(&rgbIntrinsic, &cameraParam_.rgbIntrinsic, sizeof(OBCameraIntrinsic));
        CoordinateUtil::transformationDepthToRGBDPointCloudByUVTables(rgbIntrinsic, &xyTables_, depthFrame->getData(), colorData, (void *)pointFrame->getData(),
                                                                      positionDataScale_, coordinateSystemType_, isColorDataNormalization_);
    }

    // 完善帧信息
    float depthValueScale = depthFrame->as<DepthFrame>()->getValueScale();
    pointFrame->copyInfo(depthFrame);
    auto streamProfile = pointFrame->getStreamProfile()->clone();
    streamProfile->setFormat(OB_FORMAT_RGB_POINT);
    pointFrame->setStreamProfile(streamProfile);
    //Actual coordinate scaling = Depth scaling factor / Set coordinate scaling factor.
    pointFrame->as<PointsFrame>()->setCoordinateValueScale(depthValueScale / positionDataScale_);
    return pointFrame;
}

std::shared_ptr<Frame> PointCloudFilter::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    std::shared_ptr<Frame> pointsFrame = nullptr;
    auto                   newFrame    = FrameFactory::cloneFrame(frame);
    if(pointFormat_ == OB_FORMAT_POINT) {
        pointsFrame = createDepthPointCloud(newFrame);
    }
    else {
        pointsFrame = createRGBDPointCloud(newFrame);
    }
    return pointsFrame;
}

void PointCloudFilter::resetDistortionType() {
    OBCameraDistortion colorDistortion = cameraParam_.rgbDistortion;
    OBCameraDistortion depthDistortion = cameraParam_.depthDistortion;

    OBCameraDistortion zeroDistortion = {};

    if(memcmp(&colorDistortion, &depthDistortion, sizeof(OBCameraDistortion)) == 0) {
        distortionType_ = OBPointcloudDistortionType::OB_POINTCLOUD_UN_DISTORTION_TYPE;
    }
    else if((memcmp(&depthDistortion, &zeroDistortion, sizeof(OBCameraDistortion)) == 0)
            && (memcmp(&colorDistortion, &zeroDistortion, sizeof(OBCameraDistortion)) != 0)) {
        distortionType_ = OBPointcloudDistortionType::OB_POINTCLOUD_ADD_DISTORTION_TYPE;
    }
    else {
        distortionType_ = OBPointcloudDistortionType::OB_POINTCLOUD_ZERO_DISTORTION_TYPE;
    }
}

}  // namespace libobsensor
