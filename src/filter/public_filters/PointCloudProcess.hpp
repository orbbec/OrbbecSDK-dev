#pragma once
#include "FilterBase.hpp"
#include "FormatConverterProcess.hpp"
#include <mutex>

namespace libobsensor {

class PointCloudFilter : public FilterBase {
    enum class OBPointcloudDistortionType {
        // The depth camera already includes the distortion from the color camera.
        // When converting to a point cloud, distortion correction is needed.
        OB_POINTCLOUD_UN_DISTORTION_TYPE = 0,

        // The depth camera does not include the distortion from the color camera.
        // When converting to a point cloud, the distortion from the color camera should be added,
        // and then distortion correction should be performed.
        OB_POINTCLOUD_ADD_DISTORTION_TYPE,

        // The depth camera has non-zero distortion parameters, and the color camera also has non-zero distortion parameters.
        // This might indicate an abnormality in the module parameters. By default, it is assumed that the distortion is zero.
        OB_POINTCLOUD_ZERO_DISTORTION_TYPE,
    };

public:
    PointCloudFilter(const std::string &name);
    virtual ~PointCloudFilter() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> createDepthPointCloud(std::shared_ptr<const Frame> frame);
    std::shared_ptr<Frame> createRGBDPointCloud(std::shared_ptr<const Frame> frame);
    std::shared_ptr<Frame> createRGBDPointCloud(std::shared_ptr<const Frame> depthFrame, std::shared_ptr<const Frame> colorFrame);

    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    void resetDistortionType();

protected:
    std::shared_ptr<const StreamProfile> currentStreamProfile_;
    std::shared_ptr<StreamProfile>       tarStreamProfile_;

    OBFormat               pointFormat_;
    float                      positionDataScale_;
    OBCameraParam          cameraParam_;
    int                    coordinateSystemCoefficient_;  // Coordinate system coefficient, default right-hand coordinate system
    bool                   isColorDataNormalization_;
    uint32_t               cachePointsBufferManagerWidth_;
    uint32_t               cachePointsBufferManagerHeight_;

    uint32_t               cacheRGBDPointsBufferManagerWidth_;
    uint32_t               cacheRGBDPointsBufferManagerHeight_;

    std::shared_ptr<FormatConverter> formatConverter_;

    OBFormat                   cacheColorFormat_;
    OBCoordinateSystemType     coordinateSystemType_;
    OBPointcloudDistortionType distortionType_;
    std::shared_ptr<float>     tablesData_;
    OBXYTables                 xyTables_;
};

}  // namespace libobsensor
