#pragma once
#include "DeviceBase.hpp"
#include "IDeviceEnumerator.hpp"
#include "frameprocessor/FrameProcessor.hpp"

#include <map>
#include <memory>

namespace libobsensor {
class FemtoMegaDevice : public DeviceBase {
public:
    FemtoMegaDevice(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~FemtoMegaDevice() noexcept;

    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

private:
    void init() override;
    void initSensorList();
    void initProperties();

#if defined(BUILD_NET_PAL)
    void initNetSensorList();
    void initNetProperties();
#endif

    void initSensorStreamProfile(std::shared_ptr<ISensor> sensor);
    void fetchDeviceInfo() override;

private:
    std::shared_ptr<IFrameMetadataParserContainer> colorMdParserContainer_;
    std::shared_ptr<IFrameMetadataParserContainer> depthMdParserContainer_;
    std::shared_ptr<IFrameTimestampCalculator>     videoFrameTimestampCalculator_;
};
}  // namespace libobsensor