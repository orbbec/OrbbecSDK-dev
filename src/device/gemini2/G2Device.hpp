#pragma once
#include "DeviceBase.hpp"
#include "IDeviceEnumerator.hpp"
#include "frameprocessor/FrameProcessor.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class G2Device : public DeviceBase {
public:
    G2Device(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~G2Device() noexcept;

    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

private:
    void fetchDeviceInfo() override;

    void initSensorList();
    void initProperties();
    void initFrameMetadataParserContainer();
    void initSensorStreamProfile(std::shared_ptr<ISensor> sensor);

private:
    std::shared_ptr<IFrameMetadataParserContainer> colorMdParserContainer_;
    std::shared_ptr<IFrameMetadataParserContainer> depthMdParserContainer_;
    std::shared_ptr<IFrameTimestampCalculator>     videoFrameTimestampCalculator_;
};

}  // namespace libobsensor
