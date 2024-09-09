#pragma once
#include "DeviceBase.hpp"
#include "IDeviceManager.hpp"
#include "frameprocessor/FrameProcessor.hpp"
#include "G330FrameMetadataParserContainer.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class G330Device : public DeviceBase {
public:
    G330Device(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~G330Device() noexcept;

    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

private:
    void init() override;
    void initSensorList();
    void initSensorListGMSL();
    void initProperties();
    void initFrameMetadataParserContainer();
    void initSensorStreamProfile(std::shared_ptr<ISensor> sensor);

private:
    std::shared_ptr<const StreamProfile> loadDefaultStreamProfile(OBSensorType sensorType);

private:
    std::shared_ptr<G330FrameMetadataParserContainer> colorMdParserContainer_;
    std::shared_ptr<G330FrameMetadataParserContainer> depthMdParserContainer_;

    const uint64_t                                              deviceTimeFreq_ = 1000;     // in ms
    const uint64_t                                              frameTimeFreq_  = 1000000;  // in us
    std::function<std::shared_ptr<IFrameTimestampCalculator>()> videoFrameTimestampCalculatorCreator_;
    bool                                                        isGmslDevice_;
};

}  // namespace libobsensor
