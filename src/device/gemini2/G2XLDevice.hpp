#pragma once
#include "DeviceBase.hpp"
#include "IDeviceManager.hpp"
#include "frameprocessor/FrameProcessor.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class G2XLDevice : public DeviceBase {
public:
    G2XLDevice(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~G2XLDevice() noexcept;

    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

private:
    void init() override;
    void initSensorList();
    void initProperties();
    void initSensorStreamProfile(std::shared_ptr<ISensor> sensor);

private:
    const uint64_t                                              frameTimeFreq_  = 1000000;
    uint64_t                                                    deviceTimeFreq_ = 1000000;
    std::function<std::shared_ptr<IFrameTimestampCalculator>()> videoFrameTimestampCalculatorCreator_;
};

}  // namespace libobsensor
