#pragma once
#include "DeviceBase.hpp"
#include "IDeviceEnumerator.hpp"
#include "frameprocessor/FrameProcessor.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class G330Device : public DeviceBase {
public:
    G330Device(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~G330Device() noexcept;

    std::shared_ptr<const DeviceInfo> getInfo() const override;
    const std::string                &getExtensionInfo(const std::string &infoKey) override;

    std::vector<OBSensorType>             getSensorTypeList() const override;
    DeviceComponentPtr<ISensor>           getSensor(OBSensorType type) override;
    bool                                  hasAnySensorStreamActivated() const override;
    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;

    void reboot() override;
    void deactivate() override;

    void updateFirmware(const std::vector<uint8_t> &firmware, DeviceFwUpdateCallback updateCallback, bool async) override;

private:
    void initSensors();
    void initProperties();
    void initFrameMetadataParserContainer();

private:
    const std::shared_ptr<const IDeviceEnumInfo> enumInfo_;
    std::shared_ptr<DeviceInfo>                  deviceInfo_;
    std::map<std::string, std::string>           extensionInfo_;
    std::map<OBSensorType, SensorEntry>          sensors_;

    std::shared_ptr<IFrameMetadataParserContainer> colorMdParserContainer_;
    std::shared_ptr<IFrameMetadataParserContainer> depthMdParserContainer_;
    std::shared_ptr<IFrameTimestampCalculator>     videoFrameTimestampCalculator_;
    // std::shared_ptr<IFrameTimestampCalculator>     imuFrameTimestampCalculator_;
};

}  // namespace libobsensor
