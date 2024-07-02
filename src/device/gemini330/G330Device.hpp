#pragma once
#include "DeviceBase.hpp"
#include "IDeviceEnumerator.hpp"

#include "timestamp/GlobalTimestampFitter.hpp"
#include "frameprocessor/FrameProcessor.hpp"
#include "G330AlgParamManager.hpp"
#include "G330PresetManager.hpp"
#include "G330DepthAlgModeManager.hpp"
#include "G330SensorStreamStrategy.hpp"

#include <map>
#include <memory>

namespace libobsensor {

class G330Device : public DeviceBase {
public:
    G330Device(const std::shared_ptr<const IDeviceEnumInfo> &info);
    virtual ~G330Device() noexcept;

    void init() override;

    std::shared_ptr<const DeviceInfo> getInfo() const override;
    const std::string                &getExtensionInfo(const std::string &infoKey) override;

    std::vector<OBSensorType>             getSensorTypeList() const override;
    std::vector<std::shared_ptr<IFilter>> createRecommendedPostProcessingFilters(OBSensorType type) override;
    DeviceComponentPtr<ISensor>           getSensor(OBSensorType type) override;

    void enableHeadBeat(bool enable) override;

    OBDeviceState getDeviceState() override;
    int           registerDeviceStateChangeCallback(DeviceStateChangedCallback callback) override;
    void          unregisterDeviceStateChangeCallback(int index) override;

    void reboot() override;
    void deactivate() override;

    void updateFirmware(const std::vector<uint8_t> &firmware, DeviceFwUpdateCallback updateCallback, bool async) override;

    const std::vector<uint8_t> &sendAndReceiveData(const std::vector<uint8_t> &data) override;

private:
    void initSensors();
    void initProperties();
    void initFrameMetadataParserContainer();

    std::shared_ptr<IFilter> getSpecifyFilter(const std::string &name, OBSensorType type, bool createIfNotExist = true);

private:
    const std::shared_ptr<const IDeviceEnumInfo> enumInfo_;
    std::shared_ptr<DeviceInfo>                  deviceInfo_;
    std::map<std::string, std::string>           extensionInfo_;
    std::map<OBSensorType, SensorEntry>          sensors_;

    std::shared_ptr<IFrameMetadataParserContainer> colorMdParserContainer_;
    std::shared_ptr<IFrameMetadataParserContainer> depthMdParserContainer_;
    std::shared_ptr<IFrameTimestampCalculator>     videoFrameTimestampCalculator_;
    // std::shared_ptr<IFrameTimestampCalculator>     imuFrameTimestampCalculator_;

    std::map<OBSensorType, std::shared_ptr<IFilter>> filters_;

    std::shared_ptr<FrameProcessorFactory> frameProcessorFactory_ = nullptr;
};
/* #endregion ------------G330Device declare end---------------- */

}  // namespace libobsensor
