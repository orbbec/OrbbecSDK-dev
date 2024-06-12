#pragma once
#include "core/device/AbstractDevice.hpp"
#include "common/parameter/Mx6600CalibParamParser.hpp"
#include "G330AlgParamManager.hpp"
#include "G330PresetDevice.hpp"
#include "G330SensorStartStrategy.hpp"

namespace libobsensor {
namespace g330 {

class Gemini2ProfileFilter;

class G330Device : public AbstractDevice, public G330PresetDevice {
public:
    G330Device(std::shared_ptr<ObPal> obPal, const std::shared_ptr<DeviceInfo> info);
    virtual ~G330Device() noexcept;

    virtual std::shared_ptr<DeviceInfo> getDeviceInfo() override;

    virtual OBDepthAlgModeChecksum              getCurrentDepthAlgModeChecksum() override;
    virtual void                                switchDepthAlgMode(const OBDepthAlgModeChecksum &modeChecksum) override;
    virtual void                                switchDepthAlgMode(const char *modeName) override;
    virtual std::vector<OBDepthAlgModeChecksum> getDepthAlgModeChecksumList() override;
    virtual OBD2CAlignParam                     getD2CAlignParam() override;
    virtual OBD2CProfile                        getSupportedProfileInfo(uint32_t colorWidth, uint32_t colorHeight, uint32_t depthWidth, uint32_t depthHeight,
                                                                        OBAlignMode alignMode) override;

    virtual void reboot() override;

protected:
    virtual void initSensorMap() override;
    virtual void createSensor(OBSensorType sensorType) override;

    virtual void loadDefaultConfig();

    virtual void initDepthProcessParam() override;
    virtual void initPropertyList() override;
    virtual void createCommand() override;

    virtual void deviceUpgrade(std::string filePath, DeviceUpgradeCallback upgradeCallback, bool async) override;
    virtual void deviceUpgrade(const char *fileData, uint32_t fileSize, DeviceUpgradeCallback upgradeCallback, bool async) override;

    virtual OBCameraParam              getCurCameraParam() override;
    virtual std::vector<OBD2CProfile>  getD2CSupportedProfileList() override;
    virtual std::vector<OBCameraParam> getCalibrationCameraParamList() override;

    virtual std::vector<std::shared_ptr<FrameProcessingBlock>> getRecommendedProcessingBlockList(OBSensorType type) override;

    virtual std::unique_ptr<PropertyAccessor> getPropertyAccessorForce(OBPropertyID propertyId) override;
    virtual std::unique_ptr<PropertyAccessor> getPropertyAccessor(OBPropertyID propertyId, OBPermissionType permission) override;

    virtual bool onPropertyUpdate(OBPropertyID propertyId, OBPropertyValue propertyValue, OBPermissionType permissionType) override;

    virtual bool getCameraParamTransformState() override {
        return true;
    }
    void switchD2CParams(int index) override {
        curD2CParamIndex_ = index;
    }

    void initHeartBeatEventListener();

private:
    void createDepthSensor();
    void createColorSensor();
    void createLeftIrSensor();
    void createRightIrSensor();
    void createAccelSensor();
    void createGyroSensor();

    void loadDefaultStreamProfile();

    void initDepthAlgMode();
    void initFrameMetadataParserContainer();
    void updateHeartBeatByXmlConfig();

    OBDepthAlgModeChecksum              requestCurrentDepthAglMode();
    std::vector<OBDepthAlgModeChecksum> depthAlgModeChecksumListParse(const uint8_t *filedata, uint32_t dataSize);

private:
    std::vector<OBDepthAlgModeChecksum> depthAlgModeChecksumList_;
    bool                                hwD2DEnable_;
    bool                                swD2DEnable_;
    OBDepthAlgModeChecksum              currentDepthAlgMode_;

    std::mutex depthAlgModeChecksumMutex_;
    std::mutex effectiveProfileMutex_;

    DeviceStateChangedCallback stateChangedCallback_;

    std::shared_ptr<IMetadataParserContainer> depthMdParserContainer_;
    std::shared_ptr<IMetadataParserContainer> colorMdParserContainer_;

    std::shared_ptr<G330AlgParamManager>     algParamManager_;
    std::shared_ptr<G330SensorStartStrategy> sensorStartStrategy_;

    std::vector<std::shared_ptr<FrameProcessingBlock>> depthProcessingBlockList_;
    std::vector<std::shared_ptr<FrameProcessingBlock>> irProcessingBlockList_;
    std::vector<std::shared_ptr<FrameProcessingBlock>> colorProcessingBlockList_;

    std::thread upgradeThread_;
    std::string ispVersion_;

    OBFrameMetadataType frameTimeStampMetadataType_;
};
/* #endregion ------------G330Device declare end---------------- */

}  // namespace g330
}  // namespace libobsensor
