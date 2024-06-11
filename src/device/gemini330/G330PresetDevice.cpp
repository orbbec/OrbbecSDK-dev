#include "G330PresetDevice.hpp"
#include "common/exception/ObException.hpp"
#include "common/utility/ObUtils.hpp"

#include <json/json.h>

namespace libobsensor {
namespace g2r {

void G330PresetDevice::initPreset(std::shared_ptr<EventBus> bus) {
    auto depthAlgModeList = getDepthAlgModeChecksumList();
    availablePresets_.emplace_back("Custom");
    for(auto &mode: depthAlgModeList) {
        availablePresets_.emplace_back(mode.name);
    }
    currentPreset_ = availablePresets_[1];
    switchDepthAlgMode(currentPreset_.c_str());
    // currentPreset_ = "Custom";
    storedAvailablePreset_ = currentPreset_;
    eventBus_              = bus;
    if(eventBus_) {
        std::function<void(OBPropertyID, OBPropertyValue, OBPermissionType)> onPropertyValueUpdate = [&](OBPropertyID propertyId, OBPropertyValue value,
                                                                                                         OBPermissionType permissionType) {
            if(permissionType == OB_PERMISSION_WRITE) {
                switch(propertyId) {
                case OB_PROP_LASER_CONTROL_INT:
                case OB_PROP_LASER_POWER_LEVEL_CONTROL_INT:
                case OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL:
                case OB_PROP_IR_EXPOSURE_INT:
                case OB_PROP_IR_GAIN_INT:
                case OB_PROP_IR_BRIGHTNESS_INT:
                case OB_PROP_COLOR_AUTO_EXPOSURE_BOOL:
                case OB_PROP_COLOR_EXPOSURE_INT:
                case OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL:
                case OB_PROP_COLOR_WHITE_BALANCE_INT:
                case OB_PROP_COLOR_GAIN_INT:
                case OB_PROP_COLOR_CONTRAST_INT:
                case OB_PROP_COLOR_SATURATION_INT:
                case OB_PROP_COLOR_SHARPNESS_INT:
                case OB_PROP_COLOR_BRIGHTNESS_INT:
                case OB_PROP_COLOR_HUE_INT:
                case OB_PROP_COLOR_GAMMA_INT:
                case OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT:
                case OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT:

                    currentPreset_ = "Custom";
                    storeCustomPreset("Custom");

                    break;
                default:
                    break;
                }
            }
        };
        eventBus_->listenEvent("PropertyValueUpdate", onPropertyValueUpdate);
    }
}

void G330PresetDevice::loadPreset(const std::string &presetName) {
    if(std::find(availablePresets_.begin(), availablePresets_.end(), presetName) == availablePresets_.end()) {
        throw std::invalid_argument("Invalid preset name: " + presetName);
    }

    auto iter = customPresets_.find(presetName);
    if(iter != customPresets_.end()) {
        // Load custom preset
        loadCustomPreset(iter->first, iter->second);
    }
    else {
        switchDepthAlgMode(presetName == "Custom" ? storedAvailablePreset_.c_str() : presetName.c_str());
        currentPreset_ = presetName;
    }
}

template <typename T = int> void trySetPresetValue(const std::shared_ptr<PropertyManager> &propMgr, OBPropertyID propId, const T &val) {
    if(propMgr->isPropertySupported(propId, OB_PERMISSION_WRITE)) {
        auto            accessor = propMgr->getPropertyAccessor(propId, OB_PERMISSION_WRITE);
        OBPropertyValue value;
        if(typeid(T) == typeid(int) || typeid(T) == typeid(bool)) {
            value.intValue = val;
        }
        else if(typeid(T) == typeid(float)) {
            value.floatValue = val;
        }
        else {
            throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Unsupported property type: " << typeid(T).name());
        }
        accessor->setPropertyValue(value);
    }
}

void G330PresetDevice::loadCustomPreset(const std::string &presetName, const G330Preset &preset) {
    switchDepthAlgMode(preset.depthAlgMode.c_str());
    auto lock            = tryLockResource();
    auto propertyManager = getPropertyManager(lock);

    trySetPresetValue(propertyManager, OB_PROP_LASER_CONTROL_INT, preset.laserState);
    trySetPresetValue(propertyManager, OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, preset.laserPowerLevel);
    trySetPresetValue(propertyManager, OB_PROP_IR_EXPOSURE_INT,
                      preset.depthExposureTime);  // using ir exposures for depth due to access property manager directly
    trySetPresetValue(propertyManager, OB_PROP_IR_GAIN_INT, preset.depthGain);
    trySetPresetValue(propertyManager, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, (bool)preset.depthAutoExposure);
    trySetPresetValue(propertyManager, OB_PROP_IR_BRIGHTNESS_INT, preset.depthBrightness);

    trySetPresetValue(propertyManager, OB_PROP_COLOR_EXPOSURE_INT, preset.colorExposureTime);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_GAIN_INT, preset.colorGain);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, (bool)preset.colorAutoExposure);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_WHITE_BALANCE_INT, preset.colorWhiteBalance);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, (bool)preset.colorAutoWhiteBalance);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_CONTRAST_INT, preset.colorContrast);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_SATURATION_INT, preset.colorSaturation);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_SHARPNESS_INT, preset.colorSharpness);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_BRIGHTNESS_INT, preset.colorBrightness);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_HUE_INT, preset.colorHue);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_GAMMA_INT, preset.colorGamma);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, (bool)preset.colorBacklightCompensation);
    trySetPresetValue(propertyManager, OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, (int)preset.colorPowerLineFrequency);

    currentPreset_ = presetName;
}

const std::string &G330PresetDevice::getCurrentPresetName() const {
    return currentPreset_;
}

const std::string &G330PresetDevice::getCurrentDepthModeName() const {
    return storedAvailablePreset_;
}

const std::vector<std::string> &G330PresetDevice::getAvailablePresetList() const {
    return availablePresets_;
}

void G330PresetDevice::loadPresetFromJsonData(const std::string &presetName, const std::vector<uint8_t> &jsonData) {
    Json::Value  root;
    Json::Reader reader;
    if(!reader.parse(std::string((const char *)jsonData.data(), jsonData.size()), root)) {
        throw std::invalid_argument("Invalid JSON data");
    }
    loadPresetFromJsonValue(presetName, root);
}

void G330PresetDevice::loadPresetFromJsonFile(const std::string &filePath) {
    Json::Value   root;
    std::ifstream ifs(filePath);
    ifs >> root;
    storeCustomPreset("TempCustom");
    loadPresetFromJsonValue(filePath, root);
    restoreCustomPreset("TempCustom");
}

void G330PresetDevice::loadPresetFromJsonValue(const std::string &presetName, const Json::Value &root) {
    G330Preset preset;
    preset.depthAlgMode               = root["depth_alg_mode"].asString();
    preset.laserState                 = root["laser_state"].asInt();
    preset.laserPowerLevel            = root["laser_power_level"].asInt();
    preset.depthAutoExposure          = root["depth_auto_exposure"].asBool();
    preset.depthExposureTime          = root["depth_exposure_time"].asInt();
    preset.depthGain                  = root["depth_gain"].asInt();
    preset.depthBrightness            = root["target_mean_intensity"].asInt();
    preset.colorAutoExposure          = root["color_auto_exposure"].asBool();
    preset.colorExposureTime          = root["color_exposure_time"].asInt();
    preset.colorAutoWhiteBalance      = root["color_auto_white_balance"].asBool();
    preset.colorWhiteBalance          = root["color_white_balance"].asInt();
    preset.colorGain                  = root["color_gain"].asInt();
    preset.colorContrast              = root["color_contrast"].asInt();
    preset.colorSaturation            = root["color_saturation"].asInt();
    preset.colorSharpness             = root["color_sharpness"].asInt();
    preset.colorBrightness            = root["color_brightness"].asInt();
    preset.colorHue                   = root["color_hue"].asInt();
    preset.colorGamma                 = root["color_gamma"].asInt();
    preset.colorBacklightCompensation = root["color_backlight_compensation"].asBool();
    preset.colorPowerLineFrequency    = root["color_power_line_frequency"].asInt();

    loadCustomPreset(presetName, preset);

    if(customPresets_.find(presetName) == customPresets_.end()) {
        availablePresets_.emplace_back(presetName);
    }
    customPresets_[presetName] = preset;
}

template <typename T = int> T tryGetPresetValue(std::shared_ptr<PropertyManager> propMgr, OBPropertyID propId) {
    if(propMgr->isPropertySupported(propId, OB_PERMISSION_READ)) {
        auto            accessor = propMgr->getPropertyAccessor(propId, OB_PERMISSION_READ);
        OBPropertyValue value;
        accessor->getPropertyValue(&value);
        if(typeid(T) == typeid(int) || typeid(T) == typeid(bool)) {
            return value.intValue;
        }
        else if(typeid(T) == typeid(float)) {
            return value.floatValue;
        }
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Unsupported property type: " << typeid(T).name());
    }
    return 0;
}

Json::Value G330PresetDevice::exportSettingsAsPresetJsonValue(const std::string &presetName) {
    G330Preset preset;
    auto      lock            = tryLockResource();
    auto      propertyManager = getPropertyManager(lock);

    preset.laserState                 = tryGetPresetValue<int>(propertyManager, OB_PROP_LASER_CONTROL_INT);
    preset.laserPowerLevel            = tryGetPresetValue<int>(propertyManager, OB_PROP_LASER_POWER_LEVEL_CONTROL_INT);
    preset.depthAutoExposure          = tryGetPresetValue<bool>(propertyManager, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    preset.depthExposureTime          = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_EXPOSURE_INT);
    preset.depthGain                  = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_GAIN_INT);
    preset.depthBrightness            = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_BRIGHTNESS_INT);
    preset.colorAutoExposure          = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
    preset.colorExposureTime          = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_EXPOSURE_INT);
    preset.colorAutoWhiteBalance      = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    preset.colorWhiteBalance          = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_WHITE_BALANCE_INT);
    preset.colorGain                  = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_GAIN_INT);
    preset.colorContrast              = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_CONTRAST_INT);
    preset.colorSaturation            = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_SATURATION_INT);
    preset.colorSharpness             = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_SHARPNESS_INT);
    preset.colorBrightness            = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_BRIGHTNESS_INT);
    preset.colorHue                   = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_HUE_INT);
    preset.colorGamma                 = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_GAMMA_INT);
    preset.colorBacklightCompensation = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT);
    preset.colorPowerLineFrequency    = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);

    preset.depthAlgMode = getCurrentDepthAlgModeChecksum().name;

    if(customPresets_.find(presetName) == customPresets_.end()) {
        availablePresets_.emplace_back(presetName);
    }
    customPresets_[presetName] = preset;
    currentPreset_             = presetName;

    storeCustomPreset("Custom");

    Json::Value root;
    root["depth_alg_mode"]               = preset.depthAlgMode;
    root["laser_state"]                  = preset.laserState;
    root["laser_power_level"]            = preset.laserPowerLevel;
    root["depth_auto_exposure"]          = preset.depthAutoExposure;
    root["depth_exposure_time"]          = preset.depthExposureTime;
    root["depth_gain"]                   = preset.depthGain;
    root["target_mean_intensity"]        = preset.depthBrightness;
    root["color_auto_exposure"]          = preset.colorAutoExposure;
    root["color_exposure_time"]          = preset.colorExposureTime;
    root["color_auto_white_balance"]     = preset.colorAutoWhiteBalance;
    root["color_white_balance"]          = preset.colorWhiteBalance;
    root["color_gain"]                   = preset.colorGain;
    root["color_contrast"]               = preset.colorContrast;
    root["color_saturation"]             = preset.colorSaturation;
    root["color_sharpness"]              = preset.colorSharpness;
    root["color_brightness"]             = preset.colorBrightness;
    root["color_hue"]                    = preset.colorHue;
    root["color_gamma"]                  = preset.colorGamma;
    root["color_backlight_compensation"] = preset.colorBacklightCompensation;
    root["color_power_line_frequency"]   = preset.colorPowerLineFrequency;

    return root;
}

const std::vector<uint8_t> &G330PresetDevice::exportSettingsAsPresetJsonData(const std::string &presetName) {
    auto                      root = exportSettingsAsPresetJsonValue(presetName);
    Json::StreamWriterBuilder builder;
    builder.settings_["enableYAMLCompatibility"] = true;
    builder.settings_["dropNullPlaceholders"]    = true;
    std::ostringstream oss;
    builder.newStreamWriter()->write(root, &oss);
    tmpJsonData_.clear();
    auto str = oss.str();
    std::copy(str.begin(), str.end(), std::back_inserter(tmpJsonData_));
    return tmpJsonData_;
}

void G330PresetDevice::exportSettingsAsPresetJsonFile(const std::string &filePath) {
    auto root = exportSettingsAsPresetJsonValue(filePath);

    std::ofstream             ofs(filePath);
    Json::StreamWriterBuilder builder;
    // builder.settings_["indentation"]             = "    ";
    builder.settings_["enableYAMLCompatibility"] = true;
    builder.settings_["dropNullPlaceholders"]    = true;
    auto writer                                  = builder.newStreamWriter();
    writer->write(root, &ofs);
}

void G330PresetDevice::storeCustomPreset(std::string presetName) {
    auto lock            = tryLockResource();
    auto propertyManager = getPropertyManager(lock);
    // std::string tempCustom      = "TempCustom";

    customPresets_[presetName].laserState                 = tryGetPresetValue<int>(propertyManager, OB_PROP_LASER_CONTROL_INT);
    customPresets_[presetName].laserPowerLevel            = tryGetPresetValue<int>(propertyManager, OB_PROP_LASER_POWER_LEVEL_CONTROL_INT);
    customPresets_[presetName].depthAutoExposure          = tryGetPresetValue<bool>(propertyManager, OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
    customPresets_[presetName].depthExposureTime          = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_EXPOSURE_INT);
    customPresets_[presetName].depthGain                  = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_GAIN_INT);
    customPresets_[presetName].depthBrightness            = tryGetPresetValue<int>(propertyManager, OB_PROP_IR_BRIGHTNESS_INT);
    customPresets_[presetName].colorAutoExposure          = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
    customPresets_[presetName].colorExposureTime          = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_EXPOSURE_INT);
    customPresets_[presetName].colorAutoWhiteBalance      = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
    customPresets_[presetName].colorWhiteBalance          = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_WHITE_BALANCE_INT);
    customPresets_[presetName].colorGain                  = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_GAIN_INT);
    customPresets_[presetName].colorContrast              = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_CONTRAST_INT);
    customPresets_[presetName].colorSaturation            = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_SATURATION_INT);
    customPresets_[presetName].colorSharpness             = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_SHARPNESS_INT);
    customPresets_[presetName].colorBrightness            = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_BRIGHTNESS_INT);
    customPresets_[presetName].colorHue                   = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_HUE_INT);
    customPresets_[presetName].colorGamma                 = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_GAMMA_INT);
    customPresets_[presetName].colorBacklightCompensation = tryGetPresetValue<bool>(propertyManager, OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT);
    customPresets_[presetName].colorPowerLineFrequency    = tryGetPresetValue<int>(propertyManager, OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);

    customPresets_[presetName].depthAlgMode = getCurrentDepthAlgModeChecksum().name;
    storedAvailablePreset_                  = getCurrentDepthAlgModeChecksum().name;
}

void G330PresetDevice::restoreCustomPreset(std::string presetName) {
    auto lock            = tryLockResource();
    auto propertyManager = getPropertyManager(lock);
    // std::string tempCustom      = "TempCustom";

    customPresets_["Custom"].laserState                 = customPresets_[presetName].laserState;
    customPresets_["Custom"].laserPowerLevel            = customPresets_[presetName].laserPowerLevel;
    customPresets_["Custom"].depthAutoExposure          = customPresets_[presetName].depthAutoExposure;
    customPresets_["Custom"].depthExposureTime          = customPresets_[presetName].depthExposureTime;
    customPresets_["Custom"].depthGain                  = customPresets_[presetName].depthGain;
    customPresets_["Custom"].depthBrightness            = customPresets_[presetName].depthBrightness;
    customPresets_["Custom"].colorAutoExposure          = customPresets_[presetName].colorAutoExposure;
    customPresets_["Custom"].colorExposureTime          = customPresets_[presetName].colorExposureTime;
    customPresets_["Custom"].colorAutoWhiteBalance      = customPresets_[presetName].colorAutoWhiteBalance;
    customPresets_["Custom"].colorWhiteBalance          = customPresets_[presetName].colorWhiteBalance;
    customPresets_["Custom"].colorGain                  = customPresets_[presetName].colorGain;
    customPresets_["Custom"].colorContrast              = customPresets_[presetName].colorContrast;
    customPresets_["Custom"].colorSaturation            = customPresets_[presetName].colorSaturation;
    customPresets_["Custom"].colorSharpness             = customPresets_[presetName].colorSharpness;
    customPresets_["Custom"].colorBrightness            = customPresets_[presetName].colorBrightness;
    customPresets_["Custom"].colorHue                   = customPresets_[presetName].colorHue;
    customPresets_["Custom"].colorGamma                 = customPresets_[presetName].colorGamma;
    customPresets_["Custom"].colorBacklightCompensation = customPresets_[presetName].colorBacklightCompensation;
    customPresets_["Custom"].colorPowerLineFrequency    = customPresets_[presetName].colorPowerLineFrequency;

    customPresets_["Custom"].depthAlgMode = customPresets_[presetName].depthAlgMode;
    storedAvailablePreset_                = customPresets_[presetName].depthAlgMode;
}

}  // namespace g2r
}  // namespace libobsensor