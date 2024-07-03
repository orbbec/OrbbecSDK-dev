#include "G330PresetManager.hpp"
#include "property/InternalProperty.hpp"
#include "InternalTypes.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"
#include "G330DepthAlgModeManager.hpp"

#include <json/json.h>

namespace libobsensor {

G330PresetManager::G330PresetManager(IDevice *owner) : DeviceComponentBase(owner) {
    auto comp                = owner->getComponent(OB_DEV_COMPONENT_DEPTH_ALG_MODE_MANAGER);
    auto depthAlgModeManager = comp.as<G330DepthAlgModeManager>();
    auto depthAlgModeList    = depthAlgModeManager->getDepthAlgModeList();

    availablePresets_.emplace_back("Custom");
    for(auto &mode: depthAlgModeList) {
        availablePresets_.emplace_back(mode.name);
    }
    currentPreset_ = availablePresets_[1];
    depthAlgModeManager->switchDepthAlgMode(currentPreset_.c_str());

    auto onPropertyValueUpdate = [&](uint32_t propertyId, const uint8_t *data, size_t size, PropertyOperationType operationType) {
        utils::unusedVar(data);
        utils::unusedVar(size);

        if(operationType == PROP_OP_WRITE) {
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
    auto propAccessor = owner->getPropertyAccessor();
    propAccessor->registerAccessCallback(onPropertyValueUpdate);
}

void G330PresetManager::loadPreset(const std::string &presetName) {
    if(std::find(availablePresets_.begin(), availablePresets_.end(), presetName) == availablePresets_.end()) {
        throw std::invalid_argument("Invalid preset name: " + presetName);
    }

    auto iter = customPresets_.find(presetName);
    if(iter != customPresets_.end()) {
        // Load custom preset
        loadCustomPreset(iter->first, iter->second);
    }
    else {
        auto owner               = getOwner();
        auto comp                = owner->getComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
        auto depthAlgModeManager = comp.as<G330DepthAlgModeManager>();
        depthAlgModeManager->switchDepthAlgMode(presetName.c_str());
        currentPreset_ = presetName;
    }
}

void G330PresetManager::loadCustomPreset(const std::string &presetName, const G330Preset &preset) {

    auto owner = getOwner();
    auto comp  = owner->getComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
    {
        auto depthAlgModeManager = comp.as<G330DepthAlgModeManager>();
        depthAlgModeManager->switchDepthAlgMode(preset.depthAlgMode.c_str());
    }

    auto propAccessor = owner->getPropertyAccessor();
    propAccessor->setPropertyValueT(OB_PROP_LASER_CONTROL_INT, preset.laserState);
    propAccessor->setPropertyValueT(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, preset.laserPowerLevel);
    propAccessor->setPropertyValueT(OB_PROP_IR_EXPOSURE_INT,
                                    preset.depthExposureTime);  // using ir exposures for depth due to access property manager directly
    propAccessor->setPropertyValueT(OB_PROP_IR_GAIN_INT, preset.depthGain);
    propAccessor->setPropertyValueT(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, (bool)preset.depthAutoExposure);
    propAccessor->setPropertyValueT(OB_PROP_IR_BRIGHTNESS_INT, preset.depthBrightness);

    propAccessor->setPropertyValueT(OB_PROP_COLOR_EXPOSURE_INT, preset.colorExposureTime);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_GAIN_INT, preset.colorGain);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, (bool)preset.colorAutoExposure);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_WHITE_BALANCE_INT, preset.colorWhiteBalance);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, (bool)preset.colorAutoWhiteBalance);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_CONTRAST_INT, preset.colorContrast);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_SATURATION_INT, preset.colorSaturation);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_SHARPNESS_INT, preset.colorSharpness);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_BRIGHTNESS_INT, preset.colorBrightness);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_HUE_INT, preset.colorHue);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_GAMMA_INT, preset.colorGamma);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, (bool)preset.colorBacklightCompensation);
    propAccessor->setPropertyValueT(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, (int)preset.colorPowerLineFrequency);

    currentPreset_ = presetName;
}

const std::string &G330PresetManager::getCurrentPresetName() const {
    return currentPreset_;
}

const std::vector<std::string> &G330PresetManager::getAvailablePresetList() const {
    return availablePresets_;
}

void G330PresetManager::loadPresetFromJsonData(const std::string &presetName, const std::vector<uint8_t> &jsonData) {
    Json::Value  root;
    Json::Reader reader;
    if(!reader.parse(std::string((const char *)jsonData.data(), jsonData.size()), root)) {
        throw std::invalid_argument("Invalid JSON data");
    }
    storeCustomPreset("TempCustom");
    loadPresetFromJsonValue(presetName, root);
    restoreCustomPreset("TempCustom");
}

void G330PresetManager::loadPresetFromJsonFile(const std::string &filePath) {
    Json::Value   root;
    std::ifstream ifs(filePath);
    ifs >> root;
    storeCustomPreset("TempCustom");
    loadPresetFromJsonValue(filePath, root);
    restoreCustomPreset("TempCustom");
}

void G330PresetManager::loadPresetFromJsonValue(const std::string &presetName, const Json::Value &root) {
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

Json::Value G330PresetManager::exportSettingsAsPresetJsonValue(const std::string &presetName) {
    G330Preset preset;

    {
        auto owner        = getOwner();
        auto propAccessor = owner->getPropertyAccessor();

        preset.laserState                 = propAccessor->getPropertyValueT<int>(OB_PROP_LASER_CONTROL_INT);
        preset.laserPowerLevel            = propAccessor->getPropertyValueT<int>(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT);
        preset.depthAutoExposure          = propAccessor->getPropertyValueT<bool>(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
        preset.depthExposureTime          = propAccessor->getPropertyValueT<int>(OB_PROP_IR_EXPOSURE_INT);
        preset.depthGain                  = propAccessor->getPropertyValueT<int>(OB_PROP_IR_GAIN_INT);
        preset.depthBrightness            = propAccessor->getPropertyValueT<int>(OB_PROP_IR_BRIGHTNESS_INT);
        preset.colorAutoExposure          = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
        preset.colorExposureTime          = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_EXPOSURE_INT);
        preset.colorAutoWhiteBalance      = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
        preset.colorWhiteBalance          = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_WHITE_BALANCE_INT);
        preset.colorGain                  = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_GAIN_INT);
        preset.colorContrast              = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_CONTRAST_INT);
        preset.colorSaturation            = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_SATURATION_INT);
        preset.colorSharpness             = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_SHARPNESS_INT);
        preset.colorBrightness            = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_BRIGHTNESS_INT);
        preset.colorHue                   = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_HUE_INT);
        preset.colorGamma                 = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_GAMMA_INT);
        preset.colorBacklightCompensation = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT);
        preset.colorPowerLineFrequency    = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);
    }

    {
        auto owner               = getOwner();
        auto comp                = owner->getComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
        auto depthAlgModeManager = comp.as<G330DepthAlgModeManager>();
        preset.depthAlgMode      = depthAlgModeManager->getCurrentDepthAlgModeChecksum().name;
    }

    if(customPresets_.find(presetName) == customPresets_.end()) {
        availablePresets_.emplace_back(presetName);
    }
    customPresets_[presetName] = preset;
    currentPreset_             = presetName;

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

const std::vector<uint8_t> &G330PresetManager::exportSettingsAsPresetJsonData(const std::string &presetName) {
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

void G330PresetManager::exportSettingsAsPresetJsonFile(const std::string &filePath) {
    auto root = exportSettingsAsPresetJsonValue(filePath);

    std::ofstream             ofs(filePath);
    Json::StreamWriterBuilder builder;
    // builder.settings_["indentation"]             = "    ";
    builder.settings_["enableYAMLCompatibility"] = true;
    builder.settings_["dropNullPlaceholders"]    = true;
    auto writer                                  = builder.newStreamWriter();
    writer->write(root, &ofs);
}

void G330PresetManager::storeCustomPreset(std::string presetName) {
    {
        auto owner        = getOwner();
        auto propAccessor = owner->getPropertyAccessor();

        customPresets_[presetName].laserState                 = propAccessor->getPropertyValueT<int>(OB_PROP_LASER_CONTROL_INT);
        customPresets_[presetName].laserPowerLevel            = propAccessor->getPropertyValueT<int>(OB_PROP_LASER_POWER_LEVEL_CONTROL_INT);
        customPresets_[presetName].depthAutoExposure          = propAccessor->getPropertyValueT<bool>(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
        customPresets_[presetName].depthExposureTime          = propAccessor->getPropertyValueT<int>(OB_PROP_IR_EXPOSURE_INT);
        customPresets_[presetName].depthGain                  = propAccessor->getPropertyValueT<int>(OB_PROP_IR_GAIN_INT);
        customPresets_[presetName].depthBrightness            = propAccessor->getPropertyValueT<int>(OB_PROP_IR_BRIGHTNESS_INT);
        customPresets_[presetName].colorAutoExposure          = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
        customPresets_[presetName].colorExposureTime          = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_EXPOSURE_INT);
        customPresets_[presetName].colorAutoWhiteBalance      = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
        customPresets_[presetName].colorWhiteBalance          = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_WHITE_BALANCE_INT);
        customPresets_[presetName].colorGain                  = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_GAIN_INT);
        customPresets_[presetName].colorContrast              = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_CONTRAST_INT);
        customPresets_[presetName].colorSaturation            = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_SATURATION_INT);
        customPresets_[presetName].colorSharpness             = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_SHARPNESS_INT);
        customPresets_[presetName].colorBrightness            = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_BRIGHTNESS_INT);
        customPresets_[presetName].colorHue                   = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_HUE_INT);
        customPresets_[presetName].colorGamma                 = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_GAMMA_INT);
        customPresets_[presetName].colorBacklightCompensation = propAccessor->getPropertyValueT<bool>(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT);
        customPresets_[presetName].colorPowerLineFrequency    = propAccessor->getPropertyValueT<int>(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);
    }

    {
        auto owner                              = getOwner();
        auto comp                               = owner->getComponent(OB_DEV_COMPONENT_ALG_PARAM_MANAGER);
        auto depthAlgModeManager                = comp.as<G330DepthAlgModeManager>();
        customPresets_[presetName].depthAlgMode = depthAlgModeManager->getCurrentDepthAlgModeChecksum().name;
    }
}

void G330PresetManager::restoreCustomPreset(std::string presetName) {
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
}

}  // namespace libobsensor