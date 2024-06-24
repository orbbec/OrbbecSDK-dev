#include "libobsensor/h/advanced.h"
#include "ImplTypes.hpp"
#include "exception/ObException.hpp"

#include "IPresetManager.hpp"
#include "IDevice.hpp"

#ifdef __cplusplus
extern "C" {
#endif

const char *ob_device_get_current_preset_name(ob_device *device, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(device);
    auto presetMgr = device->device->getPresetManager();
    return presetMgr->getCurrentPresetName().c_str();
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, device)

void ob_device_load_preset(ob_device *device, const char *preset_name, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(device);
    VALIDATE_NOT_NULL(preset_name);
    auto presetMgr = device->device->getPresetManager();
    presetMgr->loadPreset(preset_name);
}
HANDLE_EXCEPTIONS_NO_RETURN(device, preset_name)

void ob_device_load_preset_from_json_file(ob_device *device, const char *json_file_path, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(device);
    VALIDATE_NOT_NULL(json_file_path);
    auto presetMgr = device->device->getPresetManager();
    presetMgr->loadPresetFromJsonFile(json_file_path);
}
HANDLE_EXCEPTIONS_NO_RETURN(device, json_file_path)

void ob_device_export_current_settings_as_preset_json_file(ob_device *device, const char *json_file_path, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(device);
    VALIDATE_NOT_NULL(json_file_path);
    auto presetMgr = device->device->getPresetManager();
    presetMgr->exportSettingsAsPresetJsonFile(json_file_path);
}
HANDLE_EXCEPTIONS_NO_RETURN(device, json_file_path)

ob_device_preset_list *ob_device_get_available_preset_list(ob_device *device, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(device);
    auto presetMgr  = device->device->getPresetManager();
    auto presetList = presetMgr->getAvailablePresetList();

    auto impl        = new ob_device_preset_list();
    impl->presetList = presetList;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, device)

void ob_delete_preset_list(ob_device_preset_list *preset_list, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(preset_list);
    delete preset_list;
}
HANDLE_EXCEPTIONS_NO_RETURN(preset_list)

uint32_t ob_device_preset_list_count(ob_device_preset_list *preset_list, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(preset_list);
    return static_cast<uint32_t>(preset_list->presetList.size());
}
HANDLE_EXCEPTIONS_AND_RETURN(0, preset_list)

const char *ob_device_preset_list_get_name(ob_device_preset_list *preset_list, uint32_t index, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(preset_list);
    VALIDATE_UNSIGNED_INDEX(index, preset_list->presetList.size());
    return preset_list->presetList.at(index).c_str();
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, preset_list, index)

bool ob_device_preset_list_has_preset(ob_device_preset_list *preset_list, const char *preset_name, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(preset_list);
    VALIDATE_NOT_NULL(preset_name);
    for(auto &name: preset_list->presetList) {
        if(name == preset_name) {
            return true;
        }
    }
    return false;
}
HANDLE_EXCEPTIONS_AND_RETURN(false, preset_list, preset_name)

#ifdef __cplusplus
}
#endif