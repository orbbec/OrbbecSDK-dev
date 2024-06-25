#ifdef __cplusplus
extern "C" {
#endif

#include "ObTypes.h"

/**
 * @brief Get the current depth work mode.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_depth_work_mode The current depth work mode.
 */
OB_EXPORT ob_depth_work_mode ob_device_get_current_depth_work_mode(ob_device *device, ob_error **error);

/**
 * @brief Switch the depth work mode by ob_depth_work_mode.
 *        Prefer to use ob_device_switch_depth_work_mode_by_name to switch depth mode when the complete name of the depth work mode is known.
 *
 * @param[in] device The device object.
 * @param[in] work_mode The depth work mode from ob_depth_work_mode_list which is returned by ob_device_get_depth_work_mode_list.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
OB_EXPORT ob_status ob_device_switch_depth_work_mode(ob_device *device, const ob_depth_work_mode *work_mode, ob_error **error);

/**
 * @brief Switch the depth work mode by work mode name.
 *
 * @param[in] device The device object.
 * @param[in] mode_name The depth work mode name which is equal to ob_depth_work_mode.name.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_status The switch result. OB_STATUS_OK: success, other failed.
 */
OB_EXPORT ob_status ob_device_switch_depth_work_mode_by_name(ob_device *device, const char *mode_name, ob_error **error);

/**
 * @brief Request the list of supported depth work modes.
 *
 * @param[in] device The device object.
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 * @return ob_depth_work_mode_list The list of ob_depth_work_mode.
 */
OB_EXPORT ob_depth_work_mode_list *ob_device_get_depth_work_mode_list(ob_device *device, ob_error **error);

/**
 * \if English
 * @brief Get the depth work mode count that ob_depth_work_mode_list hold
 * @param[in] work_mode_list data struct contain list of ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return The total number contained in ob_depth_work_mode_list
 *
 */
OB_EXPORT uint32_t ob_depth_work_mode_list_count(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @brief Get the index target of ob_depth_work_mode from work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[in] index Index of the target ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 * @return ob_depth_work_mode
 *
 */
OB_EXPORT ob_depth_work_mode ob_depth_work_mode_list_get_item(ob_depth_work_mode_list *work_mode_list, uint32_t index, ob_error **error);

/**
 * @brief Free the resources of ob_depth_work_mode_list
 *
 * @param[in] work_mode_list Data structure containing a list of ob_depth_work_mode
 * @param[out] error Pointer to an error object that will be set if an error occurs.
 *
 */
OB_EXPORT void ob_delete_depth_work_mode_list(ob_depth_work_mode_list *work_mode_list, ob_error **error);

/**
 * @breif Get the current preset name.
 * @brief The preset mean a set of parameters or configurations that can be applied to the device to achieve a specific effect or function.
 *
 * @param device The device object.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The current preset name, it should be one of the preset names returned by @ref ob_device_get_available_preset_list.
 */
OB_EXPORT const char *ob_device_get_current_preset_name(ob_device *device, ob_error **error);

/**
 * @brief Get the available preset list.
 * @attention After loading the preset, the settings in the preset will set to the device immediately. Therefore, it is recommended to re-read the device
 * settings to update the user program temporarily.
 *
 * @param device The device object.
 * @param preset_name  Pointer to an error object that will be set if an error occurs. The name should be one of the preset names returned by @ref
 * ob_device_get_available_preset_list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_load_preset(ob_device *device, const char *preset_name, ob_error **error);

/**
 * @brief Load preset from json string.
 * @brief After loading the custom preset, the settings in the custom preset will set to the device immediately.
 * @brief After loading the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_load_preset_from_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Export current settings as a preset json file.
 * @brief After exporting the custom preset, the available preset list will be appended with the custom preset and named as the file name.
 *
 * @param device   The device object.
 * @param json_file_path  The json file path.
 * @param error   Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_device_export_current_settings_as_preset_json_file(ob_device *device, const char *json_file_path, ob_error **error);

/**
 * @brief Get the available preset list.
 *
 * @param device The device object.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The available preset list.
 */
OB_EXPORT ob_device_preset_list *ob_device_get_available_preset_list(ob_device *device, ob_error **error);

/**
 * @brief Delete the available preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 */
OB_EXPORT void ob_delete_preset_list(ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the number of preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The number of preset in the preset list.
 */
OB_EXPORT uint32_t ob_device_preset_list_count(ob_device_preset_list *preset_list, ob_error **error);

/**
 * @brief Get the name of the preset in the preset list.
 *
 * @param preset_list The available preset list.
 * @param index  The index of the preset in the preset list.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  The name of the preset in the preset list.
 */
OB_EXPORT const char *ob_device_preset_list_get_name(ob_device_preset_list *preset_list, uint32_t index, ob_error **error);

/**
 * @brief Check if the preset list has the preset.
 *
 * @param preset_list The available preset list.
 * @param preset_name  The name of the preset.
 * @param error  Pointer to an error object that will be set if an error occurs.
 * @return  Whether the preset list has the preset. If true, the preset list has the preset. If false, the preset list does not have the preset.
 */
OB_EXPORT bool ob_device_preset_list_has_preset(ob_device_preset_list *preset_list, const char *preset_name, ob_error **error);

#ifdef __cplusplus
}
#endif