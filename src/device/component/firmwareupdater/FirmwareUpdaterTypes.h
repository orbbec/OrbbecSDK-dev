#pragma once
#include "libobsensor/h/Device.h"
#include "libobsensor/h/Error.h"

#ifdef __cplusplus
extern "C" {
#endif

// /**
//  * @brief Write to an AHB register.
//  *
//  * @param[in] device The device object.
//  * @param reg The register to be written.
//  * @param mask The mask.
//  * @param value The value to be written.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_write_ahb)(ob_device *device, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

// /**
//  * @brief Read an AHB register.
//  *
//  * @param[in] device The device object.
//  * @param reg The register to be read.
//  * @param mask The mask.
//  * @param value The value to be read.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_read_ahb)(ob_device *device, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

// /**
//  * @brief Write to an I2C register.
//  *
//  * @param[in] device The device object.
//  * @param module_id The I2C module id to be written.
//  * @param reg The register to be written.
//  * @param mask The mask.
//  * @param value The value to be written.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_write_i2c)(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t value, ob_error **error);

// /**
//  * @brief Read an I2C register.
//  *
//  * @param[in] device The device object.
//  * @param module_id The id of the I2C module to be read.
//  * @param reg The register to be read.
//  * @param mask The mask.
//  * @param value The value to be read.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_read_i2c)(ob_device *device, uint32_t module_id, uint32_t reg, uint32_t mask, uint32_t *value, ob_error **error);

// /**
//  * @brief Set the properties of writing to Flash [Asynchronous Callback].
//  *
//  * @param[in] device The device object.
//  * @param offset The flash offset address.
//  * @param data The property data to be written.
//  * @param data_size The size of the property to be written.
//  * @param cb The set data callback.
//  * @param[in] async Whether to execute asynchronously.
//  * @param[in] user_data User-defined data that will be returned in the callback.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_write_flash)(ob_device *device, uint32_t offset, const void *data, uint32_t data_size, ob_set_data_callback cb, bool async,
//                                             void *user_data, ob_error **error);

// /**
//  * @brief Read Flash properties [asynchronous callback].
//  *
//  * @param[in] device The device object.
//  * @param offset The flash offset address.
//  * @param data_size The size of the data to be read.
//  * @param cb The read flash data and progress callback.
//  * @param[in] async Whether to execute asynchronously.
//  * @param[in] user_data User-defined data that will be returned in the callback.
//  * @param[out] error Log error messages.
//  */
// typedef void (*pfunc_ob_device_read_flash)(ob_device *device, uint32_t offset, uint32_t data_size, ob_get_data_callback cb, bool async, void *user_data,
//                                            ob_error **error);

typedef void (*pfunc_ob_device_update_firmware_ext)(ob_device *device, const char *path, ob_device_fw_update_callback callback, bool async, void *user_data,
                                                    ob_error **error);

typedef void (*pfunc_ob_device_update_firmware_from_raw_data_ext)(ob_device *device, const uint8_t *firmware_data, uint32_t firmware_size,
                                                                  ob_device_fw_update_callback callback, bool async, void *user_data, ob_error **error);

#ifdef __cplusplus
}
#endif