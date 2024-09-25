#include "FirmwareUpdater.hpp"
#include "environment/EnvConfig.hpp"
#include "exception/ObException.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {
// FirmwareUpdaterFactory::FirmwareUpdaterFactory(IDevice* owner) : DeviceComponentBase(owner) {
//     std::string moduleLoadPath = EnvConfig::getExtensionsDirectory() + "/firmwareupdater/";

//     context_         = std::make_shared<FirmwareUpdateContext>();
//     context_->dylib_ = std::make_shared<dylib>(moduleLoadPath.c_str(), "OrbbecSDKExt");

//     if(context_->dylib_) {
//         // context_->write_ahb = context_->dylib_->get_function<void(ob_device *, uint32_t, uint32_t, uint32_t, ob_error **)>("ob_device_write_ahb");
//         // context_->read_ahb  = context_->dylib_->get_function<void(ob_device *, uint32_t, uint32_t, uint32_t *, ob_error **)>("ob_device_read_ahb");
//         // context_->write_i2c = context_->dylib_->get_function<void(ob_device *, uint32_t, uint32_t, uint32_t, uint32_t, ob_error
//         **)>("ob_device_write_i2c");
//         // context_->read_i2c  = context_->dylib_->get_function<void(ob_device *, uint32_t, uint32_t, uint32_t, uint32_t *, ob_error
//         **)>("ob_device_read_i2c");
//         // context_->write_flash =
//         //     context_->dylib_->get_function<void(ob_device *, uint32_t, const void *, uint32_t, ob_set_data_callback, bool, void *, ob_error **)>(
//         //         "ob_device_write_flash");
//         // context_->read_flash =
//         //     context_->dylib_->get_function<void(ob_device *, uint32_t, uint32_t, ob_get_data_callback, bool, void *, ob_error
//         **)>("ob_device_read_flash"); context_->update_firmware_ext =
//             context_->dylib_->get_function<void(ob_device *, const char *, ob_device_fw_update_callback, bool, void *, ob_error **)>(
//                 "ob_device_update_firmware_ext");
//         context_->update_firmware_from_raw_data_ext =
//             context_->dylib_->get_function<void(ob_device *, const uint8_t *, uint32_t, ob_device_fw_update_callback, bool, void *, ob_error **)>(
//                 "ob_device_update_firmware_from_raw_data_ext");
//     }
//     else {

//     }
// }

// std::shared_ptr<FirmwareUpdater> FirmwareUpdaterFactory::createFirmwareUpdater(IDevice *owner) {
//     if (!context_ || (context_ && !context_->dylib_)) {
//         return nullptr;
//     }

//     auto owner = getOwner();

//     return std::make_shared<FirmwareUpdater>(owner, context_);
// }

FirmwareUpdater::FirmwareUpdater(IDevice *owner) : DeviceComponentBase(owner) {
    std::string moduleLoadPath = EnvConfig::getExtensionsDirectory() + "/firmwareupdater/";
    try {
        ctx_                      = std::make_shared<FirmwareUpdateContext>();
        ctx_->dylib_              = std::make_shared<dylib>(moduleLoadPath.c_str(), "OrbbecSDKExt");
        ctx_->update_firmware_ext = ctx_->dylib_->get_function<void(ob_device *, const char *, ob_device_fw_update_callback, bool, void *, ob_error **)>(
            "ob_device_update_firmware_ext");
        ctx_->update_firmware_from_raw_data_ext =
            ctx_->dylib_->get_function<void(ob_device *, const uint8_t *, uint32_t, ob_device_fw_update_callback, bool, void *, ob_error **)>(
                "ob_device_update_firmware_from_raw_data_ext");
    }
    catch(const std::exception &e) {
        LOG_DEBUG("Failed to load OrbbecSDKExt library: {}", e.what());
    }
}

FirmwareUpdater::~FirmwareUpdater() noexcept {}

DeviceFwUpdateCallback& FirmwareUpdater::getDeviceFwUpdateCallback() {
    static DeviceFwUpdateCallback deviceFwUpdateCallback_ = nullptr;
    return deviceFwUpdateCallback_;

}
void FirmwareUpdater::onDeviceFwUpdateCallback(ob_fw_update_state state, const char *message, uint8_t percent, void *user_data) {
    (void)user_data;
    auto deviceFwUpdateCallback_ = getDeviceFwUpdateCallback();
    if (deviceFwUpdateCallback_) {
        deviceFwUpdateCallback_(state, message, percent);
    }
}

void FirmwareUpdater::setDeviceFwUpdateCallback(DeviceFwUpdateCallback callback) {
    auto deviceFwUpdateCallback_ = getDeviceFwUpdateCallback();
    deviceFwUpdateCallback_ = callback;
}
void FirmwareUpdater::updateFirmwareExt(const std::string& path, DeviceFwUpdateCallback callback, bool async, void *user_data) {
    ob_error  *error  = nullptr;
    ob_device *c_device = new ob_device();
    c_device->device    = getOwner()->shared_from_this();
    setDeviceFwUpdateCallback(callback);
    ctx_->update_firmware_ext(c_device, path.c_str(), onDeviceFwUpdateCallback, async, user_data, &error);
    delete c_device;
    if(error) {
        throw libobsensor_exception(std::string(error->message), error->exception_type);
    }
}

void FirmwareUpdater::updateFirmwareFromRawDataExt(const uint8_t *firmwareData, uint32_t firmwareSize, DeviceFwUpdateCallback callback, bool async, void *userData) {
    ob_error  *error  = nullptr;
    ob_device *c_device = new ob_device();
    c_device->device    = getOwner()->shared_from_this();
    setDeviceFwUpdateCallback(callback);
    ctx_->update_firmware_from_raw_data_ext(c_device, firmwareData, firmwareSize, onDeviceFwUpdateCallback, async, userData, &error);
    delete c_device;
    if(error) {
        throw libobsensor_exception(std::string(error->message), error->exception_type);
    }
}

}  // namespace libobsensor