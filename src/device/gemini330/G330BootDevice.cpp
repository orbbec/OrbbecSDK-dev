#include "G330BootDevice.hpp"
#include "G330Firmware.hpp"
#include "command/G330HostProtocol.hpp"
#include "core/command/MX6600VendorCommand.hpp"
#include "libobsensor/internal/InternalProperty.h"
namespace libobsensor {

G330BootDevice::G330BootDevice(std::shared_ptr<ObPal> obPal, const std::shared_ptr<DeviceInfo> info) : AbstractDevice(obPal, info) {
    LOG_DEBUG("G330BootDevice init ...");
    LOG_DEBUG("Create command start!");
    for(auto &item: deviceInfo_->sourcePortInfoList_) {
        auto portInfo = std::dynamic_pointer_cast<USBSourcePortInfo>(item);
        if(portInfo->portType == SOURCE_PORT_USB_VENDOR) {

            auto port = obPal_->createSourcePort(portInfo);
            if(port) {
                auto vendorPort   = std::dynamic_pointer_cast<VendorDataPort>(port);
                auto hostProtocol = std::make_shared<G330HostProtocol>(vendorPort, FLASH_PAGE_SIZE, HP_ERASE_FLASH_CMD_DATA_SIZE, FLASH_PAGE_SIZE);
                command_          = std::make_shared<MX6600VendorCommand>(hostProtocol);
#ifdef __ANDROID__
                auto vendorUsbPort           = std::dynamic_pointer_cast<VendorUsbDevicePort>(port);
                deviceInfo_->connectionType_ = vendorUsbPort->getUsbConnectType();
#endif
            }

            break;
        }
    }
    if(command_ == nullptr) {
        throw std::runtime_error("Create vendor command failed! Device no response or bad connection.");
    }

    BEGIN_TRY_EXECUTE({
        OBPropertyValue pidValue;
        command_->getPropertyValue(OB_PROP_PID_INT, &pidValue);
        devModePid_ = pidValue.intValue;
    })
    CATCH_EXCEPTION_AND_EXECUTE({
        LOG_WARN("Get device mode PID failed! Set to 0x0000.");
        devModePid_ = 0x0000;
    })

    LOG_DEBUG("Create command done!");
    LOG_INFO("G330BootDevice created! PID: 0x{:04x}, SN: {}", deviceInfo_->pid_, deviceInfo_->deviceSn_);
}

G330BootDevice::~G330BootDevice() {
    if(upgradeThread_.joinable()) {
        upgradeThread_.join();
    }
    LOG_INFO("G330BootDevice destroyed!");
}

std::shared_ptr<DeviceInfo> G330BootDevice::getDeviceInfo() {
    if(!fullDevInfoObtained_) {
        uint32_t      size;
        OBVersionInfo version{};

        BEGIN_TRY_EXECUTE({
            auto accessor = getPropertyAccessorForce(OB_STRUCT_VERSION);
            accessor->getFirmwareData(&version, &size);
            deviceInfo_->name_                = "Recovery Mode";
            deviceInfo_->fwVersion_           = version.firmwareVersion;
            deviceInfo_->deviceSn_            = version.serialNumber;
            deviceInfo_->asicName_            = version.depthChip;
            deviceInfo_->hwVersion_           = version.hardwareVersion;
            deviceInfo_->type_                = version.deviceType;
            deviceInfo_->supportedSdkVersion_ = version.sdkVersion;
            // Log print Firmware version info when the SDK init.
            LOG_INFO("\t- Firmware version: {}", version.firmwareVersion);
        })
        CATCH_EXCEPTION_AND_LOG(ERROR, "get device versions failed")
        fullDevInfoObtained_ = true;
    }
    return deviceInfo_;
}

void G330BootDevice::deviceUpgrade(std::string filePath, DeviceUpgradeCallback upgradeCallback, bool async) {
    std::vector<uint8_t> fileData;

    std::ifstream file(filePath, std::ios::binary);
    if(!file.is_open()) {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Failed to open file: " << filePath);
    }

    file.seekg(0, std::ios::end);
    auto fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    fileData.resize(fileSize);
    file.read((char *)fileData.data(), fileSize);
    file.close();

    deviceUpgrade((char *)fileData.data(), fileSize, upgradeCallback, async);
}

void G330BootDevice::deviceUpgrade(const char *fileData, uint32_t fileSize, DeviceUpgradeCallback upgradeCallback, bool async) {
    if(upgradeThread_.joinable()) {
        upgradeThread_.join();
    }
    auto        fwHandle     = std::make_shared<G330Firmware>((const uint8_t *)fileData, fileSize);
    const auto &fwFileHeader = fwHandle->getFileHeader();
    if(std::string(fwFileHeader.serial) != "Gemini2R" && std::string(fwFileHeader.serial) != "G300" && std::string(fwFileHeader.serial) != "Gemini 300"
       && std::string(fwFileHeader.serial) != "Gemini 330") {
        throw libobsensor::invalid_value_exception(ObUtils::to_string() << "Invalid firmware file with unmatched serial: " << fwFileHeader.serial);
    }
    const auto &firmwareDataList = fwHandle->getFirmwareDataList();
    auto        upgradeFunc      = [this, upgradeCallback, firmwareDataList]() {
        auto fwNum = firmwareDataList.size();
        auto fwIdx = 0;
        for(auto &firmwareData: firmwareDataList) {
            fwIdx++;
            if(!G330Firmware::isFirmwareDataAdaptable(firmwareData, devModePid_)) {
                continue;
            }

            auto dataSize = firmwareData.header.actualSize;
            auto address  = firmwareData.header.flash_address;
            if(address == ISP_FLASH_ADDR) {
                // isp固件为最后一个时，直接回调成功。
                if(fwIdx >= fwNum) {
                    std::string info = "Upgrade successful! Please reboot your device manually!";
                    upgradeCallback(STAT_DONE, info.c_str(), 100);
                    return;
                }
                continue;
            }
            OBUpgradeState upgradeState = ERR_OTHER;

            const int MAX_RETRY = 4;
            int       retry     = MAX_RETRY;
            do {
                if(retry != MAX_RETRY) {
                    LOG_DEBUG("Update firmware: [{0}] failed! retry {1}/{2}", firmwareData.header.name, retry, MAX_RETRY);
                }
                BEGIN_TRY_EXECUTE({
                    command_->writeFlash(
                        address, firmwareData.data.data(), dataSize,
                        [&](OBDataTranState state, uint8_t percent) {
                            std::string msg;
                            switch(state) {
                            case DATA_TRAN_STAT_DONE:
                                if(fwIdx >= fwNum) {
                                    upgradeState = STAT_DONE;
                                    msg          = "Upgrade successful! Please reboot your device manually!";
                                }
                                else {
                                    upgradeState = STAT_FILE_TRANSFER;
                                    msg          = std::string("The ") + firmwareData.header.name + " firmware upgrade done!";
                                }
                                break;
                            case DATA_TRAN_STAT_TRANSFERRING:
                                upgradeState = STAT_FILE_TRANSFER;
                                msg          = std::string("The ") + firmwareData.header.name + " firmware data transferring!";
                                break;
                            case DATA_TRAN_STAT_VERIFYING:
                                upgradeState = STAT_VERIFY_IMAGE;
                                msg          = std::string("The ") + firmwareData.header.name + " firmware data verifying!";
                                break;
                            default:
                                upgradeState = ERR_OTHER;
                                msg          = std::string("The ") + firmwareData.header.name + "Upgrade failed!";
                                break;
                            }
                            upgradeCallback(upgradeState, msg.c_str(), percent);
                        },
                        false);
                            })
                CATCH_EXCEPTION_AND_EXECUTE({ break; })
            } while(retry-- > 0 && upgradeState != STAT_DONE && upgradeState != STAT_FILE_TRANSFER);

            if(upgradeState != STAT_DONE && upgradeState != STAT_FILE_TRANSFER) {
                break;
            }
        }
    };
    upgradeThread_ = std::thread(upgradeFunc);
    if(!async) {
        upgradeThread_.join();
    }
}


}  // namespace libobsensor