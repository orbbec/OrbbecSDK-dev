#include "G2DepthWorkModeManager.hpp"
#include "property/InternalProperty.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

G2DepthWorkModeManager::G2DepthWorkModeManager(IDevice *owner) : DeviceComponentBase(owner) {

    auto propServer = owner->getPropertyServer();

    depthWorkModeChecksumList_ = propServer->getStructureDataListProtoV1_1_T<OBDepthWorkModeChecksum, 0>(OB_RAW_DATA_DEPTH_ALG_MODE_LIST);
    currentWorkMode_           = propServer->getStructureDataProtoV1_1_T<OBDepthWorkModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE);
}

std::vector<OBDepthWorkModeChecksum> G2DepthWorkModeManager::getDepthWorkModeList() const {
    return depthWorkModeChecksumList_;
}

const OBDepthWorkModeChecksum &G2DepthWorkModeManager::getCurrentDepthWorkModeChecksum() const {
    return currentWorkMode_;
}

void G2DepthWorkModeManager::switchDepthWorkMode(const std::string &modeName) {
    auto iter = std::find_if(depthWorkModeChecksumList_.begin(), depthWorkModeChecksumList_.end(),
                             [&modeName](const OBDepthWorkModeChecksum &mode) { return modeName == mode.name; });

    if(iter == depthWorkModeChecksumList_.end()) {
        std::string totalNames;
        std::for_each(depthWorkModeChecksumList_.begin(), depthWorkModeChecksumList_.end(), [&totalNames](const OBDepthWorkModeChecksum &mode) {
            if(!totalNames.empty()) {
                totalNames += ",";
            }
            totalNames += mode.name;
        });
        throw unsupported_operation_exception("Invalid depth mode: " + modeName + ", support depth work mode list: " + totalNames);
    }

    OBDepthWorkModeChecksum dstMode = *iter;
    switchDepthWorkMode(dstMode);
}

void G2DepthWorkModeManager::switchDepthWorkMode(const OBDepthWorkModeChecksum &targetDepthMode) {
    auto owner = getOwner();
    {
        auto propServer = owner->getPropertyServer();  // get property server first to lock resource to avoid start stream at the same time

        if(owner->hasAnySensorStreamActivated()) {
            throw unsupported_operation_exception(utils::string::to_string()
                                                  << "Cannot switch depth work mode while any stream is started. Please stop all stream first!");
        }

        if(strncmp(currentWorkMode_.name, targetDepthMode.name, sizeof(targetDepthMode.name)) == 0) {
            LOG_DEBUG("switchDepthWorkMode done with same mode: {1}", currentWorkMode_.name, targetDepthMode.name);
            return;
        }

        propServer->setStructureDataProtoV1_1_T<OBDepthWorkModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, targetDepthMode);
        currentWorkMode_ = targetDepthMode;
    }

    LOG_INFO("Device depth work mode have been switch to: {1}, device will be reinitialize to apply the new mode.", targetDepthMode.name);
    owner->reset();
}

}  // namespace libobsensor