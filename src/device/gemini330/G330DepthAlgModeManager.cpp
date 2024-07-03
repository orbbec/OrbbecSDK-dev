#include "G330DepthAlgModeManager.hpp"
#include "property/InternalProperty.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

G330DepthAlgModeManager::G330DepthAlgModeManager(IDevice *owner) : DeviceComponentBase(owner) {

    auto propAccessor = owner->getPropertyAccessor();

    depthAlgModeChecksumList_ = propAccessor->getStructureDataListProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_RAW_DATA_DEPTH_ALG_MODE_LIST);
    currentAlgMode_           = propAccessor->getStructureDataProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE);
}

std::vector<OBDepthAlgModeChecksum> G330DepthAlgModeManager::getDepthAlgModeList() const {
    return depthAlgModeChecksumList_;
}

const OBDepthAlgModeChecksum &G330DepthAlgModeManager::getCurrentDepthAlgModeChecksum() const {
    return currentAlgMode_;
}

void G330DepthAlgModeManager::switchDepthAlgMode(const std::string &modeName) {
    auto iter = std::find_if(depthAlgModeChecksumList_.begin(), depthAlgModeChecksumList_.end(),
                             [&modeName](const OBDepthAlgModeChecksum &mode) { return modeName == mode.name; });

    if(iter == depthAlgModeChecksumList_.end()) {
        std::string totalNames;
        std::for_each(depthAlgModeChecksumList_.begin(), depthAlgModeChecksumList_.end(), [&totalNames](const OBDepthAlgModeChecksum &mode) {
            if(!totalNames.empty()) {
                totalNames += ",";
            }
            totalNames += mode.name;
        });
        throw unsupported_operation_exception("Invalid depth mode: " + modeName + ", support depth work mode list: " + totalNames);
    }

    OBDepthAlgModeChecksum dstMode = *iter;
    switchDepthAlgMode(dstMode);
}

void G330DepthAlgModeManager::switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode) {
    auto owner        = getOwner();
    auto propAccessor = owner->getPropertyAccessor();  // get property accessor first to lock resource to avoid start stream at the same time

    if(owner->hasAnySensorStreamActivated()) {
        throw unsupported_operation_exception(utils::string::to_string()
                                              << "Cannot switch depth work mode while any stream is started. Please stop all stream first!");
    }

    if(strncmp(currentAlgMode_.name, targetDepthMode.name, sizeof(targetDepthMode.name)) == 0) {
        LOG_DEBUG("switchDepthWorkMode done with same mode: {1}", currentAlgMode_.name, targetDepthMode.name);
        return;
    }

    propAccessor->setStructureDataProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, targetDepthMode);
    currentAlgMode_ = targetDepthMode;
    LOG_DEBUG("switchDepthWorkMode done with mode: {1}", currentAlgMode_.name, targetDepthMode.name);
}

}  // namespace libobsensor