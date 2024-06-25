#include "G330DepthAlgModeManager.hpp"
#include "property/InternalProperty.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

G330DepthAlgModeManager::G330DepthAlgModeManager(DeviceResourceGetter<IPropertyAccessor> &propertyAccessorGetter)
    : propertyAccessorGetter_(propertyAccessorGetter) {

    auto propAccessor = propertyAccessorGetter_.get();

    depthAlgModeChecksumList_ = propAccessor->getStructureDataListProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_RAW_DATA_DEPTH_ALG_MODE_LIST);
    currentAlgMode_           = propAccessor->getStructureDataProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE);
}

std::vector<OBDepthAlgModeChecksum> G330DepthAlgModeManager::getDepthAlgModeList() const {
    return depthAlgModeChecksumList_;
}

const OBDepthAlgModeChecksum &G330DepthAlgModeManager::getCurrentDepthAlgModeChecksum() const {
    return currentAlgMode_;
}

void G330DepthAlgModeManager::switchDepthAlgMode(const char *modeName) {
    OBDepthAlgModeChecksum dstMode;
    memset(&dstMode, 0, sizeof(dstMode));
    for(auto &mode: depthAlgModeChecksumList_) {
        if(strcmp(mode.name, modeName) == 0) {
            dstMode = mode;
            break;
        }
    }

    if(strlen(dstMode.name) == 0) {
        std::string totalNames;
        for(auto &mode: depthAlgModeChecksumList_) {
            if(!totalNames.empty()) {
                totalNames = totalNames + ",";
            }
            totalNames = totalNames + std::string(mode.name);
        }
        throw unsupported_operation_exception(
            std::string("Invalid depth mode: " + std::string(modeName) + ", support depth work mode list: " + totalNames).c_str());
    }

    switchDepthAlgMode(dstMode);
}

void G330DepthAlgModeManager::switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode) {
    // todo: check stream status
    // for(auto it: sensorEntryList_) {
    //     auto sensor = it.second.sensor;
    //     if(sensor && sensor->isStreamStarted()) {
    //         std::ostringstream ss;
    //         ss << "Cannot switch depth work mode while stream is started. Please stop stream first! sensor " << sensor->getSensorType() << " is streaming";
    //         throw unsupported_operation_exception(ss.str());
    //     }
    // }

    if(strncmp(currentAlgMode_.name, targetDepthMode.name, sizeof(targetDepthMode.name)) == 0) {
        LOG_DEBUG("switchDepthWorkMode done with same mode: {1}", currentAlgMode_.name, targetDepthMode.name);
        return;
    }

    auto propAccessor = propertyAccessorGetter_.get();
    propAccessor->setStructureDataProtoV1_1_T<OBDepthAlgModeChecksum, 0>(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, targetDepthMode);
    currentAlgMode_ = targetDepthMode;
    LOG_DEBUG("switchDepthWorkMode done with mode: {1}", currentAlgMode_.name, targetDepthMode.name);
}

}  // namespace libobsensor