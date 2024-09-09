#pragma once

#include "metadata/FrameMetadataParserContainer.hpp"
#include "G330MetadataTypes.hpp"

namespace libobsensor {
class G330FrameMetadataParserContainer : public FrameMetadataParserContainer {
public:
    G330FrameMetadataParserContainer(IDevice *device) : device_(device) {}

    void registerParser(OBFrameMetadataType type, std::shared_ptr<IFrameMetadataParser> phaser) override {
        for(const auto &pair: idMap_) {
            auto propertyId = pair.first;
            auto types      = pair.second;
            if(std::find(types.begin(), types.end(), type) == types.end()) {
                continue;
            }
            if(std::find(syncProperties_.begin(), syncProperties_.end(), propertyId) != syncProperties_.end()) {
                continue;
            }
            syncProperties_.push_back(propertyId);
        }
        FrameMetadataParserContainer::registerParser(type, phaser);
    }

    void syncProperties() {
        auto               propertyServer = device_->getPropertyServer();
        OBPropertyValue    value          = {};
        PropertyAccessType accessType     = PROP_ACCESS_USER;
        for(const auto &property: syncProperties_) {
            if(property == OB_STRUCT_COLOR_AE_ROI || property == OB_STRUCT_DEPTH_AE_ROI || property == OB_STRUCT_DEPTH_HDR_CONFIG) {
                propertyServer->getStructureData(property, accessType);
            }
            else {
                propertyServer->getPropertyValue(property, &value, accessType);
            }
        }
    }

protected:
    std::multimap<OBPropertyID, std::vector<OBFrameMetadataType>> idMap_;

private:
    IDevice *device_;

    std::vector<OBPropertyID> syncProperties_;
};

class G330ColorFrameMetadataParserContainer : public G330FrameMetadataParserContainer {
public:
    G330ColorFrameMetadataParserContainer(IDevice *device) : G330FrameMetadataParserContainer(device) {
        idMap_ = initMetadataTypeIdMap(OB_SENSOR_COLOR);
    };
};

class G330DepthFrameMetadataParserContainer : public G330FrameMetadataParserContainer {
public:
    G330DepthFrameMetadataParserContainer(IDevice *device) : G330FrameMetadataParserContainer(device) {
        idMap_ = initMetadataTypeIdMap(OB_SENSOR_DEPTH);
    }
};

}  // namespace libobsensor