#pragma once
#include "IDevice.hpp"
#include "InternalTypes.hpp"

namespace libobsensor {

class G330DepthAlgModeManager {
public:
    G330DepthAlgModeManager(DeviceResourceGetter<IPropertyAccessor> &propertyAccessorGetter);
    virtual ~G330DepthAlgModeManager() = default;

    std::vector<OBDepthAlgModeChecksum> getDepthAlgModeList() const;
    const OBDepthAlgModeChecksum       &getCurrentDepthAlgModeChecksum() const;
    void                                switchDepthAlgMode(const char *modeName);
    void                                switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode);

private:
    DeviceResourceGetter<IPropertyAccessor> propertyAccessorGetter_;

    std::vector<OBDepthAlgModeChecksum> depthAlgModeChecksumList_;
    OBDepthAlgModeChecksum              currentAlgMode_;
};

}  // namespace libobsensor