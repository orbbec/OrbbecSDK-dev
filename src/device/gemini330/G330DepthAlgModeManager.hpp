#pragma once
#include "IDevice.hpp"
#include "InternalTypes.hpp"
#include "component/DeviceComponentBase.hpp"

namespace libobsensor {

class G330DepthAlgModeManager : public DeviceComponentBase {
public:
    G330DepthAlgModeManager(std::shared_ptr<IDevice> owner);
    virtual ~G330DepthAlgModeManager() = default;

    std::vector<OBDepthAlgModeChecksum> getDepthAlgModeList() const;
    const OBDepthAlgModeChecksum       &getCurrentDepthAlgModeChecksum() const;
    void                                switchDepthAlgMode(const char *modeName);
    void                                switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode);

private:
    std::vector<OBDepthAlgModeChecksum> depthAlgModeChecksumList_;
    OBDepthAlgModeChecksum              currentAlgMode_;
};

}  // namespace libobsensor