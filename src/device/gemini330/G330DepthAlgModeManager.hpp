#pragma once
#include "IDevice.hpp"
#include "InternalTypes.hpp"
#include "IDepthAlgModeManager.hpp"
#include "DeviceComponentBase.hpp"

namespace libobsensor {

class G330DepthAlgModeManager : public IDepthAlgModeManager, public DeviceComponentBase {
public:
    G330DepthAlgModeManager(IDevice *owner);
    virtual ~G330DepthAlgModeManager() = default;

    std::vector<OBDepthAlgModeChecksum> getDepthAlgModeList() const override;
    const OBDepthAlgModeChecksum       &getCurrentDepthAlgModeChecksum() const override;
    void                                switchDepthAlgMode(const std::string &modeName) override;

private:
    void                                switchDepthAlgMode(const OBDepthAlgModeChecksum &targetDepthMode);

private:
    std::vector<OBDepthAlgModeChecksum> depthAlgModeChecksumList_;
    OBDepthAlgModeChecksum              currentAlgMode_;
};

}  // namespace libobsensor