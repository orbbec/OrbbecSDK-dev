#pragma once
#include "IDevice.hpp"
#include "InternalTypes.hpp"
#include "IDepthWorkModeManager.hpp"
#include "DeviceComponentBase.hpp"

namespace libobsensor {

class G2DepthWorkModeManager : public IDepthWorkModeManager, public DeviceComponentBase {
public:
    G2DepthWorkModeManager(IDevice *owner);
    virtual ~G2DepthWorkModeManager() = default;

    std::vector<OBDepthWorkModeChecksum> getDepthWorkModeList() const override;
    const OBDepthWorkModeChecksum       &getCurrentDepthWorkModeChecksum() const override;
    void                                 switchDepthWorkMode(const std::string &modeName) override;

private:
    void switchDepthWorkMode(const OBDepthWorkModeChecksum &targetDepthMode);

private:
    std::vector<OBDepthWorkModeChecksum> depthWorkModeChecksumList_;
    OBDepthWorkModeChecksum              currentWorkMode_;
};

}  // namespace libobsensor