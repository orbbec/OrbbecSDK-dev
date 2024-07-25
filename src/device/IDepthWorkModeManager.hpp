#pragma once
#include <string>
#include <vector>
#include "InternalTypes.hpp"

namespace libobsensor {

class IDepthWorkModeManager {
public:
    virtual ~IDepthWorkModeManager() = default;

    virtual std::vector<OBDepthWorkModeChecksum> getDepthWorkModeList() const                = 0;
    virtual const OBDepthWorkModeChecksum       &getCurrentDepthWorkModeChecksum() const     = 0;
    virtual void                                switchDepthWorkMode(const std::string &name) = 0;
};
}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif

struct ob_depth_work_mode_list_t {
    std::vector<OBDepthWorkModeChecksum> workModeList;
};

#ifdef __cplusplus
}
#endif