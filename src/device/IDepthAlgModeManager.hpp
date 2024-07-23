#pragma once
#include <string>
#include <vector>
#include "InternalTypes.hpp"

namespace libobsensor {

class IDepthAlgModeManager {
public:
    virtual ~IDepthAlgModeManager() = default;

    virtual std::vector<OBDepthAlgModeChecksum> getDepthAlgModeList() const                 = 0;
    virtual const OBDepthAlgModeChecksum       &getCurrentDepthAlgModeChecksum() const      = 0;
    virtual void                                switchDepthAlgMode(const std::string &name) = 0;
};
}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif

struct ob_depth_work_mode_list_t {
    std::vector<OBDepthAlgModeChecksum> workModeList;
};

#ifdef __cplusplus
}
#endif