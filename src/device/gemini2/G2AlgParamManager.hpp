#pragma once

#include "param/AlgParamManagerBase.hpp"

namespace libobsensor {
class G2AlgParamManager : public DisparityAlgParamManagerBase {
public:
    G2AlgParamManager(IDevice *owner);
    virtual ~G2AlgParamManager() = default;

    void fetchParams() override;
    void registerBasicExtrinsics() override;

private:
    std::vector<OBDepthCalibrationParam> depthCalibParamList_;
};
}  // namespace libobsensor