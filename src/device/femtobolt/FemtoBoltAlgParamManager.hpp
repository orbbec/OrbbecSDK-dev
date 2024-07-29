#pragma once
#include "param/AlgParamManagerBase.hpp"

#include <vector>
#include <memory>

namespace libobsensor {
class FemtoBoltAlgParamManager : public AlgParamManagerBase {
public:
    FemtoBoltAlgParamManager(IDevice *owner);
    virtual ~FemtoBoltAlgParamManager() noexcept = default;

private:
    void fetchParams() override;
    void registerBasicExtrinsics() override;
};
}  // namespace libobsensor