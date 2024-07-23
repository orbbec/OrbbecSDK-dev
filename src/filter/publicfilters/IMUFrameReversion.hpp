#pragma once
#include "FilterBase.hpp"
#include <mutex>

namespace libobsensor {

class IMUFrameReversion : public FilterBase {
public:
    IMUFrameReversion(const std::string &name);
    virtual ~IMUFrameReversion() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
};

}  // namespace libobsensor
