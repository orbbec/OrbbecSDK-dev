#pragma once
#include "FilterBase.hpp"
#include <map>

namespace libobsensor {

class HdrMerge : public FilterBase {
public:
    HdrMerge(const std::string &name);
    virtual ~HdrMerge() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    void                   discardDepthMergedFrameIfNeeded(std::shared_ptr<Frame> frame);
    std::shared_ptr<Frame> mergingAlgorithm(std::shared_ptr<const Frame> first_f, std::shared_ptr<const Frame> second_f, const bool use_ir);

protected:
    std::map<uint64_t, std::shared_ptr<Frame>> frames_;
    std::shared_ptr<Frame>                depth_merged_frame_;

};

}  // namespace libobsensor
