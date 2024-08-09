#pragma once
#include "FilterBase.hpp"
#include <mutex>
#include <map>

namespace libobsensor {

class SequenceIdFilter : public FilterBase {
public:
    SequenceIdFilter(const std::string &name);
    virtual ~SequenceIdFilter() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    std::recursive_mutex valueUpdateMutex_;

    // selected id; 0 for all
    int32_t             selectedID_ = -1;

    std::map<std::pair<int64_t, OBFrameType>, std::shared_ptr<Frame>> recentFrames_;
};

}  // namespace libobsensor
