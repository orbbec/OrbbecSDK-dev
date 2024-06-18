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

    bool isSelectedId(int stream_index);

protected:
    std::recursive_mutex valueUpdateMutex_;
    uint32_t             select_sequence_id_ = 0;

    std::map<std::pair<int, int>, std::shared_ptr<Frame>> last_frames_;
};

}  // namespace libobsensor
