#include "FilterBase.hpp"
#include "frame/FrameQueue.hpp"
#include <mutex>

extern "C" {
#include "PrivFilterTypes.h"
}

namespace libobsensor {
class PrivFilterCppWrapper : public FilterBase {
public:
    PrivFilterCppWrapper(const std::string &filterName, ob_priv_filter_context *filterCtx);
    virtual ~PrivFilterCppWrapper() noexcept;

    // Config
    void        updateConfig(std::vector<std::string> &params) override;
    std::string getConfigSchema() const override;

    void reset() override;  // Stop thread, clean memory, reset status

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;  // Filter function function, implemented on child class

private:
    ob_priv_filter_context_t *privFilterCtx_;
};
}  // namespace libobsensor