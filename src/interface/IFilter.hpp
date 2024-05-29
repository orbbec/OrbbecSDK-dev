#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace libobsensor {
class Frame;

typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;
class IFilter {
public:
    virtual ~IFilter() noexcept = default;

    virtual const std::string &getName() const = 0;

    // Config
    virtual void        updateConfig(std::vector<std::string> &params) = 0;
    virtual std::string getConfigSchema() const                        = 0;

    virtual void reset()           = 0;  // Stop thread, clean memory, reset status
    virtual void enable(bool en)   = 0;
    virtual bool isEnabled() const = 0;

    // Synchronize
    virtual std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) = 0;

    // Asynchronous
    virtual void pushFrame(std::shared_ptr<const Frame> frame) = 0;
    virtual void setCallback(FilterCallback cb)                = 0;
};

class IFilterCreator {
public:
    virtual ~IFilterCreator() noexcept = default;

    virtual std::shared_ptr<IFilter> create() = 0;
};

class IPrivFilterCreator : public IFilterCreator {
public:
    virtual ~IPrivFilterCreator() noexcept = default;

    virtual std::shared_ptr<IFilter> create(const std::string &activationKey) = 0;
    virtual std::string              getVendorSpecificCode() const = 0;
};

}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif
struct ob_filter {
    std::shared_ptr<libobsensor::IFilter> filter;
};
#ifdef __cplusplus
}
#endif