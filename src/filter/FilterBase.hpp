#pragma once
#include "IFilter.hpp"
#include "frame/FrameQueue.hpp"
#include "stream/StreamProfile.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <map>

namespace libobsensor {

class FilterBase : public IFilter {
public:
    FilterBase(const std::string &name);
    virtual ~FilterBase() noexcept;
    virtual const std::string &getName() const override;

    virtual void reset() override;
    virtual void enable(bool en) override;
    virtual bool isEnabled() const override;

    const std::vector<OBFilterConfigSchemaItem> &getConfigSchemaVec() override;
    void                                         setConfigValue(const std::string &name, double value) override;
    double                                       getConfigValue(const std::string &name) override;

    // Synchronize
    virtual std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) override;

    // Asynchronous, output result to callback function
    virtual void pushFrame(std::shared_ptr<const Frame> frame) override;
    virtual void setCallback(FilterCallback cb) override;

protected:
    virtual std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) = 0;  // Filter function function, implemented on child class

private:
    void checkAndUpdateConfig();

protected:
    const std::string name_;
    std::atomic<bool> enabled_;

    std::mutex processMutex_;

    std::mutex     callbackMutex_;
    FilterCallback callback_;

    std::shared_ptr<FrameQueue<const Frame>> srcFrameQueue_;

    std::mutex                            configMutex_;
    std::atomic<bool>                     configChanged_;
    std::map<std::string, double>         configMap_;
    std::vector<OBFilterConfigSchemaItem> configSchemaVec_;
    std::vector<std::vector<std::string>> configSchemaStrSplittedVec_;
};

}  // namespace libobsensor