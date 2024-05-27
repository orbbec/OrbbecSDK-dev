#pragma once
#include "IFilter.hpp"
#include "frame/FrameQueue.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace libobsensor {

class FilterBase : public IFilter {
public:
    FilterBase(const std::string &name);
    virtual ~FilterBase() noexcept;
    const std::string &getName() const override;

    void reset() override;

    // Synchronize
    std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) override;

    // Asynchronous, output result to callback function
    void pushFrame(std::shared_ptr<const Frame> frame) override;
    void setCallback(FilterCallback cb) override;

protected:
    virtual std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) = 0;  // Filter function function, implemented on child class

protected:
    const std::string name_;

    std::mutex mutex_;

    std::mutex     callbackMutex_;
    FilterCallback callback_;

    std::shared_ptr<FrameQueue<const Frame>> srcFrameQueue_;
};
}  // namespace libobsensor