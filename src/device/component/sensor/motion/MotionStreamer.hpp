#pragma once

#include "IFilter.hpp"
#include "ISourcePort.hpp"

#include <atomic>
#include <map>
#include <mutex>

namespace libobsensor{

class MotionStreamer final {
public:
    MotionStreamer(const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter>& dataPhaser);

    ~MotionStreamer() noexcept;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback);
    void stop(std::shared_ptr<const StreamProfile> sp);

private:
    std::shared_ptr<IDataStreamPort> backend_;
    std::shared_ptr<IFilter> dataPhaser_;

    std::mutex mtx_;
    std::map<std::shared_ptr<const StreamProfile>, FrameCallback> callbacks_;

    std::atomic_bool running_;
};

}