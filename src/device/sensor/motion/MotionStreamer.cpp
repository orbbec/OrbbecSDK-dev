#include "MotionStreamer.hpp"
#include "frame/Frame.hpp"
#include "stream/StreamProfile.hpp"

namespace libobsensor {
MotionStreamer::MotionStreamer(const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter> &dataPhaser)
    : backend_(backend), dataPhaser_(dataPhaser), running_(false) {
    dataPhaser_->setCallback([this](std::shared_ptr<Frame> frame) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto                        format = frame->getFormat();
        for(auto &callback: callbacks_) {
            if(format != callback.first->getFormat()) {
                continue;
            }
            callback.second(frame);
        }
    });
}

MotionStreamer::~MotionStreamer() noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    callbacks_.clear();

    if(running_) {
        auto dataStreamPort = std::dynamic_pointer_cast<IDataStreamPort>(backend_);
        dataStreamPort->stopStream();
        dataPhaser_->reset();
        running_ = false;
    }
}

void MotionStreamer::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    std::lock_guard<std::mutex> lock(mtx_);
    callbacks_[sp] = callback;
    if(running_) {
        return;
    }

    running_ = true;

    backend_->startStream([&](std::shared_ptr<Frame> frame) { dataPhaser_->pushFrame(frame); });
}

void MotionStreamer::stop(std::shared_ptr<const StreamProfile> sp) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto                        iter = callbacks_.find(sp);
    if(iter == callbacks_.end()) {
        throw invalid_value_exception("Stop stream failed, stream profile not found.");
    }

    callbacks_.erase(iter);
    if(!callbacks_.empty()) {
        return;
    }

    backend_->stopStream();
    dataPhaser_->reset();
    running_ = false;
}

}  // namespace libobsensor