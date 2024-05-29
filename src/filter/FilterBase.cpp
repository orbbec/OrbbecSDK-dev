#include "FilterBase.hpp"
#include "frame/FrameFactory.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
namespace libobsensor {

FilterBase::FilterBase(const std::string &name) : name_(name), enabled_(true) {
    srcFrameQueue_ = std::make_shared<FrameQueue<const Frame>>(10);  // todo： read from config file to set the size of frame queue
    LOG_DEBUG("Filter {} created with frame queue capacity {}", name_, srcFrameQueue_->size());
}

FilterBase::~FilterBase() noexcept {
    srcFrameQueue_->stop();
    LOG_DEBUG("Filter {} destroyed", name_);
}

const std::string &FilterBase::getName() const {
    return name_;
}

std::shared_ptr<Frame> FilterBase::process(std::shared_ptr<const Frame> frame) {
    if(!enabled_){
        return FrameFactory::cloneFrame(frame);
    }
    std::unique_lock<std::mutex> lock(mutex_);
    return processFunc(frame);
}

void FilterBase::pushFrame(std::shared_ptr<const Frame> frame) {
    if(!srcFrameQueue_->isStarted()) {
        srcFrameQueue_->start([&](std::shared_ptr<const Frame>) {
            std::shared_ptr<Frame> rstFrame;
            if(enabled_){
                std::unique_lock<std::mutex> lock(mutex_);
                BEGIN_TRY_EXECUTE({ rstFrame = processFunc(frame); })
                CATCH_EXCEPTION_AND_EXECUTE({  // catch all exceptions to avoid crashing on the inner thread
                    LOG_WARN("Filter {}: exception caught while processing frame {}#{}, this frame will be dropped", name_, frame->getType(), frame->getNumber());
                    return;
                })
            }else{
                rstFrame = FrameFactory::cloneFrame(frame);
            }
            std::unique_lock<std::mutex> lock(callbackMutex_);
            if(callback_) {
                callback_(rstFrame);
            }
        });
        LOG_DEBUG("Filter {}: start frame queue", name_);
    }
    srcFrameQueue_->enqueue(frame);
}

void FilterBase::setCallback(FilterCallback cb) {
    std::unique_lock<std::mutex> lock(callbackMutex_);
    callback_ = cb;
}

void FilterBase::reset() {
    srcFrameQueue_->flush();
    srcFrameQueue_->clear();
    LOG_DEBUG("Filter {}: reset frame queue", name_);
}

void FilterBase::enable(bool en) {
    enabled_ = en;
}

bool FilterBase::isEnabled() const {
    return enabled_;
}

}  // namespace libobsensor