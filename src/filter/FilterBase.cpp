#include "FilterBase.hpp"
#include "exception/ObException.hpp"
namespace libobsensor {

FilterBase::FilterBase(const std::string &name) : name_(name){
    srcFrameQueue_ = std::make_shared<FrameQueue<const Frame>>(10);  // todo： read from config file to set the size of frame queue
    LOG_DEBUG("Filter {} created with frame queue capacity {}", name_, srcFrameQueue_->size());
}

FilterBase::~FilterBase() noexcept {
    srcFrameQueue_->stop();
    LOG_DEBUG("Filter {} destroyed", name_);
}

const std::string &FilterBase::getName() const{
    return name_;
}

std::shared_ptr<Frame> FilterBase::process(std::shared_ptr<const Frame> frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    return processFunc(frame);
}

void FilterBase::pushFrame(std::shared_ptr<const Frame> frame) {
    if(!srcFrameQueue_->isStarted()) {
        srcFrameQueue_->start([&](std::shared_ptr<const Frame>) {
            std::shared_ptr<Frame> rstFrame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                BEGIN_TRY_EXECUTE({ rstFrame = processFunc(frame); })
                CATCH_EXCEPTION_AND_EXECUTE({  // catch all exceptions to avoid crashing on the inner thread
                    return;
                })
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

}  // namespace libobsensor