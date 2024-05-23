#include "FrameQueue.hpp"

namespace libobsensor{

FrameQueue::FrameQueue(size_t capacity) : capacity_(capacity), stoped_(false), flushing_(false), callback_(nullptr) {}

FrameQueue::~FrameQueue() noexcept {
    clear();
}

size_t FrameQueue::size() {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
}

bool FrameQueue::empty() {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
}

bool FrameQueue::enqueue(std::shared_ptr<Frame> frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(queue_.size() >= capacity_ || flushing_) {
        return false;
    }
    queue_.push(frame);
    condition_.notify_one();
    return true;
}

std::shared_ptr<Frame> FrameQueue::dequeue(uint64_t timeoutMsec) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(!queue_.empty()) {
        auto result = queue_.front();
        queue_.pop();
        return result;
    }

    if(timeoutMsec == 0 || stoped_ || flushing_) {
        return nullptr;
    }

    condition_.wait_for(lock, std::chrono::milliseconds(timeoutMsec), [this] { return !queue_.empty() || stoped_ || flushing_; });
    if(queue_.empty()) {
        return nullptr;
    }
    auto result = queue_.front();
    queue_.pop();
    return result;
}

void FrameQueue::start(std::function<void(std::shared_ptr<Frame>)> callback) {
    if(callback_ != nullptr) {
        return;
    }
    callback_      = callback;
    stoped_        = false;
    flushing_      = false;
    dequeueThread_ = std::thread([&] {
        std::unique_lock<std::mutex> lock(mutex_);
        while(!stoped_) {
            condition_.wait(lock, [this] { return !queue_.empty() || stoped_ || flushing_; });
            if(queue_.empty()) {
                break;
            }
            std::shared_ptr<Frame> frame = queue_.front();
            queue_.pop();
            if(frame) {
                callback_(frame);
            }
        }
    });
}

void FrameQueue::flush() {
    flushing_ = true;
    condition_.notify_one();
    if(dequeueThread_.joinable()) {
        dequeueThread_.join();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while(!queue_.empty()) {
        queue_.pop();
    }
}

void FrameQueue::stop() {
    stoped_ = true;
    condition_.notify_one();
    if(dequeueThread_.joinable()) {
        dequeueThread_.join();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while(!queue_.empty()) {
        queue_.pop();
    }
}

void FrameQueue::clear() {
    stop(); // try stop if it's running, clear all frames on queue
    callback_ = nullptr;
    flushing_ = false;
    stoped_   = false;
}


}  // namespace ob