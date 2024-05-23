#pragma once

#include "frame/Frame.hpp"

#include <queue>

namespace libobsensor{


class FrameQueue : public std::enable_shared_from_this<FrameQueue> {
public:
    explicit FrameQueue(size_t capacity);
    ~FrameQueue() noexcept;

    size_t size();
    bool   empty();
    bool   enqueue(std::shared_ptr<Frame> frame);  // returns false if queue is full

    // blocking methods
    std::shared_ptr<Frame> dequeue(uint64_t timeoutMsec = 0);  // returns nullptr if timeout is reached

    // async methods
    void start(std::function<void(std::shared_ptr<Frame>)> callback);  // start async dequeue
    void flush();                                                      // stop until all frames are called back
    void stop();                                                       // stop immediately

    void clear();  // clear all frames in queue, flags, and callback. Stop dequeue thread

private:
    std::mutex                         mutex_;
    std::condition_variable            condition_;
    std::queue<std::shared_ptr<Frame>> queue_;
    const size_t                       capacity_;

    std::thread                                 dequeueThread_;
    std::function<void(std::shared_ptr<Frame>)> callback_;
    std::atomic<bool>                           stoped_;
    std::atomic<bool>                           flushing_;
};


}  // namespace ob