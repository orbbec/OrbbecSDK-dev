#pragma once
#include <atomic>
#include <memory>
#include <functional>

namespace libobsensor
{
class Frame;

typedef std::function<void(std::shared_ptr<const Frame>)> FrameCallback;
typedef std::function<void(std::shared_ptr<Frame>)> FrameCallbackUnsafe;
} // namespace libobsensor


#ifdef __cplusplus
extern "C" {
#endif
struct ob_frame_t {
    std::shared_ptr<libobsensor::Frame> frame;
    std::atomic<int>                    refCnt = {1};
};
#ifdef __cplusplus
}
#endif