#pragma once
#include "ISensorStreamStrategy.hpp"
#include "G330DepthAlgModeManager.hpp"
#include "core/stream/StreamProfile.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "component/DeviceComponentBase.hpp"
#include <map>

namespace libobsensor {

class G330SensorStreamStrategy : public ISensorStreamStrategy, public DeviceComponentBase {
public:
    G330SensorStreamStrategy(IDevice *owner);
    virtual ~G330SensorStreamStrategy() noexcept;

    void validateStartStream(const std::shared_ptr<const StreamProfile> &profile) override;
    void validateStartStream(const std::vector<std::shared_ptr<const StreamProfile>> &profiles) override;
    void markStreamStarted(const std::shared_ptr<const StreamProfile> &profile) override;
    void markStreamStopped(const std::shared_ptr<const StreamProfile> &profile) override;

private:
    void validateDepthAndIrStream(const std::vector<std::shared_ptr<const StreamProfile>> &profiles);
    void validatePreset(const std::vector<std::shared_ptr<const StreamProfile>> &profiles);

private:
    std::mutex                                        startedStreamListMutex_;
    std::vector<std::shared_ptr<const StreamProfile>> startedStreamList_;
};

}  // namespace libobsensor