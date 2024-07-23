#pragma once
#include "libobsensor/h/ObTypes.h"

#include "IStreamProfile.hpp"

#include <memory>

namespace libobsensor {

class ISensorStreamStrategy {
public:
    virtual ~ISensorStreamStrategy() = default;

    virtual void validateStartStream(const std::shared_ptr<const StreamProfile> &profile)               = 0;
    virtual void validateStartStream(const std::vector<std::shared_ptr<const StreamProfile>> &profiles) = 0;
    virtual void markStreamStarted(const std::shared_ptr<const StreamProfile> &profile)                 = 0;
    virtual void markStreamStopped(const std::shared_ptr<const StreamProfile> &profile)                 = 0;
};

}  // namespace libobsensor