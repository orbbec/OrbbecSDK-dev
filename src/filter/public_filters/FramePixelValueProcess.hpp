#pragma once
#include "FilterBase.hpp"

namespace libobsensor {

class PixelValueScaler : public FilterBase {
public:
    PixelValueScaler(const std::string &name);
    virtual ~PixelValueScaler() noexcept;

public:
    void setScale(float scale) {
        scale_ = scale;
    }

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    float                                scale_ = 1.0f;
};

}  // namespace libobsensor
