#pragma once
#include "FilterBase.hpp"
#include <mutex>

namespace libobsensor {

class DisparityTransform : public FilterBase {
public:
    DisparityTransform(const std::string &name);
    virtual ~DisparityTransform() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    void updateParam();
    void dualCameraInitLookUpTable();
    void singleCameraInitLookUpTable();
    void checkDisparityConverterParams(std::shared_ptr<Frame> frame);

    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

protected:
    uint32_t  maxDepthValue_;
    uint32_t  minDepthValue_;
    float     shiftScale_;
    uint8_t   outputPixelAvailableBitSize_;
    uint32_t  lookUpTableSize_;
    uint16_t *lookUpTable_ = nullptr;  // packed disparity to depth LUT
};

}  // namespace libobsensor
