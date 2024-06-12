#pragma once
#include "FilterBase.hpp"
#include <mutex>

namespace libobsensor {

class FrameMirror : public FilterBase {
public:
    FrameMirror(const std::string &name);
    virtual ~FrameMirror() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    static OBCameraIntrinsic  mirrorOBCameraIntrinsic(const OBCameraIntrinsic &src);
    static OBCameraDistortion mirrorOBCameraDistortion(const OBCameraDistortion &src);
protected:
    std::shared_ptr<const StreamProfile> srcStreamProfile_;
    std::shared_ptr<VideoStreamProfile>  rstStreamProfile_;
};

}  // namespace libobsensor
