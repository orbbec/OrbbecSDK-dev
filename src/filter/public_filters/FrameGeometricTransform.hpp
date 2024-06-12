#pragma once
#include "FilterBase.hpp"
#include <mutex>
#include <thread>
#include <atomic>

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


class FrameFlip : public FilterBase {
public:
    FrameFlip(const std::string &name);
    virtual ~FrameFlip() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    static OBCameraIntrinsic  flipOBCameraIntrinsic(const OBCameraIntrinsic &src);
    static OBCameraDistortion flipOBCameraDistortion(const OBCameraDistortion &src);

protected:
    std::shared_ptr<const StreamProfile> srcStreamProfile_;
    std::shared_ptr<VideoStreamProfile>  rstStreamProfile_;
};


class FrameRotate : public FilterBase {
public:
    FrameRotate(const std::string &name);
    virtual ~FrameRotate() noexcept;

    void               updateConfig(std::vector<std::string> &params) override;
    const std::string &getConfigSchema() const override;

private:
    std::shared_ptr<Frame> processFunc(std::shared_ptr<const Frame> frame) override;

    static OBCameraIntrinsic  rotateOBCameraIntrinsic(const OBCameraIntrinsic &src, uint32_t rotateDegree);
    static OBCameraDistortion rotateOBCameraDistortion(const OBCameraDistortion &src, uint32_t rotateDegree);
    static OBExtrinsic        rotateOBExtrinsic(uint32_t rotateDegree);

protected:
    std::mutex                           mtx_;
    uint32_t                             rotateDegree_ = 0;
    std::atomic<bool>                    rotateDegreeUpdated_;
    std::shared_ptr<const StreamProfile> srcStreamProfile_;
    std::shared_ptr<VideoStreamProfile>  rstStreamProfile_;
};
}  // namespace libobsensor
