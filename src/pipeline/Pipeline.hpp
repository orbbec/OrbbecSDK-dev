// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "IPipeline.hpp"
#include "IDevice.hpp"
#include "IFrame.hpp"
#include "frame/FrameQueue.hpp"
#include "Config.hpp"
#include "FrameAggregator.hpp"
#include "context/Context.hpp"

namespace libobsensor {
class Config;
class Pipeline {
public:
    Pipeline(std::shared_ptr<IDevice> dev);
    virtual ~Pipeline() noexcept;

    void                         start(std::shared_ptr<const Config> cfg = nullptr);
    void                         start(std::shared_ptr<const Config> cfg, FrameCallback callback);
    StreamProfileList            getEnabledStreamProfileList();
    void                         stop();
    std::shared_ptr<IDevice>     getDevice();
    std::shared_ptr<const Frame> waitForFrame(uint32_t timeout_ms = 1000);

    StreamProfileList getD2CDepthProfileList(std::shared_ptr<const StreamProfile> colorProfile, OBAlignMode alignMode);

    void enableFrameSync();
    void disableFrameSync();

    std::shared_ptr<const Config> getConfig();

private:
    inline void startStream();
    inline void stopStream();

    void onFrameCallback(std::shared_ptr<const Frame> frame);
    void outputFrame(std::shared_ptr<const Frame> frame);

    void loadDefaultConfig();
    void loadFrameQueueSizeConfig();

    void configAlignMode();
    void resetAlignMode();

    std::shared_ptr<const VideoStreamProfile> getCurrentVideoStreamProfile(OBStreamType type);

    std::shared_ptr<Config> checkAndSetConfig(std::shared_ptr<const Config> cfg);

private:
    OBStreamState streamState_;
    std::mutex    streamMutex_;

    std::shared_ptr<const Config> config_;
    std::shared_ptr<IDevice>      device_;

    std::shared_ptr<FrameQueue<const Frame>> outputFrameQueue_;
    FrameCallback                            pipelineCallback_;

    std::shared_ptr<FrameAggregator> frameAggregator_;


    int MAX_FRAME_QUEUE_SIZE = 10;

    std::shared_ptr<Context> context_;
};

}  // namespace libobsensor
