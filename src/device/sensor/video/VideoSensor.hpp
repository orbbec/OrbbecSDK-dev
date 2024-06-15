// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "sensor/SensorBase.hpp"
#include "IFilter.hpp"

#include <map>

namespace libobsensor {

enum class FormatFilterPolicy {
    REMOVE,   // Directly remove the stream profile if the format matches the src format
    REPLACE,  // Replace the stream profile with a new format stream profile. If there is a frame format converter available, use it to convert the frame to the
              // new format
    ADD,      // Use the frame format converter to convert the frame to the new format and add a new format stream profile to the list
};

struct FormatFilterConfig {
    FormatFilterPolicy                policy;
    OBFormat                          srcFormat;
    OBFormat                          dstFormat;
    std::shared_ptr<IFormatConverter> converter;
};

class VideoSensor : public SensorBase {
public:
    VideoSensor(const std::shared_ptr<IDevice> &owner, OBSensorType sensorType, const std::shared_ptr<ISourcePort> &backend);

    ~VideoSensor() noexcept override = default;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) override;
    void stop() override;


    StreamProfileList getStreamProfileList() const override;

    void updateFormatFilterConfig(const std::vector<FormatFilterConfig> &configs);

private:
    std::vector<FormatFilterConfig>                 formatFilterConfigs_;
    std::vector<FormatFilterConfig>::const_iterator currentFormatFilterConfig_;
    std::shared_ptr<const StreamProfile>            currentBackendStreamProfile_;
    StreamProfileList                               filteredStreamProfileList_;

    typedef std::pair<std::shared_ptr<const StreamProfile>, std::vector<FormatFilterConfig>::const_iterator> StreamProfileFilterConfigMapValue;
    std::map<std::shared_ptr<const StreamProfile>, StreamProfileFilterConfigMapValue>                        streamProfileFilterConfigMap_;
};

}  // namespace libobsensor
