#pragma once
#include <stdint.h>

#include "IFrame.hpp"
#include "G330MetadataTypes.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

static const std::multimap<OBPropertyID, std::vector<OBFrameMetadataType>> initMetadataTypeIdMap() {
    std::multimap<OBPropertyID, std::vector<OBFrameMetadataType>> map;

    map.insert({ OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, { OB_FRAME_METADATA_TYPE_EXPOSURE } });
    map.insert({ OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, { OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE } });
    map.insert({ OB_PROP_COLOR_WHITE_BALANCE_INT, { OB_FRAME_METADATA_TYPE_WHITE_BALANCE } });
    map.insert({ OB_PROP_COLOR_BRIGHTNESS_INT, { OB_FRAME_METADATA_TYPE_BRIGHTNESS } });
    map.insert({ OB_PROP_COLOR_CONTRAST_INT, { OB_FRAME_METADATA_TYPE_CONTRAST } });
    map.insert({ OB_PROP_COLOR_SATURATION_INT, { OB_FRAME_METADATA_TYPE_SATURATION } });
    map.insert({ OB_PROP_COLOR_SHARPNESS_INT, { OB_FRAME_METADATA_TYPE_SHARPNESS } });
    map.insert({ OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, { OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION } });
    map.insert({ OB_PROP_COLOR_HUE_INT, { OB_FRAME_METADATA_TYPE_HUE } });
    map.insert({ OB_PROP_COLOR_GAMMA_INT, { OB_FRAME_METADATA_TYPE_GAMMA } });
    map.insert({ OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, { OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY } });
    map.insert({ OB_STRUCT_COLOR_AE_ROI,
                 { OB_FRAME_METADATA_TYPE_AE_ROI_LEFT, OB_FRAME_METADATA_TYPE_AE_ROI_TOP, OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT,
                   OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM } });

    return map;
}

static const std::multimap<OBPropertyID, std::vector<OBFrameMetadataType>> metadataTypeIdMap = initMetadataTypeIdMap();

template <typename T, typename Field> class StructureMetadataParser : public IFrameMetadataParser {
public:
    StructureMetadataParser(Field T::*field, FrameMetadataModifier mod) : field_(field), modifier_(mod){};
    virtual ~StructureMetadataParser() = default;

    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            throw unsupported_operation_exception("Current metadata does not contain this structure!");
        }
        auto value = static_cast<int64_t>((*reinterpret_cast<const T *>(metadata)).*field_);
        if(modifier_) {
            value = modifier_(value);
        }
        return value;
    }

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        (void)metadata;
        return dataSize >= sizeof(T);
    }

private:
    Field T::            *field_;
    FrameMetadataModifier modifier_;
};

template <typename T, typename Field>
std::shared_ptr<StructureMetadataParser<T, Field>> makeStructureMetadataParser(Field T::*field, FrameMetadataModifier mod = nullptr) {
    return std::make_shared<StructureMetadataParser<T, Field>>(field, mod);
}

template <typename T> class G330MetadataTimestampParser : public IFrameMetadataParser {
public:
    G330MetadataTimestampParser(){};
    virtual ~G330MetadataTimestampParser() = default;

    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md = reinterpret_cast<const T *>(metadata);
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec;
    }

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        (void)metadata;
        return dataSize >= sizeof(T);
    }
};

// for depth and ir sensor
class G330MetadataSensorTimestampParser : public IFrameMetadataParser {
public:
    G330MetadataSensorTimestampParser(){};
    explicit G330MetadataSensorTimestampParser(FrameMetadataModifier exp_to_usec) : exp_to_usec_(exp_to_usec){};
    virtual ~G330MetadataSensorTimestampParser() = default;

    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md          = reinterpret_cast<const G330CommonUvcMetadata *>(metadata);
        auto exp_in_usec = exp_to_usec_ ? exp_to_usec_(md->exposure) : md->exposure;
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec - exp_in_usec / 2;
    }

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        (void)metadata;
        return dataSize >= sizeof(G330CommonUvcMetadata);
    }

private:
    FrameMetadataModifier exp_to_usec_ = nullptr;
};

class G330ColorMetadataSensorTimestampParser : public IFrameMetadataParser {
public:
    G330ColorMetadataSensorTimestampParser(){};
    explicit G330ColorMetadataSensorTimestampParser(FrameMetadataModifier exp_to_usec) : exp_to_usec_(exp_to_usec){};
    virtual ~G330ColorMetadataSensorTimestampParser() = default;

    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md = reinterpret_cast<const G330ColorUvcMetadata *>(metadata);
        // auto exp_in_usec = exp_to_usec_ ? exp_to_usec_(md->exposure) : md->exposure;
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec + md->sensor_timestamp_offset_usec;
    }

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        utils::unusedVar(metadata);
        return dataSize >= sizeof(G330CommonUvcMetadata);
    }

private:
    FrameMetadataModifier exp_to_usec_ = nullptr;
};

class G330ScrMetadataParserBase : public IFrameMetadataParser {
public:
    G330ScrMetadataParserBase(){};
    virtual ~G330ScrMetadataParserBase() = default;

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        utils::unusedVar(metadata);
        return dataSize >= sizeof(StandardUvcFramePayloadHeader::scrSourceClock);
    }
};

class G330ColorScrMetadataExposureParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain color exposure!");
            return 0;
        }
        auto     standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint16_t exposure            = standardUvcMetadata.scrSourceClock[0] | ((standardUvcMetadata.scrSourceClock[1] & 0b00000111) << 8);

        return static_cast<int64_t>(exposure);
    }
};

class G330ColorScrMetadataTimestampOffsetParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain color timestamp offset!");
            return 0;
        }
        auto    standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint8_t timestampOffset     = ((standardUvcMetadata.scrSourceClock[1] & 0b11111000) >> 3);
        return static_cast<int64_t>(timestampOffset);
    }
};

class G330ColorScrMetadataGainParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain color gain!");
            return 0;
        }
        auto     standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint16_t gain                = standardUvcMetadata.scrSourceClock[2];

        return static_cast<int64_t>(gain);
    }
};

class G330ColorMetadataParser : public IFrameMetadataParser {
public:
    G330ColorMetadataParser(IDevice *device, OBFrameMetadataType type) {
        for(const auto &item: metadataTypeIdMap) {
            const std::vector<OBFrameMetadataType> &types = item.second;
            if(std::find(types.begin(), types.end(), type) != types.end()) {
                propertyId_ = item.first;
                break;
            }
        }

        auto propertyServer = device->getPropertyServer();
        propertyServer->registerAccessCallback(propertyId_,
                                               [this, type](uint32_t propertyId, const uint8_t *data, size_t dataSize, PropertyOperationType operationType) {
                                                   utils::unusedVar(dataSize);
                                                   utils::unusedVar(operationType);
                                                   if(propertyId != static_cast<uint32_t>(propertyId_)) {
                                                       return;
                                                   }
                                                   if(propertyId == OB_STRUCT_COLOR_AE_ROI) {
                                                       auto roi = *(reinterpret_cast<const OBRegionOfInterest *>(data));
                                                       if(type == OB_FRAME_METADATA_TYPE_AE_ROI_LEFT) {
                                                           data_ = reinterpret_cast<void *>(&roi.x0_left);
                                                       }
                                                       else if(type == OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT) {
                                                           data_ = reinterpret_cast<void *>(&roi.x1_right);
                                                       }
                                                       else if(type == OB_FRAME_METADATA_TYPE_AE_ROI_TOP) {
                                                           data_ = reinterpret_cast<void *>(&roi.y0_top);
                                                       }
                                                       else if(type == OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM) {
                                                           data_ = reinterpret_cast<void *>(&roi.y1_bottom);
                                                       }
                                                   }
                                                   else {
                                                       data_ = reinterpret_cast<void *>(&data);
                                                   }
                                               });
    };

    virtual ~G330ColorMetadataParser() = default;

    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }

        auto value = *(reinterpret_cast<int64_t *>(data_));
        return value;
    }

    bool isSupported(const uint8_t *metadata, size_t dataSize) override {
        utils::unusedVar(metadata);
        utils::unusedVar(dataSize);
        return true;
    }

private:
    OBPropertyID propertyId_;

    void *data_;
};

class G330DepthScrMetadataExposureParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }
        auto     standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint32_t exposure =
            standardUvcMetadata.scrSourceClock[0] | (standardUvcMetadata.scrSourceClock[1] << 8) | ((standardUvcMetadata.scrSourceClock[2] & 0b00000011) << 16);

        return static_cast<int64_t>(exposure);
    }
};

class G330DepthScrMetadataLaserStatusParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }
        auto    standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint8_t laserStatus         = (standardUvcMetadata.scrSourceClock[2] & 0b00000100) >> 2;
        return static_cast<int64_t>(laserStatus);
    }
};

class G330DepthScrMetadataLaserPowerLevelParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }
        auto    standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint8_t laserPowerLevel     = (standardUvcMetadata.scrSourceClock[2] & 0b00111000) >> 3;
        return static_cast<int64_t>(laserPowerLevel);
    }
};

class G330DepthScrMetadataHDRSequenceIDParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }
        auto    standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint8_t hdrSequenceId       = (standardUvcMetadata.scrSourceClock[2] & 0b11000000) >> 6;
        return static_cast<int64_t>(hdrSequenceId);
    }
};

class G330DepthScrMetadataGainParser : public G330ScrMetadataParserBase {
public:
    int64_t getValue(const uint8_t *metadata, size_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain depth exposure!");
            return 0;
        }
        auto    standardUvcMetadata = *(reinterpret_cast<const StandardUvcFramePayloadHeader *>(metadata));
        uint8_t gain                = standardUvcMetadata.scrSourceClock[3];
        return static_cast<int64_t>(gain);
    }
};

}  // namespace libobsensor