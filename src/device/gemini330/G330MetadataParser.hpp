#pragma once
#include <stdint.h>

#include "IFrame.hpp"
#include "G330MetadataTypes.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {

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
        (void )metadata;
        return dataSize >= sizeof(T);
    }

private:
    Field T::       *field_;
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
        (void )metadata;
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
        (void )metadata;
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

}  // namespace libobsensor