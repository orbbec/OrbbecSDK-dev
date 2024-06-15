#pragma once
#include <stdint.h>

#include "interface/IMetadataParser.hpp"
#include "G330MetadataTypes.hpp"
#include "common/exception/ObException.hpp"
#include "common/logger/LoggerInterval.hpp"

namespace libobsensor {


template <typename T, typename Field> class StructureMetadataParser : public IMetadataParser {
public:
    StructureMetadataParser(Field T::*field, MetadataModifier mod) : field_(field), modifier_(mod){};
    virtual ~StructureMetadataParser() = default;

    virtual int64_t getValue(uint8_t *metadata, uint32_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            throw libobsensor::unsupported_operation_exception(ObUtils::to_string() << "Current metadata does not contain timestamp!");
        }
        auto value = static_cast<int64_t>((*reinterpret_cast<T *>(metadata)).*field_);
        if(modifier_) {
            value = modifier_(value);
        }
        return value;
    }

    virtual bool isSupported(uint8_t *metadata, uint32_t dataSize) override {
        (void *)metadata;
        return dataSize >= sizeof(T);
    }

private:
    Field T::       *field_;
    MetadataModifier modifier_;
};

template <typename T, typename Field>
std::shared_ptr<StructureMetadataParser<T, Field>> makeStructureMetadataParser(Field T::*field, MetadataModifier mod = nullptr) {
    return std::make_shared<StructureMetadataParser<T, Field>>(field, mod);
}

template <typename T> class G330MetadataTimestampParser : public IMetadataParser {
public:
    G330MetadataTimestampParser(){};
    virtual ~G330MetadataTimestampParser() = default;

    virtual int64_t getValue(uint8_t *metadata, uint32_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md = reinterpret_cast<T *>(metadata);
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec;
    }

    virtual bool isSupported(uint8_t *metadata, uint32_t dataSize) override {
        (void *)metadata;
        return dataSize >= sizeof(T);
    }
};

// for depth and ir sensor
class G330MetadataSensorTimestampParser : public IMetadataParser {
public:
    G330MetadataSensorTimestampParser(){};
    explicit G330MetadataSensorTimestampParser(MetadataModifier exp_to_usec) : exp_to_usec_(exp_to_usec){};
    virtual ~G330MetadataSensorTimestampParser() = default;

    virtual int64_t getValue(uint8_t *metadata, uint32_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md          = reinterpret_cast<G330CommonUvcMetadata *>(metadata);
        auto exp_in_usec = exp_to_usec_ ? exp_to_usec_(md->exposure) : md->exposure;
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec - exp_in_usec / 2;
    }

    virtual bool isSupported(uint8_t *metadata, uint32_t dataSize) override {
        (void *)metadata;
        return dataSize >= sizeof(G330CommonUvcMetadata);
    }

private:
    MetadataModifier exp_to_usec_ = nullptr;
};


class G330ColorMetadataSensorTimestampParser : public IMetadataParser {
public:
    G330ColorMetadataSensorTimestampParser(){};
    explicit G330ColorMetadataSensorTimestampParser(MetadataModifier exp_to_usec) : exp_to_usec_(exp_to_usec){};
    virtual ~G330ColorMetadataSensorTimestampParser() = default;

    virtual int64_t getValue(uint8_t *metadata, uint32_t dataSize) override {
        if(!isSupported(metadata, dataSize)) {
            LOG_WARN_INTVL("Current metadata does not contain timestamp!");
            return 0;
        }
        auto md          = reinterpret_cast<G330ColorUvcMetadata *>(metadata);
        auto exp_in_usec = exp_to_usec_ ? exp_to_usec_(md->exposure) : md->exposure;
        return (int64_t)md->timestamp_sof_sec * 1000000 + md->timestamp_sof_nsec / 1000 - md->timestamp_offset_usec + md->sensor_timestamp_offset_usec;
    }

    virtual bool isSupported(uint8_t *metadata, uint32_t dataSize) override {
        (void *)metadata;
        return dataSize >= sizeof(G330CommonUvcMetadata);
    }

private:
    MetadataModifier exp_to_usec_ = nullptr;
};


}  // namespace libobsensor