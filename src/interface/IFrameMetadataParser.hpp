#pragma once
#include <stdint.h>
#include <memory>
#include <map>

#include <openobsdk/h/ObTypes.h>

namespace ob {

class IFrameMetadataParser {
public:
    virtual ~IFrameMetadataParser()                                   = default;
    virtual int64_t getValue(const uint8_t *metadata, uint32_t dataSize)    = 0;
    virtual bool    isSupported(const uint8_t *metadata, uint32_t dataSize) = 0;
};

class IFrameMetadataParserContainer {
public:
    IFrameMetadataParserContainer() {}
    virtual ~IFrameMetadataParserContainer()                                                                                             = default;
    virtual void                                  registerParser(OBFrameMetadataType type, std::shared_ptr<IFrameMetadataParser> phaser) = 0;
    virtual bool                                  isContained(OBFrameMetadataType type)                                                  = 0;
    virtual std::shared_ptr<IFrameMetadataParser> get(OBFrameMetadataType type)                                                          = 0;
};

}  // namespace ob