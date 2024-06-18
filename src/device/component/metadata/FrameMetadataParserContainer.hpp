#pragma once

#include "IFrame.hpp"
#include "exception/ObException.hpp"
#include "utils/Utils.hpp"

#include <map>

namespace libobsensor {

class FrameMetadataParserContainer : public IFrameMetadataParserContainer {
public:
    FrameMetadataParserContainer() {}
    virtual ~FrameMetadataParserContainer() = default;

    virtual void registerParser(OBFrameMetadataType type, std::shared_ptr<IFrameMetadataParser> phaser) {
        parsers[type] = phaser;
    }

    virtual bool isContained(OBFrameMetadataType type) {
        return parsers.find(type) != parsers.end();
    }

    virtual std::shared_ptr<IFrameMetadataParser> get(OBFrameMetadataType type) {
        if(!isContained(type)) {
            throw unsupported_operation_exception(utils::to_string() << "Unsupported metadata type: " << type);
        }
        return parsers[type];
    }

protected:
    std::map<OBFrameMetadataType, std::shared_ptr<IFrameMetadataParser>> parsers;
};

}  // namespace libobsensor