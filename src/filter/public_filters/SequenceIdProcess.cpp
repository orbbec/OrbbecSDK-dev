#include "SequenceIdProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "libobsensor/h/ObTypes.h"

namespace libobsensor {

SequenceIdFilter::SequenceIdFilter(const std::string &name) : FilterBase(name) {}
SequenceIdFilter::~SequenceIdFilter() noexcept {}

void SequenceIdFilter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 1) {
        throw invalid_value_exception("SequenceIdFilter config error: params size not match");
    }
    try {

        uint32_t select_sequence_id = std::stoi(params[0]);
        if (select_sequence_id <= 2) {
            if (select_sequence_id != select_sequence_id_) {
                std::lock_guard<std::recursive_mutex> lk(valueUpdateMutex_);
                select_sequence_id_ = select_sequence_id;
            }
        }
    }
    catch(const std::exception &e) {
        throw invalid_value_exception("SequenceIdFilter config error: " + std::string(e.what()));
    }
}

const std::string &SequenceIdFilter::getConfigSchema() const {
    // csv format: name，type， min，max，step，default，description
    static const std::string schema = "sequenceid, int, 0, 2, 1, 0, frame data sequence id value";
    return schema;
}

std::shared_ptr<Frame> SequenceIdFilter::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto outFrame = FrameFactory::createFrameFromOtherFrame(frame);
    if(outFrame->is<FrameSet>()) {
        LOG_WARN_INTVL("The Frame processed by SequenceIdFilter cannot be FrameSet!");
        return outFrame;
    }

    // steps:
    // check hdr seq id in metadata -
    // if not as the option selected id, return last frame with the selected id
    // else return current frame
    int  seq_id      = static_cast<int>(outFrame->getMetadataValue(OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX));
    auto unique_id   = outFrame->getType();
    auto current_key = std::make_pair(seq_id, unique_id);

    std::lock_guard<std::recursive_mutex> lk(valueUpdateMutex_);
    if(isSelectedId(seq_id + 1)) {
        last_frames_[current_key] = outFrame;
        return outFrame;
    }
    else {
        int  seq_id_selected      = (seq_id == 0) ? 1 : 0;
        auto key_with_selected_id = std::make_pair(seq_id_selected, unique_id);
        if(last_frames_[key_with_selected_id])
            return last_frames_[key_with_selected_id];
        return outFrame;
    }
}

bool SequenceIdFilter::isSelectedId(int stream_index) {
    if(static_cast<int>(select_sequence_id_) != 0 && stream_index != static_cast<int>(select_sequence_id_))
        return false;
    return true;
}

}  // namespace libobsensor
