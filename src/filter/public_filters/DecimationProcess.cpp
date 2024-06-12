#include "DecimationProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "openobsdk/h/ObTypes.h"

#define PIX_SORT(a, b)          \
    {                           \
        if((a) > (b))           \
            PIX_SWAP((a), (b)); \
    }
#define PIX_SWAP(a, b)          \
    {                           \
        pixelvalue temp = (a);  \
        (a)             = (b);  \
        (b)             = temp; \
    }
#define PIX_MIN(a, b) ((a) > (b)) ? (b) : (a)
#define PIX_MAX(a, b) ((a) > (b)) ? (a) : (b)

namespace libobsensor {

/*----------------------------------------------------------------------------
Function :   opt_med3()
In       :   pointer to array of 3 pixel values
Out      :   a pixelvalue
Job      :   optimized search of the median of 3 pixel values
Notice   :   found on sci.image.processing
cannot go faster unless assumptions are made
on the nature of the input signal.
---------------------------------------------------------------------------*/
template <class pixelvalue> inline pixelvalue opt_med3(pixelvalue *p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[0], p[1]);
    return p[1];
}

/*
** opt_med4()
hacked version
**/
template <class pixelvalue> inline pixelvalue opt_med4(pixelvalue *p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[0], p[2]);
    PIX_SORT(p[1], p[3]);
    return PIX_MIN(p[1], p[2]);
}

/*----------------------------------------------------------------------------
Function :   opt_med5()
In       :   pointer to array of 5 pixel values
Out      :   a pixelvalue
Job      :   optimized search of the median of 5 pixel values
Notice   :   found on sci.image.processing
cannot go faster unless assumptions are made
on the nature of the input signal.
---------------------------------------------------------------------------*/
template <class pixelvalue> inline pixelvalue opt_med5(pixelvalue *p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[1] = PIX_MIN(p[1], p[4]);
    PIX_SORT(p[1], p[2]);
    p[2] = PIX_MIN(p[2], p[3]);
    return PIX_MAX(p[1], p[2]);
}

/*----------------------------------------------------------------------------
Function :   opt_med6()
In       :   pointer to array of 6 pixel values
Out      :   a pixelvalue
Job      :   optimized search of the median of 6 pixel values
Notice   :   from Christoph_John@gmx.de
based on a selection network which was proposed in
"FAST, EFFICIENT MEDIAN FILTERS WITH EVEN LENGTH WINDOWS"
J.P. HAVLICEK, K.A. SAKADY, G.R.KATZ
If you need larger even length kernels check the paper
---------------------------------------------------------------------------*/
template <class pixelvalue> inline pixelvalue opt_med6(pixelvalue *p) {
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    p[4] = PIX_MIN(p[4], p[5]);
    p[2] = PIX_MAX(p[1], p[2]);
    p[3] = PIX_MIN(p[3], p[4]);
    return PIX_MIN(p[2], p[3]);
}

/*----------------------------------------------------------------------------
Function :   opt_med7()
In       :   pointer to array of 7 pixel values
Out      :   a pixelvalue
Job      :   optimized search of the median of 7 pixel values
Notice   :   found on sci.image.processing
cannot go faster unless assumptions are made
on the nature of the input signal.
---------------------------------------------------------------------------*/
template <class pixelvalue> inline pixelvalue opt_med7(pixelvalue *p) {
    PIX_SORT(p[0], p[5]);
    PIX_SORT(p[0], p[3]);
    PIX_SORT(p[1], p[6]);
    PIX_SORT(p[2], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[5]);
    PIX_SORT(p[2], p[6]);
    p[3] = PIX_MAX(p[2], p[3]);
    p[3] = PIX_MIN(p[3], p[6]);
    p[4] = PIX_MIN(p[4], p[5]);
    PIX_SORT(p[1], p[4]);
    p[3] = PIX_MAX(p[1], p[3]);
    return PIX_MIN(p[3], p[4]);
}

/*----------------------------------------------------------------------------
Function :   opt_med9()
Hacked version of opt_med9()
*/
template <class pixelvalue> inline pixelvalue opt_med8(pixelvalue *p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[5], p[6]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    p[4] = PIX_MIN(p[4], p[7]);
    PIX_SORT(p[3], p[6]);
    p[5] = PIX_MAX(p[2], p[5]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[1] = PIX_MIN(p[1], p[4]);
    p[3] = PIX_MIN(p[3], p[6]);
    PIX_SORT(p[3], p[1]);
    p[3] = PIX_MAX(p[5], p[3]);
    return PIX_MIN(p[3], p[1]);
}

/*----------------------------------------------------------------------------
Function :   opt_med9()
In       :   pointer to an array of 9 pixelvalues
Out      :   a pixelvalue
Job      :   optimized search of the median of 9 pixelvalues
Notice   :   in theory, cannot go faster without assumptions on the
signal.
Formula from:
XILINX XCELL magazine, vol. 23 by John L. Smith

The input array is modified in the process
The result array is guaranteed to contain the median
value
in middle position, but other elements are NOT sorted.
---------------------------------------------------------------------------*/
template <class pixelvalue> inline pixelvalue opt_med9(pixelvalue *p) {
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[7], p[8]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[7], p[8]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[5] = PIX_MIN(p[5], p[8]);
    PIX_SORT(p[4], p[7]);
    p[6] = PIX_MAX(p[3], p[6]);
    p[4] = PIX_MAX(p[1], p[4]);
    p[2] = PIX_MIN(p[2], p[5]);
    p[4] = PIX_MIN(p[4], p[7]);
    PIX_SORT(p[4], p[2]);
    p[4] = PIX_MAX(p[6], p[4]);
    return PIX_MIN(p[4], p[2]);
}

DecimationFilter::DecimationFilter(const std::string &name)
    : FilterBase(name),
      decimation_factor_(2),
      control_val_(2),
      patch_size_(2),
      kernel_size_(patch_size_ * patch_size_),
      real_width_(0),
      real_height_(0),
      padded_width_(0),
      padded_height_(0),
      recalc_profile_(false),
      options_changed_(false) {}

DecimationFilter::~DecimationFilter() noexcept {}

void DecimationFilter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 1) {
        throw invalid_value_exception("DecimationFilter config error: params size not match");
    }
    try {
        uint8_t value = static_cast<uint8_t>(std::stoul(params[0]));
        if(value >= 1 && value <= 8) {
            if(value != control_val_) {
                std::lock_guard<std::recursive_mutex> lk(scaleChangedMutex_);
                // decimationRange_.cur = value;
                control_val_ = value;
                patch_size_ = decimation_factor_ = control_val_;
                kernel_size_                     = patch_size_ * patch_size_;
                options_changed_                 = true;
            }
        }
    }
    catch(const std::exception &e) {
        throw invalid_value_exception("DecimationFilter config error: " + std::string(e.what()));
    }
}

const std::string &DecimationFilter::getConfigSchema() const {
    // csv format: name，type， min，max，step，default，description
    static const std::string schema = "decimate, int, 1, 8, 1, 2, value decimate factor";
    return schema;
}

std::shared_ptr<Frame> DecimationFilter::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto outFrame = FrameFactory::cloneFrame(frame);
    if(outFrame->is<FrameSet>()) {
        LOG_WARN_INTVL("The Frame processed by DecimationFilter cannot be FrameSet!");
        return outFrame;
    }

    if(!isFrameFormatTypeSupported(outFrame->getFormat())) {
        LOG_WARN_INTVL("Unsupported decimation filter processing frame format @{}.", outFrame->getFormat());
        return outFrame;
    }

    {
        std::lock_guard<std::recursive_mutex> lk(scaleChangedMutex_);
        updateOutputProfile(outFrame);

        OBFrameType frameType   = outFrame->getType();
        OBFormat    frameFormat = outFrame->getFormat();
        auto        newOutFrame = FrameFactory::createFrameFromStreamProfile(target_stream_profile_);
        if(newOutFrame) {
            newOutFrame->copyInfo(outFrame);
            auto oldVideoFrame = outFrame->as<VideoFrame>();
            auto newViodeFrame = newOutFrame->as<VideoFrame>();

            if(frameType == OB_FRAME_DEPTH && (frameFormat == OB_FORMAT_Y16 || frameFormat == OB_FORMAT_Z16 || frameFormat == OB_FORMAT_DISP16)) {
                decimateDepth((uint16_t *)outFrame->getData(), (uint16_t *)newViodeFrame->getData(), oldVideoFrame->getWidth(), patch_size_);
            }
            else {
                decimateOthers(frameFormat, (void *)outFrame->getData(), (void *)newViodeFrame->getData(), oldVideoFrame->getWidth(), patch_size_);
            }

            // TODO: DataSize need to be reset
            // newFrame->setDataSize(padded_width_ * padded_height_ * outFrame->getBytesPerPixel());

            return newOutFrame;
        }
    }

    return outFrame;
}

bool DecimationFilter::isFrameFormatTypeSupported(OBFormat type) {
    switch(type) {
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_Y8:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        return true;
    default:
        break;
    }

    return false;
}

void DecimationFilter::updateOutputProfile(const std::shared_ptr<Frame> frame) {
    auto streamProfile = frame->getStreamProfile();
    if(options_changed_ || streamProfile.get() != source_stream_profile_.get()) {
        options_changed_       = false;
        source_stream_profile_ = streamProfile->as<VideoStreamProfile>();
        const auto pf          = registered_profiles_.find(std::make_tuple(source_stream_profile_.get(), decimation_factor_));
        if(registered_profiles_.end() != pf) {
            target_stream_profile_ = pf->second;

            // Update real/padded output frame size based on retrieved input properties
            real_width_    = (uint16_t)source_stream_profile_->getWidth() / patch_size_;
            real_height_   = (uint16_t)source_stream_profile_->getHeight() / patch_size_;
            padded_width_  = (uint16_t)target_stream_profile_->getWidth();
            padded_height_ = (uint16_t)target_stream_profile_->getHeight();
        }
        else {
            recalc_profile_ = true;
        }
    }

    // Build a new target profile for every system/filter change
    if(recalc_profile_) {
        auto source_vsp = source_stream_profile_->as<VideoStreamProfile>();

        // recalculate real/padded output frame size based on new input porperties
        real_width_  = (uint16_t)source_vsp->getWidth() / patch_size_;
        real_height_ = (uint16_t)source_vsp->getHeight() / patch_size_;

        // The resulted frame dimension will be dividable by 4;
        padded_width_ = real_width_ + 3;
        padded_width_ /= 4;
        padded_width_ *= 4;

        padded_height_ = real_height_ + 3;
        padded_height_ /= 4;
        padded_height_ *= 4;

        auto intrinsic   = source_vsp->getIntrinsic();
        intrinsic.width  = padded_width_;
        intrinsic.height = padded_height_;
        intrinsic.fx     = intrinsic.fx / patch_size_;
        intrinsic.fy     = intrinsic.fy / patch_size_;
        intrinsic.cx     = intrinsic.cx / patch_size_;
        intrinsic.cy     = intrinsic.cy / patch_size_;

        target_stream_profile_ = source_vsp->clone()->as<VideoStreamProfile>();
        target_stream_profile_->setWidth(padded_width_);
        target_stream_profile_->setHeight(padded_height_);
        // extrinsic and distortion parameters remain unchanged.
        target_stream_profile_->bindIntrinsic(intrinsic);

        registered_profiles_[std::make_tuple(source_stream_profile_.get(), decimation_factor_)] = target_stream_profile_;

        recalc_profile_ = false;
    }
}

void DecimationFilter::decimateDepth(uint16_t *frame_data_in, uint16_t *frame_data_out, size_t width_in, size_t scale) {
    // Use median filtering
    std::vector<uint16_t>   working_kernel(kernel_size_);
    auto                    wk_begin = working_kernel.data();
    auto                    wk_itr   = wk_begin;
    std::vector<uint16_t *> pixel_raws(scale);
    uint16_t               *block_start = const_cast<uint16_t *>(frame_data_in);

    if(scale == 2 || scale == 3) {
        for(int j = 0; j < real_height_; j++) {
            uint16_t *p{};
            // Mark the beginning of each of the N lines that the filter will run upon
            for(size_t i = 0; i < pixel_raws.size(); i++)
                pixel_raws[i] = block_start + (width_in * i);

            for(size_t i = 0, chunk_offset = 0; i < real_width_; i++) {
                wk_itr = wk_begin;
                // extract data the kernel to process
                for(size_t n = 0; n < scale; ++n) {
                    p = pixel_raws[n] + chunk_offset;
                    for(size_t m = 0; m < scale; ++m) {
                        if(*(p + m))
                            *wk_itr++ = *(p + m);
                    }
                }

                // For even-size kernels pick the member one below the middle
                auto ks = (int)(wk_itr - wk_begin);
                if(ks == 0)
                    *frame_data_out++ = 0;
                else {
                    switch(ks) {
                    case 1:
                        *frame_data_out++ = working_kernel[0];
                        break;
                    case 2:
                        *frame_data_out++ = PIX_MIN(working_kernel[0], working_kernel[1]);
                        break;
                    case 3:
                        *frame_data_out++ = opt_med3<uint16_t>(working_kernel.data());
                        break;
                    case 4:
                        *frame_data_out++ = opt_med4<uint16_t>(working_kernel.data());
                        break;
                    case 5:
                        *frame_data_out++ = opt_med5<uint16_t>(working_kernel.data());
                        break;
                    case 6:
                        *frame_data_out++ = opt_med6<uint16_t>(working_kernel.data());
                        break;
                    case 7:
                        *frame_data_out++ = opt_med7<uint16_t>(working_kernel.data());
                        break;
                    case 8:
                        *frame_data_out++ = opt_med8<uint16_t>(working_kernel.data());
                        break;
                    case 9:
                        *frame_data_out++ = opt_med9<uint16_t>(working_kernel.data());
                        break;
                    }
                }

                chunk_offset += scale;
            }

            // Fill-in the padded colums with zeros
            for(int k = real_width_; k < padded_width_; k++)
                *frame_data_out++ = 0;

            // Skip N lines to the beginnig of the next processing segment
            block_start += width_in * scale;
        }
    }
    else {
        for(int j = 0; j < real_height_; j++) {
            uint16_t *p{};
            // Mark the beginning of each of the N lines that the filter will run upon
            for(size_t i = 0; i < pixel_raws.size(); i++)
                pixel_raws[i] = block_start + (width_in * i);

            for(size_t i = 0, chunk_offset = 0; i < real_width_; i++) {
                int sum     = 0;
                int counter = 0;

                // extract data the kernel to process
                for(size_t n = 0; n < scale; ++n) {
                    p = pixel_raws[n] + chunk_offset;
                    for(size_t m = 0; m < scale; ++m) {
                        if(*(p + m)) {
                            sum += p[m];
                            ++counter;
                        }
                    }
                }

                *frame_data_out++ = (uint16_t)(counter == 0 ? 0 : sum / counter);
                chunk_offset += scale;
            }

            // Fill-in the padded colums with zeros
            for(int k = real_width_; k < padded_width_; k++)
                *frame_data_out++ = 0;

            // Skip N lines to the beginnig of the next processing segment
            block_start += width_in * scale;
        }
    }

    // Fill-in the padded rows with zeros
    for(auto v = real_height_; v < padded_height_; ++v) {
        for(auto u = 0; u < padded_width_; ++u)
            *frame_data_out++ = 0;
    }
}

void DecimationFilter::decimateOthers(OBFormat format, void *frame_data_in, void *frame_data_out, size_t width_in, size_t scale) {
    int  sum        = 0;
    auto patch_size = scale * scale;

    switch(format) {
    case OB_FORMAT_YUYV: {
        uint8_t *from = (uint8_t *)frame_data_in;
        uint8_t *p    = nullptr;
        uint8_t *q    = (uint8_t *)frame_data_out;

        auto w_2  = width_in >> 1;
        auto rw_2 = real_width_ >> 1;
        auto pw_2 = padded_width_ >> 1;
        auto s2   = scale >> 1;
        bool odd  = (scale & 1);
        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < rw_2; ++i) {
                p   = from + scale * (j * w_2 + i) * 4;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m * 2];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + 1;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < s2; ++m)
                        sum += 2 * p[m * 4];

                    if(odd)
                        sum += p[s2 * 4];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + s2 * 4 + (odd ? 2 : 0);
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m * 2];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + 3;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < s2; ++m)
                        sum += 2 * p[m * 4];

                    if(odd)
                        sum += p[s2 * 4];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);
            }

            for(int i = rw_2; i < pw_2; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
            }
        }
    } break;

    case OB_FORMAT_UYVY: {
        uint8_t *from = (uint8_t *)frame_data_in;
        uint8_t *p    = nullptr;
        uint8_t *q    = (uint8_t *)frame_data_out;

        auto w_2  = width_in >> 1;
        auto rw_2 = real_width_ >> 1;
        auto pw_2 = padded_width_ >> 1;
        auto s2   = scale >> 1;
        bool odd  = (scale & 1);
        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < rw_2; ++i) {
                p   = from + scale * (j * w_2 + i) * 4;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < s2; ++m)
                        sum += 2 * p[m * 4];

                    if(odd)
                        sum += p[s2 * 4];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + 1;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m * 2];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + 2;
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < s2; ++m)
                        sum += 2 * p[m * 4];

                    if(odd)
                        sum += p[s2 * 4];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);

                p   = from + scale * (j * w_2 + i) * 4 + s2 * 4 + (odd ? 3 : 1);
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m * 2];

                    p += w_2 * 4;
                }
                *q++ = (uint8_t)(sum / patch_size);
            }

            for(int i = rw_2; i < pw_2; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
            }
        }
    } break;

    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR: {
        uint8_t *from = (uint8_t *)frame_data_in;
        uint8_t *p    = nullptr;
        uint8_t *q    = (uint8_t *)frame_data_out;
        ;

        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < real_width_; ++i) {
                for(int k = 0; k < 3; ++k) {
                    p   = from + scale * (j * width_in + i) * 3 + k;
                    sum = 0;
                    for(size_t n = 0; n < scale; ++n) {
                        for(size_t m = 0; m < scale; ++m)
                            sum += p[m * 3];

                        p += width_in * 3;
                    }

                    *q++ = (uint8_t)(sum / patch_size);
                }
            }

            for(int i = real_width_; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }
    } break;

    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA: {
        uint8_t *from = (uint8_t *)frame_data_in;
        uint8_t *p    = nullptr;
        uint8_t *q    = (uint8_t *)frame_data_out;

        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < real_width_; ++i) {
                for(int k = 0; k < 4; ++k) {
                    p   = from + scale * (j * width_in + i) * 4 + k;
                    sum = 0;
                    for(size_t n = 0; n < scale; ++n) {
                        for(size_t m = 0; m < scale; ++m)
                            sum += p[m * 4];

                        p += width_in * 4;
                    }

                    *q++ = (uint8_t)(sum / patch_size);
                }
            }

            for(int i = real_width_; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i) {
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
                *q++ = 0;
            }
        }
    } break;

    case OB_FORMAT_Y8: {
        uint8_t *from = (uint8_t *)frame_data_in;
        uint8_t *p    = nullptr;
        uint8_t *q    = (uint8_t *)frame_data_out;

        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < real_width_; ++i) {
                p   = from + scale * (j * width_in + i);
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m];

                    p += width_in;
                }

                *q++ = (uint8_t)(sum / patch_size);
            }

            for(int i = real_width_; i < padded_width_; ++i)
                *q++ = 0;
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i)
                *q++ = 0;
        }
    } break;

    case OB_FORMAT_Y16: {
        uint16_t *from = (uint16_t *)frame_data_in;
        uint16_t *p    = nullptr;
        uint16_t *q    = (uint16_t *)frame_data_out;

        for(int j = 0; j < real_height_; ++j) {
            for(int i = 0; i < real_width_; ++i) {
                p   = from + scale * (j * width_in + i);
                sum = 0;
                for(size_t n = 0; n < scale; ++n) {
                    for(size_t m = 0; m < scale; ++m)
                        sum += p[m];

                    p += width_in;
                }

                *q++ = (uint16_t)(sum / patch_size);
            }

            for(int i = real_width_; i < padded_width_; ++i)
                *q++ = 0;
        }

        for(int j = real_height_; j < padded_height_; ++j) {
            for(int i = 0; i < padded_width_; ++i)
                *q++ = 0;
        }
    } break;

    default:
        break;
    }
}

}  // namespace libobsensor
