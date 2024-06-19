/**
 * @file  Filter.hpp
 * @brief This file contains the Filter class, which is the processing unit of the SDK that can perform point cloud generation, format conversion, and other
 * functions.
 */
#pragma once

#include "Types.hpp"
#include "Error.hpp"
#include "Frame.hpp"
#include "openobsdk/h/Filter.h"
#include "openobsdk/h/Frame.h"
#include <functional>
#include <memory>
#include <map>
#include <string>
#include <iostream>

namespace ob {

/**
 * @brief A callback function that takes a shared pointer to a Frame object as its argument.
 */
typedef std::function<void(std::shared_ptr<Frame>)> FilterCallback;

/**
 * @brief The Filter class is the base class for all filters in the SDK.
 */
class Filter : public std::enable_shared_from_this<Filter> {
protected:
    ob_filter_t   *impl_ = nullptr;
    std::string    type_;
    FilterCallback callback_;

public:
    Filter() = default;
    Filter(ob_filter_t *impl, const char *type) : impl_(impl), type_(type) {}

    virtual ~Filter() {
        if(impl_ != nullptr) {
            ob_error *error = nullptr;
            ob_delete_filter(impl_, &error);
            Error::handle(&error, false);
        }
    }

    /**
     * @brief ReSet the filter, freeing the internal cache, stopping the processing thread, and clearing the pending buffer frame when asynchronous processing
     * is used.
     */
    virtual void reset() {
        ob_error *error = nullptr;
        ob_filter_reset(impl_, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief enable the filter
     */
    void enable(bool enable) {
        ob_error *error = nullptr;
        ob_filter_enable(impl_, enable, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Return Enable State
     */
    bool isEnabled() {
        ob_error *error  = nullptr;
        bool      enable = ob_filter_is_enable(impl_, &error);
        Error::handle(&error, true);
        return enable;
    }

    /**
     * @brief Processes a frame synchronously.
     *
     * @param frame The frame to be processed.
     * @return std::shared_ptr< Frame > The processed frame.
     */
    virtual std::shared_ptr<Frame> process(std::shared_ptr<const Frame> frame) {
        ob_error *error  = nullptr;
        auto      result = ob_filter_process(impl_, frame->getImpl(), &error);
        Error::handle(&error, true);
        return std::make_shared<Frame>(result);
    }

    /**
     * @brief Pushes the pending frame into the cache for asynchronous processing.
     *
     * @param frame The pending frame. The processing result is returned by the callback function.
     */
    virtual void pushFrame(std::shared_ptr<Frame> frame) {
        ob_error *error = nullptr;
        ob_filter_push_frame(impl_, frame->getImpl(), &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set the callback function for asynchronous processing.
     *
     * @param callback The processing result callback.
     */
    virtual void setCallBack(FilterCallback callback) {
        callback_       = callback;
        ob_error *error = nullptr;
        ob_filter_set_callback(impl_, &Filter::filterCallback, this, &error);
        Error::handle(&error, true);
    }

    static void filterCallback(ob_frame *frame, void *userData) {
        auto filter = static_cast<Filter *>(userData);
        filter->callback_(std::make_shared<Frame>(frame));
    }

    /**
     * @brief Get the type of filter.
     *
     * @return string The type of filte.
     */
    virtual const char *type() {
        return type_.c_str();
    }

    /**
     * @brief Check if the runtime type of the filter object is compatible with a given type.
     *
     * @tparam T The given type.
     * @return bool The result.
     */
    template <typename T> bool is();

    /**
     * @brief Convert the filter object to a target type.
     *
     * @tparam T The target type.
     * @return std::shared_ptr<T> The result. If it cannot be converted, an exception will be thrown.
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        return std::static_pointer_cast<T>(shared_from_this());
    }
};

/**
 * @brief The PointCloudFilter class is a subclass of Filter that generates point clouds.
 */
class PointCloudFilter : public Filter {
public:
    PointCloudFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_pointcloud_filter(&error);
        type_           = "PointCloudFilter";
        Error::handle(&error, true);
    }
    ~PointCloudFilter() override = default;

    /**
     * @brief Set the point cloud type parameters.
     *
     * @param type The point cloud type: depth point cloud or RGBD point cloud.
     */
    void setCreatePointFormat(OBFormat type) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_point_format(impl_, type, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set the camera parameters.
     *
     * @param param The camera internal and external parameters.
     */
    void setCameraParam(OBCameraParam param) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_camera_param(impl_, param, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set the frame alignment state that will be input to generate point cloud.
     *
     * @param state The alignment status. True: enable alignment; False: disable alignment.
     */
    void setFrameAlignState(bool state) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_frame_align_state(impl_, state, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set the point cloud coordinate data zoom factor.
     *
     * @attention Calling this function to set the scale will change the point coordinate scaling factor of the output point cloud frame: posScale = posScale /
     * scale. The point coordinate scaling factor for the output point cloud frame can be obtained via @ref PointsFrame::getPositionValueScale function.
     *
     * @param scale The zoom factor.
     */
    void setPositionDataScaled(float scale) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_position_data_scale(impl_, scale, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set point cloud color data normalization.
     *
     * @param state Whether normalization is required.
     */
    void setColorDataNormalization(bool state) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_color_data_normalization(impl_, state, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Set the point cloud coordinate system.
     *
     * @param type The coordinate system type.
     */
    void setCoordinateSystem(OBCoordinateSystemType type) {
        ob_error *error = nullptr;
        ob_pointcloud_filter_set_coordinate_system(impl_, type, &error);
        Error::handle(&error, true);
    }
};

/**
 * @brief The FormatConvertFilter class is a subclass of Filter that performs format conversion.
 */
class FormatConvertFilter : public Filter {
public:
    FormatConvertFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_format_convert_filter(&error);
        type_           = "FormatConverter";
        Error::handle(&error, true);
    }

    ~FormatConvertFilter() override = default;
    /**
     * @brief Set the format conversion type.
     *
     * @param type The format conversion type.
     */
    void setFormatConvertType(OBConvertFormat type) {
        ob_error *error = nullptr;
        ob_format_convert_filter_set_format(impl_, type, &error);
        Error::handle(&error, true);
    }
};

/**
 * @brief The CompressionFilter class is a subclass of Filter that performs compression.
 */
class CompressionFilter : public Filter {
public:
    CompressionFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_compression_filter(&error);
        type_           = "CompressionFilter";
        Error::handle(&error, true);
    }

    ~CompressionFilter() override = default;

    /**
     * @brief Set the compression parameters.
     *
     * @param mode The compression mode: OB_COMPRESSION_LOSSLESS or OB_COMPRESSION_LOSSY.
     * @param params The compression parameters. When mode is OB_COMPRESSION_LOSSLESS, params is NULL.
     */
    void setCompressionParams(OBCompressionMode mode, void *params) {
        ob_error *error = nullptr;
        ob_compression_filter_set_compression_params(impl_, mode, params, &error);
        Error::handle(&error, true);
    }
};

/**
 * @brief The DecompressionFilter class is a subclass of Filter that performs decompression.
 */
class DecompressionFilter : public Filter {
public:
    DecompressionFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_decompression_filter(&error);
        type_           = "DecompressionFilter";
        Error::handle(&error, true);
    }
    ~DecompressionFilter() override = default;
};

/**
 * @brief Hole filling filter,the processing performed depends on the selected hole filling mode.
 */
class HoleFillingFilter : public Filter {
public:
    HoleFillingFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_holefilling_filter(&error);
        type_           = "HoleFillingFilter";
        Error::handle(&error, true);
    }

    /**
     * @brief Set the HoleFillingFilter mode.
     *
     * @param[in] filter A holefilling_filter object.
     * @param mode OBHoleFillingMode, OB_HOLE_FILL_LEFT,OB_HOLE_FILL_NEAREST or OB_HOLE_FILL_FAREST.
     */
    void setFilterMode(OBHoleFillingMode mode) {
        ob_error *error = nullptr;
        ob_holefilling_filter_set_mode(impl_, mode, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the HoleFillingFilter mode.
     *
     * @return OBHoleFillingMode
     */
    OBHoleFillingMode getFilterMode() {
        ob_error         *error = nullptr;
        OBHoleFillingMode mode  = ob_holefilling_filter_get_mode(impl_, &error);
        Error::handle(&error, true);
        return mode;
    }
};

/**
 * @brief Temporal filter
 */
class TemporalFilter : public Filter {
public:
    TemporalFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_temporal_filter(&error);
        type_           = "TemporalFilter";
        Error::handle(&error, true);
    }

    ~TemporalFilter() override = default;

    /**
     * @brief Get the TemporalFilter diffscale range.
     *
     * @return OBFloatPropertyRange the diffscale value of property range.
     */
    OBFloatPropertyRange getDiffScaleRange() {
        ob_error            *error = nullptr;
        OBFloatPropertyRange range = ob_temporal_filter_get_diffscale_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Set the TemporalFilter diffscale value.
     *
     * @param value diffscale value.
     */
    void setDiffScale(float value) {
        ob_error *error = nullptr;
        ob_temporal_filter_set_diffscale_value(impl_, value, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the TemporalFilter weight range.
     *
     * @return OBFloatPropertyRange the weight value of property range.
     */
    OBFloatPropertyRange getWeightRange() {
        ob_error            *error = nullptr;
        OBFloatPropertyRange range = ob_temporal_filter_get_weight_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Set the TemporalFilter weight value.
     *
     * @param value weight value.
     */
    void setWeight(float value) {
        ob_error *error = nullptr;
        ob_temporal_filter_set_weight_value(impl_, value, &error);
        Error::handle(&error, true);
    }
};

/**
 * @brief Spatial advanced filte smooths the image by calculating frame with alpha and delta settings
 * alpha defines the weight of the current pixel for smoothing,
 * delta defines the depth gradient below which the smoothing will occur as number of depth levels.
 */
class SpatialAdvancedFilter : public Filter {
public:
    SpatialAdvancedFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_spatial_advanced_filter(&error);
        type_           = "SpatialAdvancedFilter";
        Error::handle(&error, true);
    }

    ~SpatialAdvancedFilter() override = default;
    /**
     * @brief Get the spatial advanced filter alpha range.
     *
     * @return OBFloatPropertyRange the alpha value of property range.
     */
    OBFloatPropertyRange getAlphaRange() {
        ob_error            *error = nullptr;
        OBFloatPropertyRange range = ob_spatial_advanced_filter_get_alpha_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the spatial advanced filter dispdiff range.
     *
     * @return OBFloatPropertyRange the dispdiff value of property range.
     */
    OBUint16PropertyRange getDispDiffRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_spatial_advanced_filter_get_disp_diff_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the spatial advanced filter radius range.
     *
     * @return OBFloatPropertyRange the radius value of property range.
     */
    OBUint16PropertyRange getRadiusRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_spatial_advanced_filter_get_radius_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the spatial advanced filter magnitude range.
     *
     * @return OBFloatPropertyRange the magnitude value of property range.
     */
    OBIntPropertyRange getMagnitudeRange() {
        ob_error          *error = nullptr;
        OBIntPropertyRange range = ob_spatial_advanced_filter_get_magnitude_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the spatial advanced filter params.
     *
     * @return OBSpatialAdvancedFilterParams
     */
    OBSpatialAdvancedFilterParams getFilterParams() {
        ob_error                     *error  = nullptr;
        OBSpatialAdvancedFilterParams params = ob_spatial_advanced_filter_get_filter_params(impl_, &error);
        Error::handle(&error, true);
        return params;
    }

    /**
     * @brief Set the spatial advanced filter params.
     *
     * @param params OBSpatialAdvancedFilterParams.
     */
    void setFilterParams(OBSpatialAdvancedFilterParams params) {
        ob_error *error = nullptr;
        ob_spatial_advanced_filter_set_filter_params(impl_, params, &error);
        Error::handle(&error, true);
    }
};

/**
 * @brief Depth to disparity or disparity to depth
 */
class DisparityTransform : public Filter {
public:
    /**
     * @brief Create a disparity transform.
     * @param depth_to_disparity disparity to depth or depth to disparity Conversion.
     */

    explicit DisparityTransform(bool depth_to_disparity) : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_disparity_transform(&error, depth_to_disparity);
        type_           = "DisparityTransform";
        Error::handle(&error, true);
    }
    ~DisparityTransform() override = default;
};

/**
 * @brief HdrMerge processing block,
 * the processing merges between depth frames with
 * different sub-preset sequence ids.
 */
class HdrMerge : public Filter {
public:
    HdrMerge() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_hdr_merge(&error);
        type_           = "HDRMerge";
        Error::handle(&error, true);
    }
};

/**
 * @brief Align for depth to other or other to depth.
 */
class Align : public Filter {
public:
    /**
     * @brief Creaet Align filter.
     * @param OBStreamType alignment is performed between a depth image and another image.
     */
    explicit Align(OBStreamType align_to_stream) : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_align(&error, align_to_stream);
        type_           = "Align";
        Error::handle(&error, true);
    }
    ~Align() override = default;

    /**
     * @brief Get the stream type to be aligned with.
     *
     * @return OBStreamType The stream type of align to.
     */
    OBStreamType getAlignToStreamType() {
        ob_error    *error      = nullptr;
        OBStreamType streamType = ob_align_get_to_stream_type(impl_, &error);
        Error::handle(&error, true);
        return streamType;
    }
};

/**
 * @brief Creates depth Thresholding filter
 * By controlling min and max options on the block
 */
class ThresholdFilter : public Filter {
public:
    ThresholdFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_threshold_filter(&error);
        type_           = "ThresholdFilter";
        Error::handle(&error, true);
    }
    ~ThresholdFilter() override = default;

    /**
     * @brief Get the threshold filter min range.
     *
     * @return OBIntPropertyRange The range of the threshold filter min.
     */
    OBIntPropertyRange getMinRange() {
        ob_error          *error = nullptr;
        OBIntPropertyRange range = ob_threshold_filter_get_min_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the threshold filter max range.
     *
     * @return OBIntPropertyRange The range of the threshold filter max.
     */
    OBIntPropertyRange getMaxRange() {
        ob_error          *error = nullptr;
        OBIntPropertyRange range = ob_threshold_filter_get_max_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the threshold filter max and min range.
     */
    bool setValueRange(uint16_t min, uint16_t max) {
        ob_error *error  = nullptr;
        bool      result = ob_threshold_filter_set_scale_value(impl_, min, max, &error);
        Error::handle(&error, true);
        return result;
    }
};

/**
 * @brief Create SequenceIdFilter processing block.
 */
class SequenceIdFilter : public Filter {
public:
    SequenceIdFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_sequenceId_filter(&error);
        type_           = "SequenceIdFilter";
        Error::handle(&error, true);
    }

    /**
     * @brief Set the sequenceId filter params.
     *
     * @param sequence id to pass the filter.
     */
    void selectSequenceId(int sequence_id) {
        ob_error *error = nullptr;
        ob_sequence_id_filter_select_sequence_id(impl_, sequence_id, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the current sequence id.
     *
     * @return sequence id to pass the filter.
     */
    int getSelectSequenceId() {
        ob_error *error       = nullptr;
        int       sequence_id = ob_sequence_id_filter_get_sequence_id(impl_, &error);
        Error::handle(&error, true);
        return sequence_id;
    }

    /**
     * @brief Get the current sequence id list.
     *
     * @return OBSequenceIdItem.
     */
    OBSequenceIdItem *getSequenceIdList() {
        ob_error         *error            = nullptr;
        OBSequenceIdItem *sequence_id_list = ob_sequence_id_filter_get_sequence_id_list(impl_, &error);
        Error::handle(&error, true);
        return sequence_id_list;
    }

    /**
     * @brief Get the sequenceId list size.
     *
     * @return the size of sequenceId list.
     */
    int getSequenceIdListSize() {
        ob_error *error = nullptr;
        int       size  = ob_sequence_id_filter_get_sequence_id_list_size(impl_, &error);
        Error::handle(&error, true);
        return size;
    }
};

/**
 * @brief The noise removal filter,removing scattering depth pixels.
 */
class NoiseRemovalFilter : public Filter {
public:
    NoiseRemovalFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_noise_removal_filter(&error);
        type_           = "NoiseRemovalFilter";
        Error::handle(&error, true);
    }

    ~NoiseRemovalFilter() override = default;
    /**
     * @brief Set the noise removal filter params.
     *
     * @param[in] params ob_noise_removal_filter_params.
     */
    void setFilterParams(OBNoiseRemovalFilterParams filterParams) {
        ob_error *error = nullptr;
        ob_noise_removal_filter_set_filter_params(impl_, filterParams, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the noise removal filter params.
     *
     * @return OBNoiseRemovalFilterParams.
     */
    OBNoiseRemovalFilterParams getFilterParams() {
        ob_error                  *error        = nullptr;
        OBNoiseRemovalFilterParams filterParams = ob_noise_removal_filter_get_filter_params(impl_, &error);
        Error::handle(&error, true);
        return filterParams;
    }

    /**
     * @brief Get the noise removal filter disp diff range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getDispDiffRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_noise_removal_filter_get_disp_diff_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the noise removal filter max size range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBIntPropertyRange getMaxSizeRange() {
        ob_error          *error = nullptr;
        OBIntPropertyRange range = ob_noise_removal_filter_get_max_size_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }
};

/**
 * @brief Decimation filter,reducing complexity by subsampling depth maps and losing depth details.
 */
class DecimationFilter : public Filter {
public:
    DecimationFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_decimation_filter(&error);
        type_           = "DecimationFilter";
        Error::handle(&error, true);
    }

    ~DecimationFilter() override = default;
    /**
     * @brief Set the decimation filter scale value.
     *
     * @param type The decimation filter scale value.
     */
    void setScaleValue(uint8_t value) {
        ob_error *error = nullptr;
        ob_decimation_filter_set_scale_value(impl_, value, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the decimation filter scale value.
     */
    uint8_t getScaleValue() {
        ob_error *error = nullptr;
        uint8_t   value = ob_decimation_filter_get_scale_value(impl_, &error);
        Error::handle(&error, true);
        return value;
    }

    /**
     * @brief Get the property range of the decimation filter scale value.
     */
    OBUint8PropertyRange getScaleRange() {
        ob_error            *error = nullptr;
        OBUint8PropertyRange range = ob_decimation_filter_get_scale_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }
};

/**
 * @brief The edge noise removal filter,removing scattering depth pixels.
 */
class EdgeNoiseRemovalFilter : public Filter {
public:
    EdgeNoiseRemovalFilter() : Filter() {
        ob_error *error = nullptr;
        impl_           = ob_create_edge_noise_removal_filter(&error);
        type_           = "EdgeNoiseRemovalFilter";
        Error::handle(&error, true);
    }
    ~EdgeNoiseRemovalFilter() override = default;

    /**
     * @brief Set the edge noise removal filter params.
     *
     * @param[in] params ob_edge_noise_removal_filter_params.
     */
    void setFilterParams(OBEdgeNoiseRemovalFilterParams filterParams) {
        ob_error *error = nullptr;
        ob_edge_noise_removal_filter_set_filter_params(impl_, filterParams, &error);
        Error::handle(&error, true);
    }

    /**
     * @brief Get the edge noise removal filter params.
     *
     * @return OBEdgeNoiseRemovalFilterParams.
     */
    OBEdgeNoiseRemovalFilterParams getFilterParams() {
        ob_error                      *error        = nullptr;
        OBEdgeNoiseRemovalFilterParams filterParams = ob_edge_noise_removal_filter_get_filter_params(impl_, &error);
        Error::handle(&error, true);
        return filterParams;
    }

    /**
     * @brief Get the edge noise removal filter margin left th range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getMarginLeftThRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_edge_noise_removal_filter_get_margin_left_th_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the edge noise removal filter margin right th range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMarginRightThRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_edge_noise_removal_filter_get_margin_right_th_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the edge noise removal filter margin top th range.
     * @return OBUint16PropertyRange The disp diff of property range.
     */
    OBUint16PropertyRange getMarginTopThRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_edge_noise_removal_filter_get_margin_top_th_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }

    /**
     * @brief Get the edge noise removal filter margin bottom th range.
     * @return OBUint16PropertyRange The max size of property range.
     */
    OBUint16PropertyRange getMarginBottomThRange() {
        ob_error             *error = nullptr;
        OBUint16PropertyRange range = ob_edge_noise_removal_filter_get_margin_bottom_th_range(impl_, &error);
        Error::handle(&error, true);
        return range;
    }
};

// Define the is() template function for the Filter class
template <typename T> bool Filter::is() {
    std::string name = type();
    if(name == "HDRMerge") {
        return typeid(T) == typeid(HdrMerge);
    }
    if(name == "SequenceIdFilter") {
        return typeid(T) == typeid(SequenceIdFilter);
    }
    if(name == "ThresholdFilter") {
        return typeid(T) == typeid(ThresholdFilter);
    }
    if(name == "DisparityTransform") {
        return typeid(T) == typeid(DisparityTransform);
    }
    if(name == "NoiseRemovalFilter") {
        return typeid(T) == typeid(NoiseRemovalFilter);
    }
    if(name == "SpatialAdvancedFilter") {
        return typeid(T) == typeid(SpatialAdvancedFilter);
    }
    if(name == "TemporalFilter") {
        return typeid(T) == typeid(TemporalFilter);
    }
    if(name == "HoleFillingFilter") {
        return typeid(T) == typeid(HoleFillingFilter);
    }
    if(name == "DecimationFilter") {
        return typeid(T) == typeid(DecimationFilter);
    }
    if(name == "PointCloudFilter") {
        return typeid(T) == typeid(PointCloudFilter);
    }
    if(name == "CompressionFilter") {
        return typeid(T) == typeid(CompressionFilter);
    }
    if(name == "DecompressionFilter") {
        return typeid(T) == typeid(DecompressionFilter);
    }
    if(name == "FormatConverter") {
        return typeid(T) == typeid(FormatConvertFilter);
    }
    if(name == "Align") {
        return typeid(T) == typeid(Align);
    }
    if(name == "EdgeNoiseRemovalFilter") {
        return typeid(T) == typeid(EdgeNoiseRemovalFilter);
    }
    return false;
}
/**
 * @brief Class representing a list of FrameProcessingBlock
 */
class OBFilterList {
private:
    ob_filter_list_t *impl_ = nullptr;

public:
    explicit OBFilterList(ob_filter_list_t *impl) : impl_(impl) {}
    ~OBFilterList() noexcept {
        ob_error *error = nullptr;
        ob_delete_filter_list(impl_, &error);
        Error::handle(&error, false);
    }

    /**
     * @brief Get the number of OBDepthWorkMode FrameProcessingBlock in the list
     *
     * @return uint32_t the number of FrameProcessingBlock objects in the list
     */
    uint32_t count() {
        ob_error *error = nullptr;
        auto      count = ob_filter_list_get_count(impl_, &error);
        Error::handle(&error);
        return count;
    }

    /**
     * @brief Get the Filter object at the specified index
     *
     * @param index the index of the target Filter object
     * @return the Filter object at the specified index
     */
    std::shared_ptr<Filter> getFilter(uint32_t index) {
        ob_error *error  = nullptr;
        auto      filter = ob_filter_list_get_filter(impl_, index, &error);
        Error::handle(&error);
        error            = nullptr;
        const char *name = ob_get_filter_name(filter, &error);
        Error::handle(&error);
        return std::make_shared<Filter>(filter, name);
    }
};

}  // namespace ob
