#ifndef D2C_DEPTH_TO_COLOR_IMPL_H
#define D2C_DEPTH_TO_COLOR_IMPL_H

#include <string>
#include <utility>
#include <unordered_map>
#include <memory>
#include "openobsdk/h/ObTypes.h"

namespace libobsensor {

#define EPSILON (1e-6)

struct ResHashFunc {
    size_t operator()(const std::pair<int, int> &p) const {
        return (((uint32_t)p.first & 0xFFFF) << 16) | ((uint32_t)p.second & 0xFFFF);
    }
};

struct ResComp {
    bool operator()(const std::pair<int, int> &p1, const std::pair<int, int> &p2) const {
        return p1.first == p2.first && p1.second == p2.second;
    }
};

class AlignImpl {
public:
    AlignImpl();

    ~AlignImpl();

    /**
     * @brief Load parameters and set up LUT
     *
     */
    void initialize(OBCameraIntrinsic depth_intrin, OBCameraDistortion depth_disto, OBCameraIntrinsic rgb_intrin, OBCameraDistortion rgb_disto,
                    OBExtrinsic from_to_extrin, float depth_unit_mm, bool add_target_distortion, bool gap_fill_copy);

    /**
     * @brief Get Depth Unit in millimeter
     *
     * @return float
     */
    float getDepthUnit() {
        return depth_unit_mm_;
    };

    /**
     * @brief Prepare LUTs of depth undistortion and rotation
     */
    void prepareDepthResolution();

    /**
     * @brief Clear buffer and de-initialize
     */
    void reset();

    /**
     * @brief       Align depth to color
     * @param       depth_buffer  data buffer of depth frame in row-major order
     * @param       depth_width   width of depth frame
     * @param       depth_height  height of depth frame
     * @param[out]  out_depth     aligned data buffer in row-major order
     * @param       color_width   width of color frame
     * @param       color_height  height of color frame
     * @retval  -1  failed
     * @retval  0   success
     */
    int D2C(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height, int *depth_xy = nullptr);

    int D2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    int C2D(const uint16_t *depth_buffer, int depth_width, int depth_height, const void *rgb_buffer, void *out_rgb, int color_width, int color_height, OBFormat format);

    /// TODO(timon): make private
    template <typename T> void mapBytes(const int *map, const T *src_buffer, int src_width, int src_height, T *dst_buffer, int dst_width);

    /**
     *@brief         Initialize LUTs for color image undistortion
     *@return        
     */
    bool initRGBUndistortion();

    /**
     * @brief        Undistort a RGB image 
     * @param[in]    src     original rgb image in row-major order, like (r00,g00,b00,r01,g01,b01,...,r10,g10,b10,...)
     * @param[in]    width   columns of the input image
     * @param[in]    height  rows of the input image
     * @param[out]   dst     undistorted output
     * @return     result of operation 
     */
    bool undistortRGB(const uint8_t *src, int width, int height, uint8_t *dst);

protected:
    bool prepareRGBDistort();

    void clearMatrixCache();

    virtual int distortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    virtual int KBDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);
    virtual int BMDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    int linearD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

private:
    bool initialized_;

    /** Linear/Distortion Rotation Coeff hash table */
    std::unordered_map<std::pair<int, int>, float *, ResHashFunc, ResComp> rot_coeff_ht_x;
    std::unordered_map<std::pair<int, int>, float *, ResHashFunc, ResComp> rot_coeff_ht_y;
    std::unordered_map<std::pair<int, int>, float *, ResHashFunc, ResComp> rot_coeff_ht_z;

    int x_start_, y_start_, x_end_, y_end_;

    float              depth_unit_mm_;
    bool               add_target_distortion_;
    bool               gap_fill_copy_;
    OBCameraIntrinsic  depth_intric_;
    OBCameraIntrinsic  rgb_intric_;
    OBCameraDistortion depth_disto_;
    OBCameraDistortion rgb_disto_;
    OBExtrinsic        transform_;  // should be depth-to-rgb all the time
    float              scaled_trans_[3];

    // possible inflection point of the calibrated K6 distortion curve
    float r2_max_loc_;

    // LUT for RGB distortion
    int    rgb_lut_width_  = 0;
    int    rgb_lut_height_ = 0;
    float *rgb_dx_lut_     = nullptr;
    float *rgb_dy_lut_     = nullptr;
};

#endif  // D2C_DEPTH_TO_COLOR_IMPL_H
}  // namespace
