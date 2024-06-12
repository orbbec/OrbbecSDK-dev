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
    bool prepareDepthResolution();

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
    int D2C(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    int D2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    int C2D(const uint16_t *depth_buffer, int depth_width, int depth_height, const void *rgb_buffer, void *out_rgb, int color_width, int color_height, OBFormat format);

    /// TODO(timon): make private
    template <typename T>
    int C2DBytes(const uint16_t *depth_buffer, int depth_width, int depth_height, const T *rgb_buffer, T *out_rgb, int color_width, int color_height);
    /**
     * @brief	彩色图坐标点转换到深度相机坐标系（单点）
     * @param[in]   depth_buffer  输入深度图buffer，行优先存储
     * @param[in]   depth_width   输入深度图宽度
     * @param[in]   depth_height  输入深度图高度
     * @param[in]   color_x		 输入彩色图坐标点x列坐标
     * @param[in]   color_y		 输入彩色图坐标点y行坐标
     * @param[in]   roi_w		 在深度图中的搜索起始位置x
     * @param[in]   roi_h		 在深度图中的搜索起始位置y
     * @param[in]   roi_w		 在深度图中的搜索范围width
     * @param[in]   roi_h		 在深度图中的搜索范围height
     * @param[out]  depth_x		 输出深度图对应坐标点x列坐标
     * @param[out]  depth_y		 输出深度图对应坐标点y行坐标
     * @param[in]   color_width   对齐后的深度图宽度
     * @param[in]   color_height  对齐后的深度图高度
     * @retval -1   failed
     * @retval  0   success
     */
    //int C2D(const uint16_t *depth_buffer, int depth_width, int depth_height, int color_x, int color_y, int roi_x, int roi_y, int roi_w, int roi_h, int &depth_x,
            //int &depth_y, int color_width, int color_height);

    /**
     * @brief	彩色图坐标点转换到深度相机坐标系（多点）
     * @param[in]   depth_buffer   输入深度图buffer，行优先存储
     * @param[in]   depth_width    输入深度图宽度
     * @param[in]   depth_height   输入深度图高度
     * @param[in]   color_xy		  输入彩色图坐标点xy坐标，在数组中使用{x1, y1, x2, y2...}的方式存储
     * @param[in]   color_xy_count 输入彩色图坐标点的个数，需要和输入输出数组中存储的值个数一致
     * @param[out]  depth_xy		  输入深度图对应坐标点xy坐标，在数组中使用{x1, y1, x2, y2...}的方式存储
     * @param[in]   color_width    对齐后的深度图宽度
     * @param[in]   color_height   对齐后的深度图高度
     * @retval -1   failed
     * @retval  0   success
     */
    //int C2D(const uint16_t *depth_buffer, int depth_width, int depth_height, int *color_xy, int color_xy_count, int *depth_xy, int color_width,
    //        int color_height);
    //int C2DWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, int *color_xy, int color_xy_count, int *depth_xy, int color_width,
    //               int color_height);

    //int C2DPts(const uint16_t *d2c_depth, int *color_xy, int color_xy_count, int *depth_xy, int color_width, int color_height);

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
    bool prepareDepthResDistort(int width, int height);
    bool prepareDepthResLinear(int width, int height);

    bool prepareRGBDistort();

    void clearMatrixCache();

    int distortedD2C(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    virtual int distortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    virtual int KBDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);
    virtual int BMDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

    int linearD2C(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height);

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
    OBExtrinsic        transform_;  // depth-to-rgb or rgb-to-depth according to initialization
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
