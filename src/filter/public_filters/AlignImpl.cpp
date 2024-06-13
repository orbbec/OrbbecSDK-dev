#include "AlignImpl.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"
#include <omp.h>
#include <fstream>
#include <iostream>
#include <chrono>

#ifdef _WIN32
#include <xmmintrin.h>
#include <smmintrin.h>
#else
#include "SSE2NEON.h"
#endif

namespace libobsensor {

// channel: 0-r, 1-g, 2-b
uint8_t BilinearInterpolationRGB(const uint8_t *img, int W, int H, float x, float y, uint8_t def_val, int channel) {
    uint8_t ret = def_val;

    if(x < 0 || y < 0 || x >= W || y >= H) {
        return ret;
    }

    int ix = (int)x;
    int iy = (int)y;

    float dx = x - ix;
    float dy = y - iy;

    int a0 = img[3 * ix + channel + iy * 3 * W];
    int a1 = img[3 * ix + channel + 1 * 3 + iy * 3 * W] - a0;
    int a2 = img[3 * ix + channel + (iy + 1) * 3 * W] - a0;
    int a3 = img[3 * ix + channel + 1 * 3 + (iy + 1) * 3 * W] - a0 - a1 - a2;

    ret = (uint8_t)(a0 + a1 * dx + a2 * dy + a3 * dx * dy);

    return ret;
}

static inline void addDistortion(const OBCameraDistortion &distort_param, const OBCameraDistortionModel model, const float pt_ud[2], float pt_d[2]) {
    float k1 = distort_param.k1, k2 = distort_param.k2, k3 = distort_param.k3;
    float k4 = distort_param.k4, k5 = distort_param.k5, k6 = distort_param.k5;
    float p1 = distort_param.p1, p2 = distort_param.p2;

    const float r2 = pt_ud[0] * pt_ud[0] + pt_ud[1] * pt_ud[1];

    if((model == OB_DISTORTION_BROWN_CONRADY) || (model == OB_DISTORTION_BROWN_CONRADY_K6)) {
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;

        float k_diff = 1 + k1 * r2 + k2 * r4 + k3 * r6;
        if(model == OB_DISTORTION_BROWN_CONRADY_K6) {
            k_diff /= (1 + k4 * r2 + k5 * r4 + k6 * r6);
        }

        const float t_x = p2 * (r2 + 2 * pt_ud[0] * pt_ud[0]) + 2 * p1 * pt_ud[0] * pt_ud[1];
        const float t_y = p1 * (r2 + 2 * pt_ud[1] * pt_ud[1]) + 2 * p2 * pt_ud[0] * pt_ud[1];

        pt_d[0] = pt_ud[0] * k_diff + t_x;
        pt_d[1] = pt_ud[1] * k_diff + t_y;
    }
    else if(model == OB_DISTORTION_KANNALA_BRANDT4) {
        const double r      = sqrt(r2);
        const double theta  = atan(r);
        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta2 * theta4;
        const double theta8 = theta4 * theta4;

        const double k_diff = theta * (1 + (k1 * theta2) + (k2 * theta4) + (k3 * theta6) + (k4 * theta8));

        pt_d[0] = static_cast<float>(k_diff / r * pt_ud[0]);
        pt_d[1] = static_cast<float>(k_diff / r * pt_ud[1]);
    }
}

static inline void removeDistortion(const OBCameraDistortion &distort_param, const OBCameraDistortionModel model, const float pt_d[2], float pt_ud[2]) {
    const float epsilon       = 1e-6f;
    const int   max_iteration = 20;
    float       tmp_p_ud[2]   = { pt_d[0], pt_d[1] };
    addDistortion(distort_param, model, tmp_p_ud, pt_ud);

    pt_ud[0]  = pt_ud[0] - tmp_p_ud[0];
    pt_ud[1]  = pt_ud[1] - tmp_p_ud[1];
    int   it  = 0;
    float err = fabs(tmp_p_ud[0] + pt_ud[0] - pt_d[0]) + fabs(tmp_p_ud[1] + pt_ud[1] - pt_d[1]);
    while(err > epsilon && it++ < max_iteration) {
        tmp_p_ud[0] = pt_d[0] - pt_ud[0];
        tmp_p_ud[1] = pt_d[1] - pt_ud[1];
        addDistortion(distort_param, model, tmp_p_ud, pt_ud);

        pt_ud[0] = pt_ud[0] - tmp_p_ud[0];
        pt_ud[1] = pt_ud[1] - tmp_p_ud[1];
        err      = fabs(tmp_p_ud[0] + pt_ud[0] - pt_d[0]) + fabs(tmp_p_ud[1] + pt_ud[1] - pt_d[1]);
    }

    pt_ud[0] = tmp_p_ud[0];
    pt_ud[1] = tmp_p_ud[1];
}

static inline void convertProjectiveToWorldDisto(const OBCameraIntrinsic &intrinsic, int u, int v, int z, float &px, float &py, float &pz,
                                                 const OBCameraDistortion &distort_param) {
    float tx = (u - intrinsic.cx) / intrinsic.fx;
    float ty = (v - intrinsic.cy) / intrinsic.fy;

    float tx_d = tx;
    float ty_d = ty;
    float pUndisto[2];
    float pdisto[2] = { tx_d, ty_d };
    removeDistortion(distort_param, intrinsic.model, pdisto, pUndisto);

    tx = pUndisto[0];
    ty = pUndisto[1];

    px = z * tx;
    py = z * ty;
    pz = static_cast<float>(z);
}

static inline void convertProjectiveToWorldLinear(const OBCameraIntrinsic &intrinsic, int u, int v, int z, float &px, float &py, float &pz) {
    float tx = (u - intrinsic.cx) / intrinsic.fx;
    float ty = (v - intrinsic.cy) / intrinsic.fy;

    px = z * tx;
    py = z * ty;
    pz = static_cast<float>(z);
}

AlignImpl::AlignImpl() : initialized_(false) {
#ifdef _WIN32
    omp_set_dynamic(0);
    int max_thread = omp_get_max_threads();
    omp_set_num_threads(max_thread);
#endif
    x_start_       = -1;
    y_start_       = -1;
    x_end_         = -1;
    y_end_         = -1;
    depth_unit_mm_ = 1.0;
    r2_max_loc_    = 0.0;
    memset(&depth_intric_, 0, sizeof(OBCameraIntrinsic));
    memset(&depth_disto_, 0, sizeof(OBCameraDistortion));
    memset(&rgb_intric_, 0, sizeof(OBCameraIntrinsic));
    memset(&rgb_disto_, 0, sizeof(OBCameraDistortion));
}

AlignImpl::~AlignImpl() {
    clearMatrixCache();
    initialized_ = false;
}

void AlignImpl::initialize(OBCameraIntrinsic depth_intrin, OBCameraDistortion depth_disto, OBCameraIntrinsic rgb_intrin, OBCameraDistortion rgb_disto,
                           OBExtrinsic extrin, float depth_unit_mm, bool add_target_distortion, bool gap_fill_copy) {
    if(initialized_) {
        return;
    }
    memcpy(&depth_intric_, &depth_intrin, sizeof(OBCameraIntrinsic));
    memcpy(&depth_disto_, &depth_disto, sizeof(OBCameraDistortion));
    memcpy(&rgb_intric_, &rgb_intrin, sizeof(OBCameraIntrinsic));
    memcpy(&rgb_disto_, &rgb_disto, sizeof(OBCameraDistortion));
    memcpy(&transform_, &extrin, sizeof(OBExtrinsic));

    add_target_distortion = add_target_distortion;
    depth_unit_mm_        = depth_unit_mm;
    gap_fill_copy_        = gap_fill_copy;
    // Translation is related to depth unit.
    for(int i = 0; i < 3; ++i)
        scaled_trans_[i] = transform_.trans[i] / depth_unit_mm_;

    /// TODO(timon): Set ROI to full temporarily
    x_start_ = 0;
    y_start_ = 0;
    x_end_   = rgb_intric_.width;
    y_end_   = rgb_intric_.height;

    prepareDepthResolution();
    initialized_ = true;
}

void AlignImpl::reset() {
    clearMatrixCache();
    initialized_ = false;
}

void AlignImpl::prepareDepthResolution() {
    clearMatrixCache();

    // There may be outliers due to possible inflection points of the calibrated K6 distortion curve;
    if((rgb_intric_.model == OB_DISTORTION_BROWN_CONRADY_K6) && (add_target_distortion_)) {
        float w  = rgb_intric_.width;
        float h  = rgb_intric_.height;
        float fx = rgb_intric_.fx;
        float fy = rgb_intric_.fy;
        float cx = rgb_intric_.cx;
        float cy = rgb_intric_.cy;
        float k1 = rgb_disto_.k1, k2 = rgb_disto_.k2, k3 = rgb_disto_.k3;
        float k4 = rgb_disto_.k4, k5 = rgb_disto_.k5, k6 = rgb_disto_.k6;

        float pt_ud_max[2] /*, pt_d_max[2]*/;
        pt_ud_max[0] = (w / 2 + abs(w / 2 - cx)) / fx;
        pt_ud_max[1] = (h / 2 + abs(h / 2 - cy)) / fy;
        float r2     = powf(pt_ud_max[0], 2) + powf(pt_ud_max[1], 2);

        float half_r2 = 0.5f * r2;
        float f2      = half_r2;
        float f4 = f2 * f2, f6 = f4 * f2;
        float half_r2_distort  = (1 + k1 * f2 + k2 * f4 + k3 * f6) / (1 + k4 * f2 + k5 * f4 + k6 * f6);
        bool  polarity_half_r2 = true;
        if(half_r2_distort < 1)
            polarity_half_r2 = false;

        float kr_diff_cur = 0;
        float delta       = 0.001f * r2;
        while(f2 < r2) {
            f4                = f2 * f2;
            f6                = f4 * f2;
            kr_diff_cur       = (1 + k1 * f2 + k2 * f4 + k3 * f6) / (1 + k4 * f2 + k5 * f4 + k6 * f6);
            bool polarity_cur = kr_diff_cur > 1 ? true : false;
            if(polarity_cur != polarity_half_r2) {
                r2_max_loc_ = f2 - 10 * delta;
                break;
            }
            f2 += delta;
        }
        if(f2 >= r2)
            r2_max_loc_ = r2;
    }

    /// TODO(timon): linear and distort for depth should be same since depth has no distortion
    /// prepareRGBRes
    {
        float *rot_coeff1 = new float[depth_intric_.width * depth_intric_.height];
        float *rot_coeff2 = new float[depth_intric_.width * depth_intric_.height];
        float *rot_coeff3 = new float[depth_intric_.width * depth_intric_.height];

        for(int v = 0; v < depth_intric_.height; ++v) {
            float *dst1 = rot_coeff1 + v * depth_intric_.width;
            float *dst2 = rot_coeff2 + v * depth_intric_.width;
            float *dst3 = rot_coeff3 + v * depth_intric_.width;
            float  y    = (v - depth_intric_.cy) / depth_intric_.fy;
            for(int u = 0; u < depth_intric_.width; ++u) {
                float x = (u - depth_intric_.cx) / depth_intric_.fx;

                if(add_target_distortion_) {
                    float pt_d[2] = { x, y };
                    float pt_ud[2];
                    removeDistortion(depth_disto_, depth_intric_.model, pt_d, pt_ud);
                    x = pt_ud[0];
                    y = pt_ud[1];
                }

                *dst1++ = transform_.rot[0] * x + transform_.rot[1] * y + transform_.rot[2];
                *dst2++ = transform_.rot[3] * x + transform_.rot[4] * y + transform_.rot[5];
                *dst3++ = transform_.rot[6] * x + transform_.rot[7] * y + transform_.rot[8];
            }
        }

        rot_coeff_ht_x[std::make_pair(depth_intric_.width, depth_intric_.height)] = rot_coeff1;
        rot_coeff_ht_y[std::make_pair(depth_intric_.width, depth_intric_.height)] = rot_coeff2;
        rot_coeff_ht_z[std::make_pair(depth_intric_.width, depth_intric_.height)] = rot_coeff3;
    }
}

void AlignImpl::clearMatrixCache() {
    for(auto item: rot_coeff_ht_x) {
        if(item.second) {
            delete[] item.second;
        }
    }
    for(auto item: rot_coeff_ht_y) {
        if(item.second) {
            delete[] item.second;
        }
    }
    for(auto item: rot_coeff_ht_z) {
        if(item.second) {
            delete[] item.second;
        }
    }

    rot_coeff_ht_x.clear();
    rot_coeff_ht_y.clear();
    rot_coeff_ht_z.clear();

    if(rgb_dx_lut_) {
        free(rgb_dx_lut_);
        rgb_dx_lut_ = nullptr;
    }
    if(rgb_dy_lut_) {
        free(rgb_dy_lut_);
        rgb_dy_lut_ = nullptr;
    }
}

int AlignImpl::BMDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height) {
    if(!initialized_) {
        LOG_ERROR("Make sure to initializa firstly");
        return -1;
    }

    auto finder_x = rot_coeff_ht_x.find(std::make_pair(depth_width, depth_height));
    auto finder_y = rot_coeff_ht_y.find(std::make_pair(depth_width, depth_height));
    auto finder_z = rot_coeff_ht_z.find(std::make_pair(depth_width, depth_height));
    if(rot_coeff_ht_x.cend() == finder_x) {
        LOG_ERROR("Found a new resolution, but initialize failed!");
        return -1;
    }

    if((!depth_buffer) || (!out_depth)) {
        LOG_ERROR("depth_buffer or out_depth is NULL!");
        return -1;
    }

    if(true)
        memset(out_depth, 0xff, color_height * color_width * sizeof(uint16_t));
    else
        memset(out_depth, 0x0, color_height * color_width * sizeof(uint16_t));

    const float *coeff_mat_x = finder_x->second;
    const float *coeff_mat_y = finder_y->second;
    const float *coeff_mat_z = finder_z->second;

    __m128 x1_limit = _mm_set_ps1(float(x_start_));
    __m128 x2_limit = _mm_set_ps1(float(x_end_));
    __m128 y1_limit = _mm_set_ps1(float(y_start_));
    __m128 y2_limit = _mm_set_ps1(float(y_end_));

    __m128 k1 = _mm_set_ps1(rgb_disto_.k1);
    __m128 k2 = _mm_set_ps1(rgb_disto_.k2);
    __m128 k3 = _mm_set_ps1(rgb_disto_.k3);
    __m128 k4 = _mm_set_ps1(rgb_disto_.k4);
    __m128 k5 = _mm_set_ps1(rgb_disto_.k5);
    __m128 k6 = _mm_set_ps1(rgb_disto_.k6);
    __m128 p1 = _mm_set_ps1(rgb_disto_.p1);
    __m128 p2 = _mm_set_ps1(rgb_disto_.p2);

    __m128 scaled_trans_1 = _mm_set_ps1(scaled_trans_[0]);
    __m128 scaled_trans_2 = _mm_set_ps1(scaled_trans_[1]);
    __m128 scaled_trans_3 = _mm_set_ps1(scaled_trans_[2]);

    __m128 color_K_0_0 = _mm_set_ps1(rgb_intric_.fx);
    __m128 color_K_0_2 = _mm_set_ps1(rgb_intric_.cx);
    __m128 color_K_1_1 = _mm_set_ps1(rgb_intric_.fy);
    __m128 color_K_1_2 = _mm_set_ps1(rgb_intric_.cy);

    __m128  point_five = _mm_set_ps1(0.5);
    __m128  two        = _mm_set_ps1(2);
    __m128i zero       = _mm_setzero_si128();
    __m128  zero_f     = _mm_set_ps1(0.0);
    __m128  r2_max_loc = _mm_set_ps1(r2_max_loc_);

    int imgSize = depth_width * depth_height;

#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
    for(int i = 0; i < imgSize; i += 8) {
        __m128i depth_i16    = _mm_loadu_si128((__m128i *)(depth_buffer + i));
        __m128i depth_lo_i   = _mm_unpacklo_epi16(depth_i16, zero);
        __m128i depth_hi_i   = _mm_unpackhi_epi16(depth_i16, zero);
        __m128  depth_sse_lo = _mm_cvtepi32_ps(depth_lo_i);
        __m128  depth_sse_hi = _mm_cvtepi32_ps(depth_hi_i);

        __m128 coeff_sse1_lo = _mm_loadu_ps(coeff_mat_x + i);
        __m128 coeff_sse1_hi = _mm_loadu_ps(coeff_mat_x + i + 4);
        __m128 coeff_sse2_lo = _mm_loadu_ps(coeff_mat_y + i);
        __m128 coeff_sse2_hi = _mm_loadu_ps(coeff_mat_y + i + 4);
        __m128 coeff_sse3_lo = _mm_loadu_ps(coeff_mat_z + i);
        __m128 coeff_sse3_hi = _mm_loadu_ps(coeff_mat_z + i + 4);

        __m128 X_lo = _mm_add_ps(_mm_mul_ps(depth_sse_lo, coeff_sse1_lo), scaled_trans_1);
        __m128 X_hi = _mm_add_ps(_mm_mul_ps(depth_sse_hi, coeff_sse1_hi), scaled_trans_1);
        __m128 Y_lo = _mm_add_ps(_mm_mul_ps(depth_sse_lo, coeff_sse2_lo), scaled_trans_2);
        __m128 Y_hi = _mm_add_ps(_mm_mul_ps(depth_sse_hi, coeff_sse2_hi), scaled_trans_2);
        __m128 Z_lo = _mm_add_ps(_mm_mul_ps(depth_sse_lo, coeff_sse3_lo), scaled_trans_3);
        __m128 Z_hi = _mm_add_ps(_mm_mul_ps(depth_sse_hi, coeff_sse3_hi), scaled_trans_3);

        X_lo = _mm_div_ps(X_lo, Z_lo);
        Y_lo = _mm_div_ps(Y_lo, Z_lo);
        X_hi = _mm_div_ps(X_hi, Z_hi);
        Y_hi = _mm_div_ps(Y_hi, Z_hi);

        __m128 x2_lo = _mm_mul_ps(X_lo, X_lo);
        __m128 y2_lo = _mm_mul_ps(Y_lo, Y_lo);
        __m128 xy_lo = _mm_mul_ps(X_lo, Y_lo);
        __m128 r2_lo = _mm_add_ps(x2_lo, y2_lo);
        __m128 r4_lo = _mm_mul_ps(r2_lo, r2_lo);
        __m128 r6_lo = _mm_mul_ps(r4_lo, r2_lo);
        __m128 x2_hi = _mm_mul_ps(X_hi, X_hi);
        __m128 y2_hi = _mm_mul_ps(Y_hi, Y_hi);
        __m128 xy_hi = _mm_mul_ps(X_hi, Y_hi);
        __m128 r2_hi = _mm_add_ps(x2_hi, y2_hi);
        __m128 r4_hi = _mm_mul_ps(r2_hi, r2_hi);
        __m128 r6_hi = _mm_mul_ps(r4_hi, r2_hi);

        // float k_jx = k_diff = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6);
        __m128 k_jx_lo = _mm_div_ps(_mm_add_ps(_mm_set_ps1(1), _mm_add_ps(_mm_add_ps(_mm_mul_ps(k1, r2_lo), _mm_mul_ps(k2, r4_lo)), _mm_mul_ps(k3, r6_lo))),
                                    _mm_add_ps(_mm_set_ps1(1), _mm_add_ps(_mm_add_ps(_mm_mul_ps(k4, r2_lo), _mm_mul_ps(k5, r4_lo)), _mm_mul_ps(k6, r6_lo))));
        __m128 k_jx_hi = _mm_div_ps(_mm_add_ps(_mm_set_ps1(1), _mm_add_ps(_mm_add_ps(_mm_mul_ps(k1, r2_hi), _mm_mul_ps(k2, r4_hi)), _mm_mul_ps(k3, r6_hi))),
                                    _mm_add_ps(_mm_set_ps1(1), _mm_add_ps(_mm_add_ps(_mm_mul_ps(k4, r2_hi), _mm_mul_ps(k5, r4_hi)), _mm_mul_ps(k6, r6_hi))));

        // float x_qx = p2 * (2 * x2 + r2) + 2 * p1 * xy;
        __m128 x_qx_lo = _mm_add_ps(_mm_mul_ps(p2, _mm_add_ps(_mm_mul_ps(x2_lo, two), r2_lo)), _mm_mul_ps(_mm_mul_ps(p1, xy_lo), two));
        __m128 x_qx_hi = _mm_add_ps(_mm_mul_ps(p2, _mm_add_ps(_mm_mul_ps(x2_hi, two), r2_hi)), _mm_mul_ps(_mm_mul_ps(p1, xy_hi), two));

        // float y_qx = p1 * (2 * y2 + r2) + 2 * p2 * xy;
        __m128 y_qx_lo = _mm_add_ps(_mm_mul_ps(p1, _mm_add_ps(_mm_mul_ps(y2_lo, two), r2_lo)), _mm_mul_ps(_mm_mul_ps(p2, xy_lo), two));
        __m128 y_qx_hi = _mm_add_ps(_mm_mul_ps(p1, _mm_add_ps(_mm_mul_ps(y2_hi, two), r2_hi)), _mm_mul_ps(_mm_mul_ps(p2, xy_hi), two));

        // float distx = tx * k_jx + x_qx;
        X_lo = _mm_add_ps(_mm_mul_ps(X_lo, k_jx_lo), x_qx_lo);
        X_hi = _mm_add_ps(_mm_mul_ps(X_hi, k_jx_hi), x_qx_hi);

        // float disty = ty * k_jx + y_qx;
        Y_lo = _mm_add_ps(_mm_mul_ps(Y_lo, k_jx_lo), y_qx_lo);
        Y_hi = _mm_add_ps(_mm_mul_ps(Y_hi, k_jx_hi), y_qx_hi);

        // Z_lo = _mm_and_ps(Z_lo, _mm_cmplt_ps(r2_lo, r2_max_loc));
        // Z_hi = _mm_and_ps(Z_hi, _mm_cmplt_ps(r2_hi, r2_max_loc));
        __m128 flag = _mm_or_ps(_mm_cmpge_ps(zero_f, r2_max_loc), _mm_cmplt_ps(r2_lo, r2_max_loc));
        Z_lo        = _mm_and_ps(Z_lo, flag);
        flag        = _mm_or_ps(_mm_cmpge_ps(zero_f, r2_max_loc), _mm_cmplt_ps(r2_hi, r2_max_loc));
        Z_hi        = _mm_and_ps(Z_hi, flag);

        // pixel[0] = tx * color_K_0_0_ + color_K_0_2_;
        // pixel[1] = ty * color_K_1_1_ + color_K_1_2_;
        __m128 pixelx_lo = _mm_add_ps(_mm_mul_ps(X_lo, color_K_0_0), color_K_0_2);
        __m128 pixely_lo = _mm_add_ps(_mm_mul_ps(Y_lo, color_K_1_1), color_K_1_2);

        __m128 pixelx_hi = _mm_add_ps(_mm_mul_ps(X_hi, color_K_0_0), color_K_0_2);
        __m128 pixely_hi = _mm_add_ps(_mm_mul_ps(Y_hi, color_K_1_1), color_K_1_2);

        // pixelx_lo = _mm_add_ps(pixelx_lo, point_five);
        // pixely_lo = _mm_add_ps(pixely_lo, point_five);
        // pixelx_hi = _mm_add_ps(pixelx_hi, point_five);
        // pixely_hi = _mm_add_ps(pixely_hi, point_five);

        __m128 cmp_flag_lo =
            _mm_and_ps(_mm_and_ps(_mm_and_ps(_mm_cmpge_ps(pixelx_lo, x1_limit), _mm_cmpge_ps(pixely_lo, y1_limit)), _mm_cmplt_ps(pixelx_lo, x2_limit)),
                       _mm_cmplt_ps(pixely_lo, y2_limit));
        __m128 cmp_flag_hi =
            _mm_and_ps(_mm_and_ps(_mm_and_ps(_mm_cmpge_ps(pixelx_hi, x1_limit), _mm_cmpge_ps(pixely_hi, y1_limit)), _mm_cmplt_ps(pixelx_hi, x2_limit)),
                       _mm_cmplt_ps(pixely_hi, y2_limit));

        float x_lo[4] = { 0 };
        float y_lo[4] = { 0 };
        float z_lo[4] = { 0 };
        float x_hi[4] = { 0 };
        float y_hi[4] = { 0 };
        float z_hi[4] = { 0 };

        _mm_storeu_ps(x_lo, _mm_and_ps(cmp_flag_lo, pixelx_lo));
        _mm_storeu_ps(y_lo, _mm_and_ps(cmp_flag_lo, pixely_lo));
        _mm_storeu_ps(z_lo, _mm_and_ps(cmp_flag_lo, Z_lo));
        _mm_storeu_ps(x_hi, _mm_and_ps(cmp_flag_hi, pixelx_hi));
        _mm_storeu_ps(y_hi, _mm_and_ps(cmp_flag_hi, pixely_hi));
        _mm_storeu_ps(z_hi, _mm_and_ps(cmp_flag_hi, Z_hi));

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_lo[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_lo[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_lo[j]);
                out_depth[pos]       = depth_value;
            }
        }

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_hi[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_hi[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_hi[j]);
                out_depth[pos]       = depth_value;
            }
        }
    }

    if(true) {
        int pixnum = color_width * color_height;
#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
        for(int idx = 0; idx < pixnum; idx++) {
            if(65535 == out_depth[idx]) {
                out_depth[idx] = 0;
            }
        }
    }

    return 0;
}

/// TOOD(timon): error handling
int AlignImpl::KBDistortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height) {
    if(!initialized_) {
        LOG_ERROR("Make sure LoadParameters() success before D2C!");
        return -1;
    }

    auto finder_x = rot_coeff_ht_x.find(std::make_pair(depth_width, depth_height));
    auto finder_y = rot_coeff_ht_y.find(std::make_pair(depth_width, depth_height));
    auto finder_z = rot_coeff_ht_z.find(std::make_pair(depth_width, depth_height));
    if(rot_coeff_ht_x.cend() == finder_x) {
        LOG_ERROR("Found a new resolution, but initialize failed!");
        return -1;
    }

    if(!depth_buffer) {
        LOG_ERROR("depth_buffer is NULL!");
        return -1;
    }

    if(!out_depth) {
        LOG_ERROR("out_depth is NULL!");
        return -1;
    }

    if(true)
        memset(out_depth, 0xff, color_height * color_width * sizeof(uint16_t));
    else
        memset(out_depth, 0x0, color_height * color_width * sizeof(uint16_t));

    const float *coeff_mat_x = finder_x->second;
    const float *coeff_mat_y = finder_y->second;
    const float *coeff_mat_z = finder_z->second;

    __m128 x1_limit = _mm_set_ps1(float(x_start_));
    __m128 x2_limit = _mm_set_ps1(float(x_end_));
    __m128 y1_limit = _mm_set_ps1(float(y_start_));
    __m128 y2_limit = _mm_set_ps1(float(y_end_));

    /// TODO(timon): check K6?
    __m128 k1 = _mm_set_ps1(rgb_disto_.k1);
    __m128 k2 = _mm_set_ps1(rgb_disto_.k2);
    __m128 k3 = _mm_set_ps1(rgb_disto_.k3);
    __m128 k4 = _mm_set_ps1(rgb_disto_.k4);

    __m128 scaled_trans_1 = _mm_set_ps1(scaled_trans_[0]);
    __m128 scaled_trans_2 = _mm_set_ps1(scaled_trans_[1]);
    __m128 scaled_trans_3 = _mm_set_ps1(scaled_trans_[2]);

    __m128 color_K_0_0 = _mm_set_ps1(rgb_intric_.fx);
    __m128 color_K_0_2 = _mm_set_ps1(rgb_intric_.cx);
    __m128 color_K_1_1 = _mm_set_ps1(rgb_intric_.fy);
    __m128 color_K_1_2 = _mm_set_ps1(rgb_intric_.cy);

    __m128  point_five = _mm_set_ps1(0.5);
    __m128  two        = _mm_set_ps1(2);
    __m128i zero       = _mm_setzero_si128();

    int imgSize = depth_width * depth_height;

#pragma omp parallel for
    for(int i = 0; i < imgSize; i += 8) {
        __m128i depth_i16    = _mm_loadu_si128((__m128i *)(depth_buffer + i));
        __m128i depth_lo_i   = _mm_unpacklo_epi16(depth_i16, zero);
        __m128i depth_hi_i   = _mm_unpackhi_epi16(depth_i16, zero);
        __m128  depth_sse_lo = _mm_cvtepi32_ps(depth_lo_i);
        __m128  depth_sse_hi = _mm_cvtepi32_ps(depth_hi_i);

        __m128 coeff_sse1_lo = _mm_loadu_ps(coeff_mat_x + i);
        __m128 coeff_sse1_hi = _mm_loadu_ps(coeff_mat_x + i + 4);
        __m128 coeff_sse2_lo = _mm_loadu_ps(coeff_mat_y + i);
        __m128 coeff_sse2_hi = _mm_loadu_ps(coeff_mat_y + i + 4);
        __m128 coeff_sse3_lo = _mm_loadu_ps(coeff_mat_z + i);
        __m128 coeff_sse3_hi = _mm_loadu_ps(coeff_mat_z + i + 4);

        __m128 X_lo = _mm_mul_ps(depth_sse_lo, coeff_sse1_lo);
        __m128 X_hi = _mm_mul_ps(depth_sse_hi, coeff_sse1_hi);
        __m128 Y_lo = _mm_mul_ps(depth_sse_lo, coeff_sse2_lo);
        __m128 Y_hi = _mm_mul_ps(depth_sse_hi, coeff_sse2_hi);
        __m128 Z_lo = _mm_mul_ps(depth_sse_lo, coeff_sse3_lo);
        __m128 Z_hi = _mm_mul_ps(depth_sse_hi, coeff_sse3_hi);
        X_lo        = _mm_add_ps(X_lo, scaled_trans_1);
        X_hi        = _mm_add_ps(X_hi, scaled_trans_1);
        Y_lo        = _mm_add_ps(Y_lo, scaled_trans_2);
        Y_hi        = _mm_add_ps(Y_hi, scaled_trans_2);
        Z_lo        = _mm_add_ps(Z_lo, scaled_trans_3);
        Z_hi        = _mm_add_ps(Z_hi, scaled_trans_3);

        __m128 tx_lo = _mm_div_ps(X_lo, Z_lo);
        __m128 ty_lo = _mm_div_ps(Y_lo, Z_lo);
        __m128 tx_hi = _mm_div_ps(X_hi, Z_hi);
        __m128 ty_hi = _mm_div_ps(Y_hi, Z_hi);

        // float r=sqrt(tx^2+ty^2)
        __m128 x2_lo = _mm_mul_ps(tx_lo, tx_lo);
        __m128 y2_lo = _mm_mul_ps(ty_lo, ty_lo);
        __m128 r2_lo = _mm_add_ps(x2_lo, y2_lo);
        __m128 x2_hi = _mm_mul_ps(tx_hi, tx_hi);
        __m128 y2_hi = _mm_mul_ps(ty_hi, ty_hi);
        __m128 r2_hi = _mm_add_ps(x2_hi, y2_hi);

        __m128 r_lo = _mm_sqrt_ps(r2_lo);
        __m128 r_hi = _mm_sqrt_ps(r2_hi);

        // float theta=atan(r)
#ifdef _WIN32
        //#if _MSC_VER >= 1920    //
        //        __m128 theta_lo = _mm_atan_ps(r_lo);
        //        __m128 theta_hi = _mm_atan_ps(r_hi);
        //#elif _MSC_VER == 1900  //
        float r_lo_[4] = { 0 }, r_hi_[4] = { 0 };
        float theta_lo_[4] = { 0 }, theta_hi_[4] = { 0 };
        _mm_storeu_ps(r_lo_, r_lo);
        _mm_storeu_ps(r_hi_, r_hi);

        theta_lo_[0] = atan(r_lo_[0]), theta_hi_[0] = atan(r_hi_[0]);
        theta_lo_[1] = atan(r_lo_[1]), theta_hi_[1] = atan(r_hi_[1]);
        theta_lo_[2] = atan(r_lo_[2]), theta_hi_[2] = atan(r_hi_[2]);
        theta_lo_[3] = atan(r_lo_[3]), theta_hi_[3] = atan(r_hi_[3]);

        __m128 theta_lo = _mm_loadu_ps(theta_lo_);
        __m128 theta_hi = _mm_loadu_ps(theta_hi_);
//#endif
#else
        float r_lo_[4] = { 0 }, r_hi_[4] = { 0 };
        float theta_lo_[4] = { 0 }, theta_hi_[4] = { 0 };
        _mm_storeu_ps(r_lo_, r_lo);
        _mm_storeu_ps(r_hi_, r_hi);

        theta_lo_[0] = atan(r_lo_[0]), theta_hi_[0] = atan(r_hi_[0]);
        theta_lo_[1] = atan(r_lo_[1]), theta_hi_[1] = atan(r_hi_[1]);
        theta_lo_[2] = atan(r_lo_[2]), theta_hi_[2] = atan(r_hi_[2]);
        theta_lo_[3] = atan(r_lo_[3]), theta_hi_[3] = atan(r_hi_[3]);

        __m128 theta_lo = _mm_loadu_ps(theta_lo_);
        __m128 theta_hi = _mm_loadu_ps(theta_hi_);
#endif

        __m128 theta2_lo = _mm_mul_ps(theta_lo, theta_lo);
        __m128 theta2_hi = _mm_mul_ps(theta_hi, theta_hi);

        __m128 theta4_lo = _mm_mul_ps(theta2_lo, theta2_lo);
        __m128 theta4_hi = _mm_mul_ps(theta2_hi, theta2_hi);

        __m128 theta6_lo = _mm_mul_ps(theta2_lo, theta4_lo);
        __m128 theta6_hi = _mm_mul_ps(theta2_hi, theta4_hi);

        __m128 theta8_lo = _mm_mul_ps(theta4_lo, theta4_lo);
        __m128 theta8_hi = _mm_mul_ps(theta4_hi, theta4_hi);

        // float theta_jx=theta+k1*theta2+k2*theta4+k3*theta6+k4*theta8
        __m128 theta_jx_lo =
            _mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_add_ps(theta_lo, _mm_mul_ps(k1, theta2_lo)), _mm_mul_ps(k2, theta4_lo)), _mm_mul_ps(k3, theta6_lo)),
                       _mm_mul_ps(k4, theta8_lo));

        __m128 theta_jx_hi =
            _mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_add_ps(theta_hi, _mm_mul_ps(k1, theta2_hi)), _mm_mul_ps(k2, theta4_hi)), _mm_mul_ps(k3, theta6_hi)),
                       _mm_mul_ps(k4, theta8_hi));

        // float tx=(theta_jx/r)*tx
        tx_lo = _mm_mul_ps(_mm_div_ps(theta_jx_lo, r_lo), tx_lo);
        tx_hi = _mm_mul_ps(_mm_div_ps(theta_jx_hi, r_hi), tx_hi);

        // float ty=(theta_jx/r)*ty
        ty_lo = _mm_mul_ps(_mm_div_ps(theta_jx_lo, r_lo), ty_lo);
        ty_hi = _mm_mul_ps(_mm_div_ps(theta_jx_hi, r_hi), ty_hi);

        // pixel[0] = tx * color_K_0_0_ + color_K_0_2_;
        // pixel[1] = ty * color_K_1_1_ + color_K_1_2_;
        __m128 pixelx_lo = _mm_add_ps(_mm_mul_ps(tx_lo, color_K_0_0), color_K_0_2);
        __m128 pixelx_hi = _mm_add_ps(_mm_mul_ps(tx_hi, color_K_0_0), color_K_0_2);
        __m128 pixely_lo = _mm_add_ps(_mm_mul_ps(ty_lo, color_K_1_1), color_K_1_2);
        __m128 pixely_hi = _mm_add_ps(_mm_mul_ps(ty_hi, color_K_1_1), color_K_1_2);

        // pixelx_lo = _mm_add_ps(pixelx_lo, point_five);
        // pixely_lo = _mm_add_ps(pixely_lo, point_five);
        // pixelx_hi = _mm_add_ps(pixelx_hi, point_five);
        // pixely_hi = _mm_add_ps(pixely_hi, point_five);

        __m128 cmp1_lo     = _mm_cmpge_ps(pixelx_lo, x1_limit);
        __m128 cmp2_lo     = _mm_cmpge_ps(pixely_lo, y1_limit);
        __m128 cmp3_lo     = _mm_cmplt_ps(pixelx_lo, x2_limit);
        __m128 cmp4_lo     = _mm_cmplt_ps(pixely_lo, y2_limit);
        __m128 cmp1_hi     = _mm_cmpge_ps(pixelx_hi, x1_limit);
        __m128 cmp2_hi     = _mm_cmpge_ps(pixely_hi, y1_limit);
        __m128 cmp3_hi     = _mm_cmplt_ps(pixelx_hi, x2_limit);
        __m128 cmp4_hi     = _mm_cmplt_ps(pixely_hi, y2_limit);
        __m128 cmp_flag_lo = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_lo, cmp2_lo), cmp3_lo), cmp4_lo);
        __m128 cmp_flag_hi = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_hi, cmp2_hi), cmp3_hi), cmp4_hi);

        __m128 result_x_lo = _mm_and_ps(cmp_flag_lo, pixelx_lo);
        __m128 result_y_lo = _mm_and_ps(cmp_flag_lo, pixely_lo);
        __m128 result_z_lo = _mm_and_ps(cmp_flag_lo, Z_lo);
        __m128 result_x_hi = _mm_and_ps(cmp_flag_hi, pixelx_hi);
        __m128 result_y_hi = _mm_and_ps(cmp_flag_hi, pixely_hi);
        __m128 result_z_hi = _mm_and_ps(cmp_flag_hi, Z_hi);

        float x_lo[4] = { 0 };
        float y_lo[4] = { 0 };
        float z_lo[4] = { 0 };
        float x_hi[4] = { 0 };
        float y_hi[4] = { 0 };
        float z_hi[4] = { 0 };

        _mm_storeu_ps(x_lo, result_x_lo);
        _mm_storeu_ps(y_lo, result_y_lo);
        _mm_storeu_ps(z_lo, result_z_lo);
        _mm_storeu_ps(x_hi, result_x_hi);
        _mm_storeu_ps(y_hi, result_y_hi);
        _mm_storeu_ps(z_hi, result_z_hi);

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_lo[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_lo[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_lo[j]);
                out_depth[pos]       = depth_value;
            }
        }

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_hi[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_hi[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_hi[j]);
                out_depth[pos]       = depth_value;
            }
        }
    }

    if(true) {
        int pixnum = color_width * color_height;
#pragma omp parallel for
        for(int idx = 0; idx < pixnum; idx++) {
            if(65535 == out_depth[idx]) {
                out_depth[idx] = 0;
            }
        }
    }

    return 0;
}

/// TODO(timon): error handling
int AlignImpl::distortedD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height) {
    if(!initialized_) {
        LOG_ERROR("Make sure LoadParameters() success before D2C!");
        return -1;
    }

    auto finder_x = rot_coeff_ht_x.find(std::make_pair(depth_width, depth_height));
    auto finder_y = rot_coeff_ht_y.find(std::make_pair(depth_width, depth_height));
    auto finder_z = rot_coeff_ht_z.find(std::make_pair(depth_width, depth_height));
    if(rot_coeff_ht_x.cend() == finder_x) {
        LOG_ERROR("Found a new resolution, but initialize failed!");
        return -1;
    }

    if(!depth_buffer || !out_depth) {
        LOG_ERROR("depth_buffer is NULL!");
        return -1;
    }

    /// TODO(timon)
    if(true)
        memset(out_depth, 0xff, color_height * color_width * sizeof(uint16_t));
    else
        memset(out_depth, 0x0, color_height * color_width * sizeof(uint16_t));

    const float *coeff_mat_x = finder_x->second;
    const float *coeff_mat_y = finder_y->second;
    const float *coeff_mat_z = finder_z->second;

    __m128 x1_limit = _mm_set_ps1(float(x_start_));
    __m128 x2_limit = _mm_set_ps1(float(x_end_));
    __m128 y1_limit = _mm_set_ps1(float(y_start_));
    __m128 y2_limit = _mm_set_ps1(float(y_end_));

    __m128 k1 = _mm_set_ps1(rgb_disto_.k1);
    __m128 k2 = _mm_set_ps1(rgb_disto_.k2);
    __m128 k3 = _mm_set_ps1(rgb_disto_.k3);
    __m128 p1 = _mm_set_ps1(rgb_disto_.p1);
    __m128 p2 = _mm_set_ps1(rgb_disto_.p2);

    __m128 scaled_trans_1 = _mm_set_ps1(scaled_trans_[0]);
    __m128 scaled_trans_2 = _mm_set_ps1(scaled_trans_[1]);
    __m128 scaled_trans_3 = _mm_set_ps1(scaled_trans_[2]);

    __m128 color_K_0_0 = _mm_set_ps1(rgb_intric_.fx);
    __m128 color_K_0_2 = _mm_set_ps1(rgb_intric_.cx);
    __m128 color_K_1_1 = _mm_set_ps1(rgb_intric_.fy);
    __m128 color_K_1_2 = _mm_set_ps1(rgb_intric_.cy);

    __m128  point_five = _mm_set_ps1(0.5);
    __m128  two        = _mm_set_ps1(2);
    __m128i zero       = _mm_setzero_si128();

    int imgSize = depth_width * depth_height;

#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
    for(int i = 0; i < imgSize; i += 8) {
        __m128i depth_i16    = _mm_loadu_si128((__m128i *)(depth_buffer + i));
        __m128i depth_lo_i   = _mm_unpacklo_epi16(depth_i16, zero);
        __m128i depth_hi_i   = _mm_unpackhi_epi16(depth_i16, zero);
        __m128  depth_sse_lo = _mm_cvtepi32_ps(depth_lo_i);
        __m128  depth_sse_hi = _mm_cvtepi32_ps(depth_hi_i);

        __m128 coeff_sse1_lo = _mm_loadu_ps(coeff_mat_x + i);
        __m128 coeff_sse1_hi = _mm_loadu_ps(coeff_mat_x + i + 4);
        __m128 coeff_sse2_lo = _mm_loadu_ps(coeff_mat_y + i);
        __m128 coeff_sse2_hi = _mm_loadu_ps(coeff_mat_y + i + 4);
        __m128 coeff_sse3_lo = _mm_loadu_ps(coeff_mat_z + i);
        __m128 coeff_sse3_hi = _mm_loadu_ps(coeff_mat_z + i + 4);
        __m128 X_lo          = _mm_mul_ps(depth_sse_lo, coeff_sse1_lo);
        __m128 X_hi          = _mm_mul_ps(depth_sse_hi, coeff_sse1_hi);
        __m128 Y_lo          = _mm_mul_ps(depth_sse_lo, coeff_sse2_lo);
        __m128 Y_hi          = _mm_mul_ps(depth_sse_hi, coeff_sse2_hi);
        __m128 Z_lo          = _mm_mul_ps(depth_sse_lo, coeff_sse3_lo);
        __m128 Z_hi          = _mm_mul_ps(depth_sse_hi, coeff_sse3_hi);
        X_lo                 = _mm_add_ps(X_lo, scaled_trans_1);
        X_hi                 = _mm_add_ps(X_hi, scaled_trans_1);
        Y_lo                 = _mm_add_ps(Y_lo, scaled_trans_2);
        Y_hi                 = _mm_add_ps(Y_hi, scaled_trans_2);
        Z_lo                 = _mm_add_ps(Z_lo, scaled_trans_3);
        Z_hi                 = _mm_add_ps(Z_hi, scaled_trans_3);
        __m128 tx_lo         = _mm_div_ps(X_lo, Z_lo);
        __m128 ty_lo         = _mm_div_ps(Y_lo, Z_lo);
        __m128 tx_hi         = _mm_div_ps(X_hi, Z_hi);
        __m128 ty_hi         = _mm_div_ps(Y_hi, Z_hi);

        __m128 x2_lo = _mm_mul_ps(tx_lo, tx_lo);
        __m128 y2_lo = _mm_mul_ps(ty_lo, ty_lo);
        __m128 xy_lo = _mm_mul_ps(tx_lo, ty_lo);
        __m128 r2_lo = _mm_add_ps(x2_lo, y2_lo);
        __m128 r4_lo = _mm_mul_ps(r2_lo, r2_lo);
        __m128 r6_lo = _mm_mul_ps(r4_lo, r2_lo);
        __m128 x2_hi = _mm_mul_ps(tx_hi, tx_hi);
        __m128 y2_hi = _mm_mul_ps(ty_hi, ty_hi);
        __m128 xy_hi = _mm_mul_ps(tx_hi, ty_hi);
        __m128 r2_hi = _mm_add_ps(x2_hi, y2_hi);
        __m128 r4_hi = _mm_mul_ps(r2_hi, r2_hi);
        __m128 r6_hi = _mm_mul_ps(r4_hi, r2_hi);

        // float k_jx = k1 * r2 + k2 * r4 + k3 * r6;
        __m128 k_jx_lo = _mm_add_ps(_mm_add_ps(_mm_mul_ps(k1, r2_lo), _mm_mul_ps(k2, r4_lo)), _mm_mul_ps(k3, r6_lo));
        __m128 k_jx_hi = _mm_add_ps(_mm_add_ps(_mm_mul_ps(k1, r2_hi), _mm_mul_ps(k2, r4_hi)), _mm_mul_ps(k3, r6_hi));

        // float x_qx = p2 * (2 * x2 + r2) + 2 * p1 * xy;
        __m128 x_qx_lo_1 = _mm_mul_ps(p2, _mm_add_ps(_mm_mul_ps(x2_lo, two), r2_lo));
        __m128 x_qx_lo_2 = _mm_mul_ps(_mm_mul_ps(p1, xy_lo), two);
        __m128 x_qx_lo   = _mm_add_ps(x_qx_lo_1, x_qx_lo_2);
        __m128 x_qx_hi_1 = _mm_mul_ps(p2, _mm_add_ps(_mm_mul_ps(x2_hi, two), r2_hi));
        __m128 x_qx_hi_2 = _mm_mul_ps(_mm_mul_ps(p1, xy_hi), two);
        __m128 x_qx_hi   = _mm_add_ps(x_qx_hi_1, x_qx_hi_2);

        // float y_qx = p1 * (2 * y2 + r2) + 2 * p2 * xy;
        __m128 y_qx_lo_1 = _mm_mul_ps(p1, _mm_add_ps(_mm_mul_ps(y2_lo, two), r2_lo));
        __m128 y_qx_lo_2 = _mm_mul_ps(_mm_mul_ps(p2, xy_lo), two);
        __m128 y_qx_lo   = _mm_add_ps(y_qx_lo_1, y_qx_lo_2);
        __m128 y_qx_hi_1 = _mm_mul_ps(p1, _mm_add_ps(_mm_mul_ps(y2_hi, two), r2_hi));
        __m128 y_qx_hi_2 = _mm_mul_ps(_mm_mul_ps(p2, xy_hi), two);
        __m128 y_qx_hi   = _mm_add_ps(y_qx_hi_1, y_qx_hi_2);

        // float distx = tx * k_jx + x_qx;
        __m128 distx_lo = _mm_add_ps(_mm_mul_ps(tx_lo, k_jx_lo), x_qx_lo);
        __m128 distx_hi = _mm_add_ps(_mm_mul_ps(tx_hi, k_jx_hi), x_qx_hi);

        // float disty = ty * k_jx + y_qx;
        __m128 disty_lo = _mm_add_ps(_mm_mul_ps(ty_lo, k_jx_lo), y_qx_lo);
        __m128 disty_hi = _mm_add_ps(_mm_mul_ps(ty_hi, k_jx_hi), y_qx_hi);

        // tx = tx + distx;
        // ty = ty + disty;
        tx_lo = _mm_add_ps(tx_lo, distx_lo);
        tx_hi = _mm_add_ps(tx_hi, distx_hi);
        ty_lo = _mm_add_ps(ty_lo, disty_lo);
        ty_hi = _mm_add_ps(ty_hi, disty_hi);

        // pixel[0] = tx * color_K_0_0_ + color_K_0_2_;
        // pixel[1] = ty * color_K_1_1_ + color_K_1_2_;
        __m128 pixelx_lo = _mm_add_ps(_mm_mul_ps(tx_lo, color_K_0_0), color_K_0_2);
        __m128 pixelx_hi = _mm_add_ps(_mm_mul_ps(tx_hi, color_K_0_0), color_K_0_2);
        __m128 pixely_lo = _mm_add_ps(_mm_mul_ps(ty_lo, color_K_1_1), color_K_1_2);
        __m128 pixely_hi = _mm_add_ps(_mm_mul_ps(ty_hi, color_K_1_1), color_K_1_2);

        // pixelx_lo = _mm_add_ps(pixelx_lo, point_five);
        // pixely_lo = _mm_add_ps(pixely_lo, point_five);
        // pixelx_hi = _mm_add_ps(pixelx_hi, point_five);
        // pixely_hi = _mm_add_ps(pixely_hi, point_five);

        __m128 cmp1_lo     = _mm_cmpge_ps(pixelx_lo, x1_limit);
        __m128 cmp2_lo     = _mm_cmpge_ps(pixely_lo, y1_limit);
        __m128 cmp3_lo     = _mm_cmplt_ps(pixelx_lo, x2_limit);
        __m128 cmp4_lo     = _mm_cmplt_ps(pixely_lo, y2_limit);
        __m128 cmp1_hi     = _mm_cmpge_ps(pixelx_hi, x1_limit);
        __m128 cmp2_hi     = _mm_cmpge_ps(pixely_hi, y1_limit);
        __m128 cmp3_hi     = _mm_cmplt_ps(pixelx_hi, x2_limit);
        __m128 cmp4_hi     = _mm_cmplt_ps(pixely_hi, y2_limit);
        __m128 cmp_flag_lo = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_lo, cmp2_lo), cmp3_lo), cmp4_lo);
        __m128 cmp_flag_hi = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_hi, cmp2_hi), cmp3_hi), cmp4_hi);

        __m128 result_x_lo = _mm_and_ps(cmp_flag_lo, pixelx_lo);
        __m128 result_y_lo = _mm_and_ps(cmp_flag_lo, pixely_lo);
        __m128 result_z_lo = _mm_and_ps(cmp_flag_lo, Z_lo);
        __m128 result_x_hi = _mm_and_ps(cmp_flag_hi, pixelx_hi);
        __m128 result_y_hi = _mm_and_ps(cmp_flag_hi, pixely_hi);
        __m128 result_z_hi = _mm_and_ps(cmp_flag_hi, Z_hi);

        float x_lo[4] = { 0 };
        float y_lo[4] = { 0 };
        float z_lo[4] = { 0 };
        float x_hi[4] = { 0 };
        float y_hi[4] = { 0 };
        float z_hi[4] = { 0 };

        _mm_storeu_ps(x_lo, result_x_lo);
        _mm_storeu_ps(y_lo, result_y_lo);
        _mm_storeu_ps(z_lo, result_z_lo);
        _mm_storeu_ps(x_hi, result_x_hi);
        _mm_storeu_ps(y_hi, result_y_hi);
        _mm_storeu_ps(z_hi, result_z_hi);

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_lo[j]))
                continue;

            /// TODO(timon)
            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_lo[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_lo[j]);
                out_depth[pos]       = depth_value;
            }
        }

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_hi[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_hi[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_hi[j]);
                out_depth[pos]       = depth_value;
            }
        }
    }

    if(true) {
        int pixnum = color_width * color_height;
#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
        for(int idx = 0; idx < pixnum; idx++) {
            if(65535 == out_depth[idx]) {
                out_depth[idx] = 0;
            }
        }
    }

    return 0;
}

/// TOOD(timon): error handeling
int AlignImpl::linearD2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height) {
    if(!initialized_) {
        LOG_ERROR("Make sure LoadParameters() success before D2C!");
        return -1;
    }

    auto finder_x = rot_coeff_ht_x.find(std::make_pair(depth_width, depth_height));
    auto finder_y = rot_coeff_ht_y.find(std::make_pair(depth_width, depth_height));
    auto finder_z = rot_coeff_ht_z.find(std::make_pair(depth_width, depth_height));

    if(rot_coeff_ht_x.cend() == finder_x) {
        LOG_ERROR("Found a new resolution, but initialize failed!");
        return -1;
    }

    if(!depth_buffer || !out_depth) {
        LOG_ERROR("depth_buffer is NULL!");
        return -1;
    }

    if(true)
        memset(out_depth, 0xff, color_height * color_width * sizeof(uint16_t));
    else
        memset(out_depth, 0x0, color_height * color_width * sizeof(uint16_t));

    const float *coeff_mat_x = finder_x->second;
    const float *coeff_mat_y = finder_y->second;
    const float *coeff_mat_z = finder_z->second;

    __m128 x1 = _mm_set_ps1(float(x_start_));
    __m128 x2 = _mm_set_ps1(float(x_end_));
    __m128 y1 = _mm_set_ps1(float(y_start_));
    __m128 y2 = _mm_set_ps1(float(y_end_));

    __m128 scaled_trans_1 = _mm_set_ps1(scaled_trans_[0]);
    __m128 scaled_trans_2 = _mm_set_ps1(scaled_trans_[1]);
    __m128 scaled_trans_3 = _mm_set_ps1(scaled_trans_[2]);

    __m128 color_K_0_0 = _mm_set_ps1(rgb_intric_.fx);
    __m128 color_K_0_2 = _mm_set_ps1(rgb_intric_.cx);
    __m128 color_K_1_1 = _mm_set_ps1(rgb_intric_.fy);
    __m128 color_K_1_2 = _mm_set_ps1(rgb_intric_.cy);

    __m128  round_point_5 = _mm_set_ps1(0.5);
    __m128i zero          = _mm_setzero_si128();

    int imgSize = depth_width * depth_height;

#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
    for(int i = 0; i < imgSize; i += 8) {
        __m128i depth_i16 = _mm_loadu_si128((__m128i *)(depth_buffer + i));

        __m128i depth_lo_i   = _mm_unpacklo_epi16(depth_i16, zero);
        __m128i depth_hi_i   = _mm_unpackhi_epi16(depth_i16, zero);
        __m128  depth_sse_lo = _mm_cvtepi32_ps(depth_lo_i);
        __m128  depth_sse_hi = _mm_cvtepi32_ps(depth_hi_i);

        __m128 coeff_sse1_lo = _mm_loadu_ps(coeff_mat_x + i);
        __m128 coeff_sse1_hi = _mm_loadu_ps(coeff_mat_x + i + 4);
        __m128 coeff_sse2_lo = _mm_loadu_ps(coeff_mat_y + i);
        __m128 coeff_sse2_hi = _mm_loadu_ps(coeff_mat_y + i + 4);
        __m128 coeff_sse3_lo = _mm_loadu_ps(coeff_mat_z + i);
        __m128 coeff_sse3_hi = _mm_loadu_ps(coeff_mat_z + i + 4);

        __m128 X_lo = _mm_mul_ps(depth_sse_lo, coeff_sse1_lo);
        __m128 X_hi = _mm_mul_ps(depth_sse_hi, coeff_sse1_hi);
        __m128 Y_lo = _mm_mul_ps(depth_sse_lo, coeff_sse2_lo);
        __m128 Y_hi = _mm_mul_ps(depth_sse_hi, coeff_sse2_hi);
        __m128 Z_lo = _mm_mul_ps(depth_sse_lo, coeff_sse3_lo);
        __m128 Z_hi = _mm_mul_ps(depth_sse_hi, coeff_sse3_hi);

        X_lo = _mm_add_ps(X_lo, scaled_trans_1);
        X_hi = _mm_add_ps(X_hi, scaled_trans_1);
        Y_lo = _mm_add_ps(Y_lo, scaled_trans_2);
        Y_hi = _mm_add_ps(Y_hi, scaled_trans_2);
        Z_lo = _mm_add_ps(Z_lo, scaled_trans_3);
        Z_hi = _mm_add_ps(Z_hi, scaled_trans_3);

        __m128 tx_lo = _mm_div_ps(X_lo, Z_lo);
        __m128 tx_hi = _mm_div_ps(X_hi, Z_hi);
        __m128 ty_lo = _mm_div_ps(Y_lo, Z_lo);
        __m128 ty_hi = _mm_div_ps(Y_hi, Z_hi);

        // pixel[0] = tx * color_K_0_0_ + color_K_0_2_;
        // pixel[1] = ty * color_K_1_1_ + color_K_1_2_;
        __m128 pixelx_lo = _mm_add_ps(_mm_mul_ps(tx_lo, color_K_0_0), color_K_0_2);
        __m128 pixelx_hi = _mm_add_ps(_mm_mul_ps(tx_hi, color_K_0_0), color_K_0_2);
        __m128 pixely_lo = _mm_add_ps(_mm_mul_ps(ty_lo, color_K_1_1), color_K_1_2);
        __m128 pixely_hi = _mm_add_ps(_mm_mul_ps(ty_hi, color_K_1_1), color_K_1_2);

        __m128 cmp1_lo = _mm_cmpge_ps(pixelx_lo, x1);
        __m128 cmp2_lo = _mm_cmpge_ps(pixely_lo, y1);
        __m128 cmp3_lo = _mm_cmplt_ps(pixelx_lo, x2);
        __m128 cmp4_lo = _mm_cmplt_ps(pixely_lo, y2);

        __m128 cmp_flag_lo = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_lo, cmp2_lo), cmp3_lo), cmp4_lo);
        __m128 result_x_lo = _mm_and_ps(cmp_flag_lo, pixelx_lo);
        __m128 result_y_lo = _mm_and_ps(cmp_flag_lo, pixely_lo);
        __m128 result_z_lo = _mm_and_ps(cmp_flag_lo, Z_lo);

        __m128 cmp1_hi     = _mm_cmpge_ps(pixelx_hi, x1);
        __m128 cmp2_hi     = _mm_cmpge_ps(pixely_hi, y1);
        __m128 cmp3_hi     = _mm_cmplt_ps(pixelx_hi, x2);
        __m128 cmp4_hi     = _mm_cmplt_ps(pixely_hi, y2);
        __m128 cmp_flag_hi = _mm_and_ps(_mm_and_ps(_mm_and_ps(cmp1_hi, cmp2_hi), cmp3_hi), cmp4_hi);
        __m128 result_x_hi = _mm_and_ps(cmp_flag_hi, pixelx_hi);
        __m128 result_y_hi = _mm_and_ps(cmp_flag_hi, pixely_hi);
        __m128 result_z_hi = _mm_and_ps(cmp_flag_hi, Z_hi);

        float x_lo[4] = { 0 };
        float y_lo[4] = { 0 };
        float z_lo[4] = { 0 };
        float x_hi[4] = { 0 };
        float y_hi[4] = { 0 };
        float z_hi[4] = { 0 };

        _mm_storeu_ps(x_lo, result_x_lo);
        _mm_storeu_ps(y_lo, result_y_lo);
        _mm_storeu_ps(z_lo, result_z_lo);
        _mm_storeu_ps(x_hi, result_x_hi);
        _mm_storeu_ps(y_hi, result_y_hi);
        _mm_storeu_ps(z_hi, result_z_hi);

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_lo[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos       = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t cur_depth = static_cast<uint16_t>(z_lo[j]);

                bool b_cur                       = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_lo[j]) * color_width + static_cast<int>(x_lo[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_lo[j]);
                out_depth[pos]       = depth_value;
            }
        }

        for(int j = 0; j < 4; j++) {
            if(0 == static_cast<uint16_t>(z_hi[j]))
                continue;

            if(gap_fill_copy_) {
                int      pos         = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_hi[j]);

                uint16_t cur_depth               = depth_value;
                bool     b_cur                   = out_depth[pos] < cur_depth;
                out_depth[pos]                   = b_cur * out_depth[pos] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + 1] < cur_depth;
                out_depth[pos + 1]               = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width] < cur_depth;
                out_depth[pos + color_width]     = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;
                b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
            }
            else {
                int      pos         = static_cast<int>(y_hi[j]) * color_width + static_cast<int>(x_hi[j]);
                uint16_t depth_value = static_cast<uint16_t>(z_hi[j]);
                out_depth[pos]       = depth_value;
            }
        }
    }

    if(true) {
        int pixnum = color_width * color_height;
#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
        for(int idx = 0; idx < pixnum; idx++) {
            if(65535 == out_depth[idx]) {
                out_depth[idx] = 0;
            }
        }
    }

    return 0;
}

bool AlignImpl::undistortRGB(const uint8_t *src, int width, int height, uint8_t *dst) {
    if((!src) || (width < 0) || (height < 0) || (!dst) || (nullptr == rgb_dx_lut_) || (nullptr == rgb_dy_lut_) || (width != rgb_lut_width_)
       || (height != rgb_lut_height_))
        return false;
    else {
        float du = 0, dv = 0;
        for(int v = 0; v < height; v++) {
            uint8_t *tgt = dst + v * 3 * width;
            for(int u = 0; u < width; u++) {
                int lut_idx = v * width + u;
                du          = rgb_dx_lut_[lut_idx];
                dv          = rgb_dy_lut_[lut_idx];
                if((du > -1) && (dv > -1)) {
                    tgt[u * 3]     = BilinearInterpolationRGB(src, width, height, du, dv, 0, 0);
                    tgt[u * 3 + 1] = BilinearInterpolationRGB(src, width, height, du, dv, 0, 1);
                    tgt[u * 3 + 2] = BilinearInterpolationRGB(src, width, height, du, dv, 0, 2);
                }
            }
        }
        return true;
    }
}

/// TODO(timon): error handling
int AlignImpl::D2C(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height, int *depth_xy) {
    if(!initialized_ || depth_width != depth_intric_.width || depth_height != depth_intric_.height || color_width != rgb_intric_.width
       || color_height != rgb_intric_.height) {
        LOG_ERROR("Not initialized or input parameters don't match");
        return -2;
    }

    auto finder_x = rot_coeff_ht_x.find(std::make_pair(depth_width, depth_height));
    auto finder_y = rot_coeff_ht_y.find(std::make_pair(depth_width, depth_height));
    auto finder_z = rot_coeff_ht_z.find(std::make_pair(depth_width, depth_height));
    if((rot_coeff_ht_x.cend() == finder_x) || (rot_coeff_ht_y.cend() == finder_y) || (rot_coeff_ht_z.cend() == finder_z)) {
        LOG_ERROR("Found a new resolution, but initialize failed!");
        return -1;
    }

    if((!depth_buffer) || ((!out_depth && !depth_xy))) {
        LOG_ERROR("Buffer not initialized");
        return -1;
    }

    /// TODO(timon): check default
    // D2C_GAP_FILLED_DISABLE != d2c_cfg_.enable_gap_fill
    if(out_depth) {
        if(true)
            memset(out_depth, 0xff, color_height * color_width * sizeof(uint16_t));
        else
            memset(out_depth, 0x0, color_height * color_width * sizeof(uint16_t));
    }

    const float *coeff_mat_x = finder_x->second;
    const float *coeff_mat_y = finder_y->second;
    const float *coeff_mat_z = finder_z->second;

#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif

    for(int v = 0; v < depth_height; v += 1) {
        float dst[3];
        float pixel[2];

        for(int u = 0; u < depth_width; u += 1) {
            int      i     = v * depth_width + u;
            uint16_t depth = depth_buffer[i];
            dst[0]         = depth * coeff_mat_x[i] + scaled_trans_[0];
            dst[1]         = depth * coeff_mat_y[i] + scaled_trans_[1];
            dst[2]         = depth * coeff_mat_z[i] + scaled_trans_[2];
            int u_rgb      = -1;
            int v_rgb      = -1;

            if(fabs(dst[2]) < EPSILON) {
                continue;
            }
            else {
                float tx = float(dst[0] / dst[2]);
                float ty = float(dst[1] / dst[2]);

                if(add_target_distortion_) {
                    float pt_ud[2] = { tx, ty };
                    float pt_d[2];
                    float r2_cur = pt_ud[0] * pt_ud[0] + pt_ud[1] * pt_ud[1];
                    if((r2_max_loc_ != 0) && (r2_cur > r2_max_loc_))
                        continue;
                    addDistortion(rgb_disto_, rgb_intric_.model, pt_ud, pt_d);
                    tx = pt_d[0];
                    ty = pt_d[1];
                }

                pixel[0] = tx * rgb_intric_.fx + rgb_intric_.cx;
                pixel[1] = ty * rgb_intric_.fy + rgb_intric_.cy;
            }

            if(pixel[0] < x_start_ || pixel[0] >= x_end_ || pixel[1] < y_start_ || pixel[1] >= y_end_)
                continue;

            u_rgb = static_cast<int>(pixel[0]);
            v_rgb = static_cast<int>(pixel[1]);
            if(depth_xy)  // coordinates mapping for C2D
            {
                depth_xy[2 * i]     = u_rgb;
                depth_xy[2 * i + 1] = v_rgb;
                /// TODO(timon): filling for C2D
            }
            if(out_depth) {
                if(gap_fill_copy_) {
                    int      pos       = v_rgb * color_width + u_rgb;
                    uint16_t cur_depth = uint16_t(dst[2]);

                    bool b_cur     = out_depth[pos] < cur_depth;
                    out_depth[pos] = b_cur * out_depth[pos] + !b_cur * cur_depth;

                    b_cur              = out_depth[pos + 1] < cur_depth;
                    out_depth[pos + 1] = b_cur * out_depth[pos + 1] + !b_cur * cur_depth;

                    b_cur                        = out_depth[pos + color_width] < cur_depth;
                    out_depth[pos + color_width] = b_cur * out_depth[pos + color_width] + !b_cur * cur_depth;

                    b_cur                            = out_depth[pos + color_width + 1] < cur_depth;
                    out_depth[pos + color_width + 1] = b_cur * out_depth[pos + color_width + 1] + !b_cur * cur_depth;
                }
                else {
                    u_rgb          = static_cast<int>(pixel[0]);
                    v_rgb          = static_cast<int>(pixel[1]);
                    uint16_t *pdst = out_depth + v_rgb * color_width + u_rgb;
                    *pdst          = (uint16_t)dst[2];
                }
            }
        }
    }

    /// TODO(timon): check default
    if(true) {
        int pixnum = color_width * color_height;
#if !defined(ANDROID) && !defined(__ANDROID__)
#pragma omp parallel for
#endif
        for(int idx = 0; idx < pixnum; idx++) {
            if(65535 == out_depth[idx]) {
                out_depth[idx] = 0;
            }
        }
    }

    return 0;
}

int AlignImpl::D2CWithSSE(const uint16_t *depth_buffer, int depth_width, int depth_height, uint16_t *out_depth, int color_width, int color_height) {
    int ret = 0;
    if(depth_width != depth_intric_.width || depth_height != depth_intric_.height || color_width != rgb_intric_.width || color_height != rgb_intric_.height) {
        LOG_ERROR("Input parameters don't match");
        return -2;
    }

    if(add_target_distortion_) {
        switch(rgb_intric_.model) {
        case OB_DISTORTION_BROWN_CONRADY:
            ret = distortedD2CWithSSE(depth_buffer, depth_width, depth_height, out_depth, color_width, color_height);
            break;
        case OB_DISTORTION_BROWN_CONRADY_K6:
            ret = BMDistortedD2CWithSSE(depth_buffer, depth_width, depth_height, out_depth, color_width, color_height);
            break;
        case OB_DISTORTION_KANNALA_BRANDT4:
            ret = KBDistortedD2CWithSSE(depth_buffer, depth_width, depth_height, out_depth, color_width, color_height);
            break;
        default:
            LOG_ERROR("Distortion model not supported yet");
            break;
        }
    }
    else {
        ret = linearD2CWithSSE(depth_buffer, depth_width, depth_height, out_depth, color_width, color_height);
    }

    return ret;
}

bool AlignImpl::initRGBUndistortion() {
    if(!initialized_) {
        ("Make sure LoadParameters() success before InitRGBUndistortion!");
        return false;
    }

    return prepareRGBDistort();
}

bool AlignImpl::prepareRGBDistort() {
    unsigned int width = rgb_intric_.width, height = rgb_intric_.height;
    rgb_lut_width_  = width;
    rgb_lut_height_ = height;
    size_t num = width * height, size = num * sizeof(double);
    rgb_dx_lut_ = (float *)malloc(size);
    rgb_dy_lut_ = (float *)malloc(size);
    if(rgb_dx_lut_)
        memset(rgb_dx_lut_, 0, size);
    if(rgb_dy_lut_)
        memset(rgb_dy_lut_, 0, size);

    float  fx = rgb_intric_.fx, fy = rgb_intric_.fy, cx = rgb_intric_.cx, cy = rgb_intric_.cy;
    float  x, y, dx, dy;
    size_t idx = 0;
    for(size_t v = 0; v < height; v++) {
        y = (v - cy) / fy;
        for(size_t u = 0; u < width; u++) {
            x        = (u - cx) / fx;
            float r2 = x * x + y * y;
            if((r2_max_loc_ != 0) && (r2 > r2_max_loc_)) {
                dx = -1;
                dy = -1;
            }
            else {
                float pt_ud[2] = { x, y };
                float pt_d[2];
                addDistortion(rgb_disto_, rgb_intric_.model, pt_ud, pt_d);

                // if (true) // UNIT_IN_PIXEL
                {
                    dx = pt_d[0] * fx + cx;
                    dy = pt_d[1] * fy + cy;
                    if((dx < 0) || (dx > width - 1) || (dy < 0) || (dy > height - 1)) {
                        dx = -1;
                        dy = -1;
                    }
                }
            }
            idx              = v * width + u;
            rgb_dx_lut_[idx] = dx;
            rgb_dy_lut_[idx] = dy;
        }
    }

    return true;
}

typedef struct {
    unsigned char byte[3];
} uint24_t;

int AlignImpl::C2D(const uint16_t *depth_buffer, int depth_width, int depth_height, const void *rgb_buffer, void *out_rgb, int color_width, int color_height,
                   OBFormat format) {

    // rgb x-y coordinates for each depth pixel
    unsigned long long size     = static_cast<unsigned long long>(depth_width) * depth_height * 2;
    int *              depth_xy = new int[size];
    memset(depth_xy, -1, size * sizeof(int));

    /// TODO(timon): error handling
    if(!D2C(depth_buffer, depth_width, depth_height, nullptr, color_width, color_height, depth_xy)) {

        switch(format) {
        case OB_FORMAT_Y8:
            mapBytes<uint8_t>(depth_xy, static_cast<const uint8_t *>(rgb_buffer), color_width, color_height, (uint8_t *)out_rgb, depth_width);
            break;
        case OB_FORMAT_Y16:
            mapBytes<uint16_t>(depth_xy, static_cast<const uint16_t *>(rgb_buffer), color_width, color_height, (uint16_t *)out_rgb, depth_width);
            break;
        case OB_FORMAT_RGB:
            mapBytes<uint24_t>(depth_xy, static_cast<const uint24_t *>(rgb_buffer), color_width, color_height, (uint24_t *)out_rgb, depth_width);
            break;
        case OB_FORMAT_BGR:
        case OB_FORMAT_BGRA:
        case OB_FORMAT_RGBA:
        case OB_FORMAT_MJPG:
        case OB_FORMAT_YUYV:
        default:
            LOG_ERROR("Not supported yet");
            break;
        }
    }
    delete[] depth_xy;
    depth_xy = nullptr;

    return 0;
}

template <typename T> void libobsensor::AlignImpl::mapBytes(const int *map, const T *src_buffer, int src_width, int src_height, T *dst_buffer, int dst_width) {
    for(int v = y_start_; v < y_end_; v++) {
        for(int u = x_start_; u < x_end_; u++) {
            int id = v * dst_width + u;
            int us = map[2 * id], vs = map[2 * id + 1];
            if((us < 0) || (us > src_width - 1) || (vs < 0) || (vs > src_height - 1))
                continue;
            int is         = vs * src_width + us;
            dst_buffer[id] = src_buffer[is];
        }
    }
}

}  // namespace libobsensor
