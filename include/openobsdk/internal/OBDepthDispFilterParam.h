#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @file OBDepthDispFilterParam.h
 * @brief 深度滤波参数，仅内部使用，用于工程软件生产标定
 */

#ifdef OB_SENSOR_SDK_DEVELOPER

/**
 * @brief 后处理类型
 */
typedef enum DDO_TYPE : uint32_t {
    NOISE_REMOVAL           = (1 << 0),  // 去噪
    EDGE_NOISE_REMOVAL      = (1 << 1),  // 边缘去噪
    SPATIAL_FILTER_FAST     = (1 << 2),  // 快速空域滤波
    SPATIAL_FILTER_MODERATE = (1 << 3),  // 适中空域滤波
    SPATIAL_FILTER_ADVANCED = (1 << 4),  // 增强空域滤波
    TEMPORAL_FILTER         = (1 << 5),  // 时域滤波
    HOLE_FILLING            = (1 << 6),  // 填洞滤波
} DDOType, ddo_type;

/**
 * @brief 去噪方式
 */
typedef enum DDO_NOISE_REMOVAL_TYPE {
    NR_LUT     = 0,  // SPLIT
    NR_OVERALL = 1,  // NON_SPLIT
} DDONoiseRemovalType,
    ddo_noise_removal_type;

/**
 * @brief 边缘去噪方式
 */
typedef enum DDO_EDGE_NOISE_REMOVAL_TYPE {
    MG_FILTER = 0,
    MGH_FILTER,  // horizontal MG
    MGA_FILTER,  // asym MG
    MGC_FILTER,
    // RM_FILTER,
} DDOEdgeNoiseRemovalType,
    ddo_edge_noise_removal_type;

typedef enum DDO_SPATIAL_FILTER_ADVANCED_TYPE {
    SFA_HORIZONTAL = 0x01,  // 0b01,
    SFA_VERTICAL   = 0x02,  // 0b10,
    SFA_ALL        = 0xff,  // 0b11,
} DDOSpatialAdvancedType,
    ddo_spatial_advanced_type;

/**
 * @brief 填洞方式
 */
typedef enum DDO_HOLE_FILLING_TYPE {
    FILL_TOP     = 0,
    FILL_NEAREST = 1,  // "max" means farest for depth, and nearest for disparity; FILL_NEAREST
    FILL_FAREST  = 2,  // FILL_FAREST
} DDOHoleFillingType,
    ddo_hole_filling_type;

typedef struct NoiseRemovalFilterParams_t {
    uint16_t            size       = 500;
    uint16_t            disp_diff  = 250;
    DDONoiseRemovalType type       = NR_OVERALL;
    uint16_t            lut[4 * 4] = {
        100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100
    };  // max_size_lut
} NoiseRemovalFilterParams, noise_removal_filter_params;

typedef struct EdgeNoiseRemovalFilterParams_t {
    DDOEdgeNoiseRemovalType type    = MG_FILTER;
    uint16_t                margin_left_th   = 3;
    uint16_t                margin_right_th  = 3;
    uint16_t                margin_top_th    = 3;
    uint16_t                margin_bottom_th = 3;
} EdgeNoiseRemovalFilterParams, edge_noise_removal_filter_params;

typedef struct SpatialFastFilterParams_t {
    uint8_t size = 0;
} SpatialFastFilterParams, spatial_faster_filter_params;

typedef struct SpatialModerateFilterParams_t {
    uint8_t  size      = 3;
    uint8_t  iters     = 0;
    uint16_t disp_diff = 0;
} SpatialModerateFilterParams, spatial_moderate_filter_params;

typedef struct SpatialAdvancedFilterParams_t {
    DDOSpatialAdvancedType type      = SFA_ALL;
    uint8_t                iters     = 1;
    float                  alpha     = 0.5f;
    uint16_t               disp_diff = 250;
    uint16_t               radius    = 3;
} SpatialAdvancedFilterParams, spatial_advanced_filter_params;

typedef struct HoleFillingFilterParams_t {
    DDOHoleFillingType type = FILL_TOP;
} HoleFillingFilterParams, hole_filing_filter_params;

typedef enum DDO_TEMPORAL_FILTER_TYPE {
		TF_FILL_DISABLED = 0,
		TF_VALID_2_IN_8 = 1,
}DDOTemporalFilterType, ddo_temporal_filter_type;

typedef struct TemporalFilterParams_t {
    DDOTemporalFilterType type = TF_FILL_DISABLED;
    float scale  = 0.5f;
    float weight = 0.5f;
} TemporalFilterParams, temporal_filter_params;

typedef struct DDOConfig_t {
    size_t                       width;
    size_t                       height;
    double                       depth_unit;
    uint32_t                     enable_bitmap;
    bool                         depth_flag;
    float                        bxf;
    //uint16_t                     disp_bit_size;
    uint16_t                     fraction_bit_size;
    uint16_t                     invalid_value;
    NoiseRemovalFilterParams     noiseRemovalFilterParams;
    EdgeNoiseRemovalFilterParams edgeNoiseRemovalFilterParams;
    SpatialFastFilterParams      spatialFastFilterParams;
    SpatialModerateFilterParams  spatialModerateFilterParams;
    SpatialAdvancedFilterParams  spatialAdvancedFilterParams;
    HoleFillingFilterParams      holeFillingFilterParams;
    TemporalFilterParams         temporalFilterParams;
} DDOConfig, ob_ddo_config;

#endif