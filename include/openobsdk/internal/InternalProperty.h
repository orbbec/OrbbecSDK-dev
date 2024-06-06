// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

/**
 * @file InternalProperty.h
 * @brief 属性控制命令列表维护, 包含内部研发、产线使用的控制命令
 */

#ifdef OB_SENSOR_SDK_DEVELOPER
#ifndef _OB_PROPERTY_H_
#define _OB_PROPERTY_H_

#include "libobsensor/h/ObTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OB_PROP_BEGIN 0
#define OB_PROP_END 999
#define OB_STRUCT_BEGIN 1000
#define OB_STRUCT_END 1999
#define OB_PROP_SENSOR_BEGIN 2000
#define OB_PROP_SENSOR_END 2999
#define OB_PROP_SDK_BEGIN 3000
#define OB_PROP_SDK_END 3499
#define OB_STRUCT_SDK_TYPE_BEGIN 3500
#define OB_STRUCT_SDK_TYPE_END 3999
#define OB_RAW_DATA_BEGIN 4000
#define OB_RAW_DATA_END 4999
#define OB_DEVICE_DEBUG_PROPERTY_BEGIN 5000
#define OB_DEVICE_DEBUG_PROPERTY_END 5499
#define OB_DEVICE_DEBUG_DATA_TYPE_BEGIN 5500
#define OB_DEVICE_DEBUG_DATA_TYPE_END 5999

/**
 * @brief 描述设备所有的属性的枚举值
 */
typedef enum {
    // 0~999为设备端int, bool及float类型控制命令
    // Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_PROP_FLASH_WRITE_PROTECTION_BOOL     = 0,  /**<FLASH写保护开关;0, 关闭;1, 开启 */
    OB_PROP_TEC_BOOL                        = 1,  /**<TEC开关 */
    OB_PROP_LDP_BOOL                        = 2,  /**<LDP开关 */
    OB_PROP_LASER_BOOL                      = 3,  /**<激光开光 */
    OB_PROP_LASER_PULSE_WIDTH_INT           = 4,  /**<读写激光Time值(脉宽) */
    OB_PROP_LASER_CURRENT_FLOAT             = 5,  /**<激光电流 */
    OB_PROP_FLOOD_BOOL                      = 6,  /**<泛光灯开关 */
    OB_PROP_FLOOD_LEVEL_INT                 = 7,  /**<泛光灯等级 */
    OB_PROP_TEMPERATURE_COMPENSATION_BOOL   = 8,  /**<温度补偿开关 */
    OB_PROP_DEPTH_ALIGN_SOFTWARE_BOOL       = 13, /**<软件D2C 仅保留用于Femto SDK-固件内部使用 */
    OB_PROP_DEPTH_MIRROR_BOOL               = 14, /**<深度镜像 */
    OB_PROP_DEPTH_FLIP_BOOL                 = 15, /**<深度翻转 */
    OB_PROP_DEPTH_POSTFILTER_BOOL           = 16, /**<深度Postfilter */
    OB_PROP_DEPTH_HOLEFILTER_BOOL           = 17, /**<深度Holefilter */
    OB_PROP_IR_MIRROR_BOOL                  = 18, /**<IR 镜像 */
    OB_PROP_IR_FLIP_BOOL                    = 19, /**<IR 翻转 */
    OB_PROP_MIN_DEPTH_INT                   = 22, /**<最小深度阈值 */
    OB_PROP_MAX_DEPTH_INT                   = 23, /**<最大深度阈值 */
    OB_PROP_DEPTH_SOFT_FILTER_BOOL          = 24, /**<软件滤波开关 */
    OB_PROP_DEVICE_RESET_BOOL               = 29, /**<软件复位 */
    OB_PROP_LDP_STATUS_BOOL                 = 32, /**<LDP状态 */
    OB_PROP_STOP_DEPTH_STREAM_BOOL          = 38, /**<关闭深度流（MX6600芯片同时也是右IR流）, 用于无法通过标准UVC协议关流的设备 */
    OB_PROP_STOP_IR_STREAM_BOOL             = 39, /**<关闭IR流（MX6600芯片同时也是左IR流）用于无法通过标准UVC协议关流的设备 */
    OB_PROP_DEPTH_MAX_DIFF_INT              = 40, /**<soft filter maxdiff param */
    OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT      = 41, /**<soft filter maxSpeckleSize */
    OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL       = 42, /**<硬件d2c开 */
    OB_PROP_TIMESTAMP_OFFSET_INT            = 43, /**<时间戳调校 */
    OB_PROP_TOF_EXPOSURE_TIME_INT           = 47, /**<TOF曝光时间  // 仅SDK-固件内部使用 */
    OB_PROP_TOF_GAIN_INT                    = 48, /**<TOF增益 // 仅SDK-固件内部使用 */
    OB_PROP_REBOOT_DEVICE_BOOL              = 57, /**<设备重启 */
    OB_PROP_HARDWARE_DISTORTION_SWITCH_BOOL = 61, /**<硬件去畸变开关 Rectify */
    OB_PROP_FAN_WORK_MODE_INT               = 62, /**<风扇开关模式 */
    OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT   = 63, /**<多分辨率D2C模式 */
    OB_PROP_ANTI_COLLUSION_ACTIVATION_STATUS_BOOL = 64, /**<防串货状态 */
    OB_PROP_SOFTWARE_DISTORTION_SWITCH_BOOL       = 65, /**<软件去畸变开关  仅保留用于Femto SDK-固件内部使用 */
    OB_PROP_TEC_MAX_CURRENT_INT                   = 70, /**<TEC 最大电流 -> 百分比 0~100% */
    OB_PROP_TEC_MAX_CURRENT_CONFIG_INT            = 71, /**<TEC最大电流配置（掉电保存） */
    OB_PROP_FAN_WORK_MODE_CONFIG_INT              = 72, /**<风扇开关模式配置（掉电保存） */
    OB_PROP_DEPTH_PRECISION_LEVEL_INT  = 75, /**< 深度精度等级, 可能会改变深度帧数据单位, 需要通过DepthFrame的ValueScale接口确认 */
    OB_PROP_TOF_FILTER_RANGE_INT       = 76, /**< tof滤波场景范围配置 */
    OB_PROP_STOP_COLOR_STREAM_BOOL     = 77, /**< 关闭Color流, 用于无法通过标准UVC协议关流的设备 */
    OB_PROP_FACTORY_MODE_BOOL          = 78, /**< 工程模式开关 */
    OB_PROP_LASER_MODE_INT             = 79, /**< mx6600 激光点亮模式, 固件端目前只返回 1: IR Drive, 2: Torch */
    OB_PROP_RECTIFY2_BOOL              = 80, /**< mx6600 brt2r-rectify功能开关(brt2r是mx6600上的一个特殊模块), 0: Disable, 1: Rectify Enable */
    OB_PROP_COLOR_MIRROR_BOOL          = 81, /**< 彩色镜像 */
    OB_PROP_COLOR_FLIP_BOOL            = 82, /**< 彩色翻转 */
    OB_PROP_INDICATOR_LIGHT_BOOL       = 83, /**< 指示灯开关, 0: Disable,  1: Enable */
    OB_PROP_CALIBRATION_MODE_BOOL      = 84, /**< 标定模式开关, 0: Disable, 1: Enable */
    OB_PROP_DISPARITY_TO_DEPTH_BOOL    = 85, /**< 视差转深度开关,  0:关闭, 深度流输出视差图; 1. 打开, 深度流输出深度图。 */
    OB_PROP_BRT_BOOL                   = 86, /**< BRT功能开关(抗背景干扰), 0: Disable,  1: Enable */
    OB_PROP_WATCHDOG_BOOL              = 87, /**< 看门狗功能开关, 0: Disable,  1: Enable */
    OB_PROP_EXTERNAL_SIGNAL_RESET_BOOL = 88, /**< 外部信号触发重启功能开关, 0: Disable,  1: Enable */
    OB_PROP_HEARTBEAT_BOOL             = 89, /**< 心跳监测功能开关, 0: Disable,  1: Enable */
    OB_PROP_DEPTH_CROPPING_MODE_INT    = 90, /**< 深度裁剪模式设备: OB_DEPTH_CROPPING_MODE */
    OB_PROP_D2C_PREPROCESS_BOOL        = 91, /**< D2C前处理开关(如RGB裁剪) */
    OB_PROP_GPM_BOOL                   = 93, /**< GPM功能开关 */
    OB_PROP_RGB_CUSTOM_CROP_BOOL       = 94, /**< 自定义RGB裁剪开关, 0为关闭, 1为开启自定义裁剪, 下发ROI裁剪区域 */
    OB_PROP_DEVICE_WORK_MODE_INT       = 95, /**< 设备工作模式（功耗）*/
    OB_PROP_DEVICE_COMMUNICATION_TYPE_INT = 97,  /**< 设备通信方式 0: USB; 1: Ethernet(RTSP) */
    OB_PROP_SWITCH_IR_MODE_INT            = 98,  /**< 切换IR模式,0为主动IR模式,1为被动IR模式*/
    OB_PROP_LASER_POWER_LEVEL_CONTROL_INT = 99,  /**< 激光能量层级 */
    OB_PROP_LDP_MEASURE_DISTANCE_INT      = 100, /**< 获取激光近距离保护的测量值 */
    OB_PROP_LDP_CALIBRATION_BOOL          = 101, /**< 进行激光近距离保护校准 */
    OB_PROP_POWER_VOLTAGE_FLOAT           = 102, /**< 获取USB供电电压 */
    OB_PROP_LDP_STATUS_LEVEL_INT = 103, /**< 0: 激光保护关掉; 1: 激光保护打开等级1; 2: 激光保护打开等级2; 3: 激光保护打开等级3 */
    OB_PROP_TIMER_RESET_SIGNAL_BOOL             = 104, /**< 触发设备时间归零 */
    OB_PROP_TIMER_RESET_TRIGGER_OUT_ENABLE_BOOL = 105, /**< 向外发送时间归零信号开关, true:打开, false: 关闭; 默认为true */
    OB_PROP_TIMER_RESET_DELAY_US_INT            = 106, /**< 设置硬件时间归零延迟时间, 单位: 微秒 */
    OB_PROP_CAPTURE_IMAGE_SIGNAL_BOOL           = 107, /**< 软触发信号, 触发抓拍图片 */
    OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL     = 108, /**< 镜像模块状态 */
    OB_PROP_DEPTH_DUAL_MIN_DISPARITY_INT        = 109, /**< 双目深度minDisparity*/
    OB_PROP_FAN_WORK_LEVEL_INT                  = 110, /**< 风扇档位 */
    OB_PROP_PID_INT                             = 111, /**< 获取设备PID */
    OB_PROP_IR_RIGHT_MIRROR_BOOL                = 112, /**< 右IR的镜像 */
    OB_PROP_CAPTURE_IMAGE_FRAME_NUMBER_INT      = 113, /**< 单次软触发抓拍的帧数, 范围：[1, 255] */
    OB_PROP_IR_RIGHT_FLIP_BOOL                  = 114, /**< 右IR的翻转, true：翻转,false：不翻转；默认为false */
    OB_PROP_COLOR_ROTATE_INT                    = 115, /**< 彩色旋转, 翻转角度范围{0, 90, 180, 270}, 默认为0 */
    OB_PROP_IR_ROTATE_INT                       = 116, /**< IR旋转, 翻转角度范围{0, 90, 180, 270}, 默认为0 */
    OB_PROP_IR_RIGHT_ROTATE_INT                 = 117, /**< 右IR旋转, 翻转角度范围{0, 90, 180, 270}, 默认为0 */
    OB_PROP_DEPTH_ROTATE_INT                    = 118, /**< 深度旋转, 翻转角度范围{0, 90, 180, 270}, 默认为0 */
    OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT =
        119, /**< 查询激光硬件的实际能量层级, OB_PROP_LASER_POWER_LEVEL_CONTROL_INT（99）指令用于设置能级,该指令用于查询设置后硬件实际能级 */
    OB_PROP_FAN_WORK_SPEED_INT      = 120, /**< 风扇转速 */
    OB_PROP_USB_POWER_STATE_INT     = 121, /**< USB供电状态，状态值枚举: OBUSBPowerState */
    OB_PROP_DC_POWER_STATE_INT      = 122, /**< DC供电状态,状态值枚举: OBDCPowerState */
    OB_PROP_DEPTH_ENGINE_RESET_BOOL = 123, /**< 深度引擎重置 */
    OB_PROP_FAN_MAX_SPEED_TEST_MODE_BOOL = 124, /**< 设置最大风速转速（工程软件工厂使用）{true：将风扇设置成最大，false：关掉该功能} */
    OB_PROP_START_COLOR_STREAM_BOOL       = 125, /**< 启动Color流 */
    OB_PROP_START_DEPTH_STREAM_BOOL       = 126, /**< 启动Depth流 */
    OB_PROP_START_IR_STREAM_BOOL          = 127, /**< 启动IR流 */
    OB_PROP_RESET_IR_SENSOR_Y_OFFSET_BOOL = 128, /**< 重置IR Sensor Y方向offset (Astra2) */
    OB_PROP_DEVICE_DEVELOPMENT_MODE_INT   = 129, /**< 设备开发模式切换，可选模式可参考 @ref OBDeviceDevelopmentMode 中的定义 */
    OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL  = 130, /**< 多机同步触发信号外发使能，true：打开，false：关闭 */
    OB_PROP_RESTORE_FACTORY_SETTINGS_BOOL = 131, /**< 恢复出厂设置和参数，只写，参数值必须为true，重启设备后生效 */
    OB_PROP_BOOT_INTO_RECOVERY_MODE_BOOL  = 132, /**< 启动设备时进入恢复模式（刷机模式）, 在该模式下可对设备系统进行升级 */
    OB_PROP_DEVICE_IN_RECOVERY_MODE_BOOL  = 133, /**< 获取当前设备是否运行在恢复模式（刷机模式） */

    /**
     * Capture interval mode, 0:time interval, 1:number interval
     */
    OB_PROP_CAPTURE_INTERVAL_MODE_INT = 134,

    OB_PROP_CAPTURE_IMAGE_TIME_INTERVAL_INT   = 135, /**< \if English Capture time interval \else 抓拍时间间隔 \endif */
    OB_PROP_CAPTURE_IMAGE_NUMBER_INTERVAL_INT = 136, /**< \if English Capture number interval \else 抓拍数量间隔 \endif */
    OB_PROP_STOP_IR_RIGHT_STREAM_BOOL         = 139, /**< 关闭右IR流, 用于无法通过标准UVC协议关流的设备 */
    OB_PROP_TIMER_RESET_ENABLE_BOOL           = 140, /**< \if English Enable timer reset function \else 时间戳清零功能使能 \endif */

    /**
     * @brief 设置设备端USB2.0重新识别的使能开关{true:识别成usb2.0时重试识别，最多重试3次，false:识别成usb2.0时不重试识别}
     */
    OB_PROP_DEVICE_USB2_REPEAT_IDENTIFY_BOOL = 141,
    OB_PROP_DEVICE_REBOOT_DELAY_INT = 142, /**< 控制设备重启，带延迟模式；类型：uint32_t，延迟时间单位：ms。delay为0：不延迟；delay大于0，延迟delay毫秒*/
    OB_PROP_DEVICE_IQ_DEBUG_BOOL                     = 143, /** IQ 调试功能，用于影像调试ISP参数*/
    OB_PROP_USB_STATUS_CHECK_BOOL                    = 144, /**< USB状态检查，true：正常状态，false：异常状态 */
    OB_PROP_OBOX_STATUS_CHECK_BOOL                   = 145, /**< OBox空闲模式检查，true：空闲状态，false：非空闲状态 */
    OB_PROP_LASER_OVERCURRENT_PROTECTION_STATUS_BOOL = 148, /**< 获取激光电流过流保护后的状态 */
    OB_PROP_LASER_PULSE_WIDTH_PROTECTION_STATUS_BOOL = 149, /**< 获取激光脉宽保护的状态 */
    OB_PROP_UPDATE_BASE_TIME_BOOL                    = 150, /**< 更新高32位时间戳 */
    OB_PROP_DEPTH_NOISE_REMOVAL_FILTER_BOOL          = 165, /**< 使能开关，深度去噪 */
    OB_PROP_DEPTH_EDGE_NOISE_REMOVAL_FILTER_BOOL     = 166, /**< 使能开关，深度边缘去噪 */
    OB_PROP_DEPTH_SPATIAL_FAST_FILTER_BOOL           = 167, /**< 使能开关，深度快速空间滤波 */
    OB_PROP_DEPTH_SPATIAL_MODERATE_FILTER_BOOL       = 168, /**< 使能开关，深度适中空间滤波 */
    OB_PROP_DEPTH_SPATIAL_ADVANCED_FILTER_BOOL       = 169, /**< 使能开关，深度增强空间滤波 */
    OB_PROP_DEPTH_HOLE_FILLING_FILTER_BOOL           = 170, /**< 使能开关，深度填洞滤波 */
    OB_PROP_DEPTH_TEMPORAL_FILTER_BOOL               = 171, /**< 使能开关，深度时域滤波 */
    OB_PROP_LASER_ALWAYS_ON_BOOL                     = 174, /**< 激光常开开关，true：打开，false：关闭 */
    OB_PROP_LASER_ON_OFF_PATTERN_INT                 = 175, /**< 激光随数据帧开关交错，0:关闭，1:打开，ON-OFF 2:打开：OFF-ON */
    OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT     = 176, /**< 深度单位灵活调整，可在范围内连续调节，单位：mm */
    OB_PROP_LASER_CONTROL_INT                        = 182, /**< 激光控制，0: 关闭激光，1: 打开激光，2: 自动 */
    OB_PROP_IR_BRIGHTNESS_INT                        = 184, /**< IR 亮度 */
    OB_PROP_DEPTH_ALG_MODE_TYPE_INT                  = 185, /**< 深度工作模式类型 */
    OB_PROP_SLAVE_DEVICE_SYNC_STATUS_BOOL            = 188, /** <从设备同步状态 */
    OB_PROP_COLOR_AE_MAX_EXPOSURE_INT                = 189, /**< COLOR AE最大曝光时间 */
    OB_PROP_IR_AE_MAX_EXPOSURE_INT                   = 190, /**< IR AE最大曝光时间 */
    OB_PROP_DISP_SEARCH_RANGE_MODE_INT               = 191, /** <视差搜索范围模式，1: 128,2:256 */

    // 1000~1999为设备端结构体控制命令
    // Device:: get/setStructuredData
    OB_STRUCT_VERSION                           = 1000, /**< 版本信息 */
    OB_STRUCT_INTERNAL_CAMERA_PARAM             = 1001, /**< 相机内外参数 */
    OB_STRUCT_BASELINE_CALIBRATION_PARAM        = 1002, /**< 基线标定参数 */
    OB_STRUCT_DEVICE_TEMPERATURE                = 1003, /**< 设备温度信息 */
    OB_STRUCT_IR_AE_PARAM                       = 1004, /**< IR-AE调试参数 */
    OB_STRUCT_EXTENSION_PARAM                   = 1005, /**< 扩展参数 */
    OB_STRUCT_DEVICE_UPGRADE_STATUS             = 1006, /**< 固件升级状态 read only */
    OB_STRUCT_DEVICE_FILE_TRAN_STATUS           = 1008, /**< 文件传输状态 read only */
    OB_STRUCT_TOF_MODULATION_FREQ               = 1012, /**< TOF调制频率信息 */
    OB_STRUCT_TOF_DUTY_CYCLE                    = 1013, /**< TOF调制信号占空比信息 */
    OB_STRUCT_TOF_EXPOSURE_THRESHOLD_CONTROL    = 1024, /**< TOF曝光阈值范围 */
    OB_STRUCT_DEVICE_STATE                      = 1025, /**< 获取当前设备状态 */
    OB_STRUCT_TEC_DATA                          = 1026, /**< 获取TEC数据 */
    OB_STRUCT_ANTI_COLLUSION_ACTIVATION_CONTENT = 1029, /**< 防串货激活码读写 */
    OB_STRUCT_ANTI_COLLUSION_ACTIVATION_VERIFY  = 1030, /**< 防串货激活码验证 */
    OB_STRUCT_GET_GYRO_PRESETS_ODR_LIST         = 1031, /**< 获取陀螺仪支持的采样率列表 */
    OB_STRUCT_GET_ACCEL_PRESETS_ODR_LIST        = 1032, /**< 获取加速度计支持的采样率列表 */
    OB_STRUCT_GET_GYRO_PRESETS_FULL_SCALE_LIST  = 1033, /**< 获取陀螺仪支持的量程列表 */
    OB_STRUCT_GET_ACCEL_PRESETS_FULL_SCALE_LIST = 1034, /**< 获取加速度计支持的量程列表 */
    OB_STRUCT_DEVICE_SERIAL_NUMBER              = 1035, /**< get/set序列号 */
    OB_STRUCT_DEVICE_TIME                       = 1037, /**< get/set device time */
    OB_STRUCT_MULTI_DEVICE_SYNC_CONFIG          = 1038, /**< 多设备同步模式和参数配置 */
    OB_STRUCT_TEMP_COMPENSATE_PARAM             = 1039, /**< get/set 温度补偿系数 */
    OB_STRUCT_RGB_CROP_ROI                      = 1040, /**< RGB裁剪ROI */
    OB_STRUCT_DEVICE_IP_ADDR_CONFIG             = 1041, /**< 设备ip地址配置 */
    OB_STRUCT_PERIPHERAL_ID_INFO                = 1042, /**< 外围器件ID信息 */
    OB_STRUCT_CURRENT_DEPTH_ALG_MODE            = 1043, /**< 当前的相机深度模式 */
    OB_STRUCT_LED_CONTROL                       = 1044, /**< FemtoMega Led控制*/
    OB_STRUCT_DEPTH_PRECISION_SUPPORT_LIST      = 1045, /**< 深度精度等级列表,返回uin16_t数组,对应精度等级的枚举定义 */
    OB_PROP_DEPTH_RECTIFY_MASK_FILTER_CONFIG    = 1046, /**< 深度标定mask配置 */
    OB_STRUCT_COLOR_STREAM_PROFILE              = 1048,
    OB_STRUCT_DEPTH_STREAM_PROFILE              = 1049,
    OB_STRUCT_IR_STREAM_PROFILE                 = 1050,
    OB_STRUCT_CUSTOMER_DATA                     = 1052, /**< 读写第三方客户自定义数据 */
    OB_STRUCT_DEVICE_STATIC_IP_CONFIG_RECORD    = 1053, /**< 设备网络静态IP配置记录获取，只读命令 */

    /**
     * @brief LDP测量扩展信息，供应商TOF器件输出的测量值。注意：Gemini2/Gemini2L/Gemini2VL/Gemini2XL旧版固件不支持该指令，
     * 新版本固件才支持，请注意try-catch。SDK支持指令日期：2023年10月16日。
     *
     */
    OB_STRUCT_LDP_MEASURE_EXTENSION_INFO = 1055,
    OB_STRUCT_DEPTH_DDO_CONFIG           = 1057, /** 配置参数，DDOConfig数据结构 */
    OB_STRUCT_DEPTH_DDO_CONFIG_DEFAULT   = 1058, /** 配置参数，DDOConfig数据结构，默认参数 */
    /**
     * @brief Using to configure the depth sensor's HDR mode
     * @brief The Value type is @ref OBHdrConfig
     *
     * @attention After enable HDR mode, the depth sensor auto exposure will be disabled.
     */
    OB_STRUCT_DEPTH_HDR_CONFIG = 1059,

    /**
     * @brief Color Sensor AE ROI configuration
     * @brief The Value type is @ref OBRegionOfInterest
     */
    OB_STRUCT_COLOR_AE_ROI = 1060,

    /**
     * @brief Depth Sensor AE ROI configuration
     * @brief The Value type is @ref OBRegionOfInterest
     * @brief Since the ir sensor is the same physical sensor as the depth sensor, this property will also effect the ir sensor.
     */
    OB_STRUCT_DEPTH_AE_ROI = 1061,

    /**
     * @brief  ASIC Serial Number
     *
     */
    OB_STRUCT_ASIC_SERIAL_NUMBER = 1063,

    // 2000~2999为Sensor控制命令
    // device::get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型)
    OB_PROP_COLOR_AUTO_EXPOSURE_BOOL         = 2000, /**< 彩色相机自动曝光 */
    OB_PROP_COLOR_EXPOSURE_INT               = 2001, /**< 彩色相机曝光调节 */
    OB_PROP_COLOR_GAIN_INT                   = 2002, /**< 彩色相机增益调节 */
    OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL    = 2003, /**< 彩色相机自动白平衡 */
    OB_PROP_COLOR_WHITE_BALANCE_INT          = 2004, /**< 彩色相机白平衡调节 */
    OB_PROP_COLOR_BRIGHTNESS_INT             = 2005, /**< 彩色相机亮度调节 */
    OB_PROP_COLOR_SHARPNESS_INT              = 2006, /**< 彩色相机锐度调节 */
    OB_PROP_COLOR_SHUTTER_INT                = 2007, /**< 彩色相机快门值调节 */
    OB_PROP_COLOR_SATURATION_INT             = 2008, /**< 彩色相机饱和度调节 */
    OB_PROP_COLOR_CONTRAST_INT               = 2009, /**< 彩色相机对比度调节 */
    OB_PROP_COLOR_GAMMA_INT                  = 2010, /**< 彩色相机伽马值调节 */
    OB_PROP_COLOR_ROLL_INT                   = 2011, /**< 彩色相机图像旋转 */
    OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT = 2012, /**< 彩色相机自动曝光优先 */
    OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT = 2013, /**< 彩色相机亮度补偿 */
    OB_PROP_COLOR_HUE_INT                    = 2014, /**< 彩色相机彩色色调 */
    OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT   = 2015, /**< 彩色相机电力线路频率 */
    OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL         = 2016, /**< 深度相机自动曝光（某些型号设备下会同步设置红外相机） */
    OB_PROP_DEPTH_EXPOSURE_INT               = 2017, /**< 深度相机曝光调节（某些型号设备下会同步设置红外相机） */
    OB_PROP_DEPTH_GAIN_INT                   = 2018, /**< 深度相机增益调节（某些型号设备下会同步设置红外相机） */
    OB_PROP_GYRO_SWITCH_BOOL                 = 2019, /**< 陀螺仪开关 */
    OB_PROP_ACCEL_SWITCH_BOOL                = 2020, /**< 加速度计开关 */
    OB_PROP_GYRO_ODR_INT                     = 2021, /**< get/set当前陀螺仪的采样率 */
    OB_PROP_ACCEL_ODR_INT                    = 2022, /**< get/set当前加速度计的采样率 */
    OB_PROP_GYRO_FULL_SCALE_INT              = 2023, /**< get/set当前陀螺仪的量程 */
    OB_PROP_ACCEL_FULL_SCALE_INT             = 2024, /**< get/set当前加速度计的量程 */
    OB_PROP_IR_AUTO_EXPOSURE_BOOL            = 2025, /**< 红外相机自动曝光（某些型号设备下会同步设置深度相机） */
    OB_PROP_IR_EXPOSURE_INT                  = 2026, /**< 红外相机曝光调节（某些型号设备下会同步设置深度相机） */
    OB_PROP_IR_GAIN_INT                      = 2027, /**< 红外相机增益调节（某些型号设备下会同步设置深度相机） */
    OB_PROP_IR_CHANNEL_DATA_SOURCE_INT = 2028, /**< 读写IR通道的输出目标sensor,不支持时返回错误。0: 左侧IR  sensor,1: 右侧IR sensor; */
    OB_PROP_DEPTH_RM_FILTER_BOOL =
        2029, /** RM是Remark Filter的缩写, 深度mask对齐参数开关, true：打开，false：关闭, 与D2C功能互斥。软硬件D2C开启时，不能使用mask功能 */
    OB_PROP_COLOR_MAXIMAL_GAIN_INT    = 2030, /**< 彩色相机最大增益 */
    OB_PROP_COLOR_MAXIMAL_SHUTTER_INT = 2031, /**< 彩色相机最大快门 */
    OB_PROP_IR_SHORT_EXPOSURE_BOOL =
        2032, /**< 红外相机一帧AE一帧短曝光模式的使能开关（DCL用），当开启时AE和短曝光数据交错输出，关闭时恢复原红外相机的曝光模式（AE或者固定曝光） */
    OB_PROP_IR_SHORT_EXPOSURE_INT = 2033, /**< 红外相机短曝光调节（DCL用），当开启短曝光模式后，用来设置红外相机短曝光时间 */

    OB_PROP_COLOR_HDR_BOOL        = 2034, /**< 彩色相机HDR开关调节 */
    OB_PROP_IR_LONG_EXPOSURE_BOOL = 2035, /**< 红外相机长曝光开关，DaBai Max Pro支持） */
    OB_PROP_SKIP_FRAME_BOOL       = 2036, /**< 相机跳帧模式开关，DaBai DCW2支持） */
    OB_PROP_HDR_MERGE_BOOL        = 2037, /**< 深度HDR merge处理使能开关，true打开，false关闭） */
    OB_PROP_COLOR_FOCUS_INT       = 2038, /**< 彩色相机焦点调节，针对Astra+设备默认1，支持用户可设置0关闭宽动态功能） */

    /** 3000~3499为SDK int, bool及float类型控制命令 */
    /** Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型) */
    OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL       = 3004, /**< 视差转深度 */
    OB_PROP_SDK_DEPTH_FRAME_UNPACK_BOOL       = 3007, /**< Depth数据解包功能开关(每次开流都会默认打开,支持RLE/Y10/Y11/Y12/Y14格式) */
    OB_PROP_SDK_IR_FRAME_UNPACK_BOOL          = 3008, /**< Ir数据解包功能开关(每次开流都会默认打开,支持RLE/Y10/Y11/Y12/Y14格式) */
    OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL  = 3009, /**< Accel数据转换功能开关(默认打开) */
    OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL   = 3010, /**< Gyro数据转换功能开关(默认打开) */
    OB_PROP_SDK_IR_LEFT_FRAME_UNPACK_BOOL     = 3011, /**< [左]Ir数据解包功能开关(每次开流都会默认打开，支持RLE/Y10/Y11/Y12/Y14格式) */
    OB_PROP_SDK_IR_RIGHT_FRAME_UNPACK_BOOL    = 3012, /**< [右]Ir数据解包功能开关(每次开流都会默认打开，支持RLE/Y10/Y11/Y12/Y14格式) */
    OB_PROP_SDK_DEPTH_RECTIFY_MG_FILTER_BOOL  = 3013, /**< MG滤波开关 */
    OB_PROP_SDK_DEPTH_RECTIFY_MG_FILTER_X_INT = 3014, /**< MG滤波水平阈值 */
    OB_PROP_SDK_DEPTH_RECTIFY_MG_FILTER_Y_INT = 3015, /**< MG滤波纵向阈值 */

    OB_PROP_SDK_DEPTH_RECTIFY_MG_FILTER_MODE_BOOL = 3016, /**< MG滤波方向控制，true为纵和横向，false为仅横向 */
    OB_PROP_SDK_UPDATE_BASE_TIME_BOOL             = 3017, /**< 更新时间戳高32位 */

    // 4000~4999为RawData控制命令
    // Device:: get/setRawData
    OB_RAW_DATA_MULTIPLE_DISTANCE_CALIBRATION_PARAM = 4000, /**< 多距离标定参数 */
    OB_RAW_DATA_REFERENCE_IMAGE                     = 4001, /**< 参考图 */
    OB_RAW_DATA_HARDWARE_ALIGN_PARAM                = 4005, /**< 硬件对齐参数 */
    OB_RAW_DATA_SOFTWARE_ALIGN_PARAM                = 4006, /**< 软件对齐参数 */
    OB_RAW_DATA_HARDWARE_DISTORTION_PARAM           = 4007, /**< 硬件畸变参数 */
    OB_RAW_DATA_HARDWARE_ALIGN_PARAM_0              = 4009, /**< 硬件对齐参数0*/
    OB_RAW_DATA_HARDWARE_ALIGN_PARAM_1              = 4010, /**< 硬件对齐参数1 */
    OB_RAW_DATA_TEMP_COMPENSATE_PARAM               = 4015, /**< 温补参数 */
    OB_RAW_DATA_SOFTWARE_ALIGN_PARAM_0              = 4016, /**< 软件对齐参数0,*/
    OB_RAW_DATA_SOFTWARE_ALIGN_PARAM_1              = 4017, /**< 软件对齐参数1 */
    OB_RAW_DATA_DEPTH_CALIB_FLASH_FILE              = 4022, /**< 深度标定参数文件(MX6600) */
    OB_RAW_DATA_ALIGN_CALIB_FLASH_FILE              = 4023, /**< 对齐标定参数文件(MX6600) */
    OB_RAW_DATA_D2C_ALIGN_SUPPORT_PROFILE_LIST      = 4024, /**< D2C对齐分辨率列表 */
    OB_RAW_DATA_D2C_ALIGN_POST_PROCESS_LIST         = 4025, /**< D2C后处理参数列表 */
    OB_RAW_DATA_DEPTH_CALIB_PARAM                   = 4026, /**< 深度标定参数(MX6600, read only) */
    OB_RAW_DATA_ALIGN_CALIB_PARAM                   = 4027, /**< 对齐标定参数(MX6600, read only) */
    OB_RAW_DATA_DISPARITY_TO_DEPTH_PROFILE_LIST     = 4028, /**< D2D 分辨率-参数配置列表(MX6600, read only)*/
    OB_RAW_DATA_CAMERA_CALIB_JSON_FILE              = 4029, /**< 从设备端读取的标定Json文件(Femto Mega, read only)*/
    OB_RAW_DATA_D2C_ROT_LUT_FILE                    = 4030, /**< D2C初始化表文件(Femto Mega)*/
    OB_RAW_DATA_MCU_UPGRADE_FILE                    = 4031, /**< MCU升级文件(Femto Mega)*/
    OB_RAW_DATA_DEFAULT_STREAM_PROFILE_LIST         = 4032, /**< 默认流配置列表获取 */
    OB_RAW_DATA_STREAM_PROFILE_LIST                 = 4033, /**< 流配置列表获取(暂时应用在Femto Mega的网络流配置获取)*/
    OB_RAW_DATA_DEPTH_ALG_MODE_LIST                 = 4034, /**< 查询深度模式列表 */
    OB_RAW_DATA_EFFECTIVE_VIDEO_STREAM_PROFILE_LIST = 4035, /**< 查询当前深度模式下的分辨率列表 */
    OB_RAW_DATA_IMU_CALIB_PARAM                     = 4036, /**< IMU标定参数文件 */
    OB_RAW_DATA_DE_IR_RECTIFY_PARAMS = 4037, /**< 获取深度引擎极线校正参数, 用于生成深度极线校正mask的原始参数; DE是depth engine的缩写 */
    OB_RAW_DATA_DEVICE_LOG_BAG               = 4038, /**< 设备log读取*/
    OB_RAW_DATA_DEVICE_UPGRADE               = 4039, /**< 固件升级 */
    OB_RAW_DATA_DEVICE_EXTENSION_INFORMATION = 4041, /**< 设备扩展信息 */
    OB_RAW_DATA_DEVICE_MAC                   = 4042, /**< 设备mac信息 */
    OB_RAW_DATA_NET_PORT_INFO_LIST           = 4043, /**< 网络端口信息列表 */
    OB_RAW_DATA_IR_AE_PARAMS                 = 4044, /**< IR-AE参数扩展 rawdata传输 */

    // 4500~4599为RawData控制命令的子集，专门提供给MX6600用raw data烧写算法参数文件的控制命令
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_REF_PARAM = 4500, /**< 文件说明：参考图buff数据；文件/数据类型：Depth_Calibration_Flash_File */
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_RECTIFY_KK_PARAM =
        4501, /**< 文件说明：Recify、d2c、d2d寄存器及浮点参数，IMU标定参数；文件/数据类型：Align_Calibration_Flash_File */
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_IR_RGB_PRE_PARAM = 4502, /**< 文件说明：IR&RGB动态裁剪及其他标定参数；文件/数据类型：IR_RGB_Pre_Flash_File */
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_RECTIFY_LUT_PARAM = 4503, /**< 文件说明：存储查找表buff的内容；文件/数据类型：LUT_Data_Flash_File */
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_D2D_PARAM = 4504, /**< 文件说明：一般是优化后的D2D参数；文件/数据类型：D2D_Params_Flash_File */
    OB_RAW_DATA_FLASH_DATA_TYPE_SECTION_IMU_PARAM = 4505, /**< 文件说明：Imu标定参数；文件/数据类型：IMU_Params_Flash_File */
    OB_RAW_DATA_UPDATE_MX6600_FLASH_DATA =
        4506, /**< 文件说明：MX6600算法参数文件，一个文件包含包含一个或者多个算法参数类型数据，SDK端自动解析算法参数内容并更新到对应的flash分区。 */

    /** 5000~5499为调试用int, bool及float类型控制命令 */
    /** Device:: get/setxxxProperty、 getxxxPropertyRange (xxx表示数据类型) */
    OB_PROP_DEBUG_ADB_FUNCTION_CONTROL_BOOL = 5000, /**< ADB调试功能开关 */
    OB_PROP_DEBUG_SET_FORCE_UPGRADE_BOOL    = 5001, /**< 强制升级 */
    OB_PROP_DEBUG_LOG_SEVERITY_LEVEL_INT    = 5003, /**< 设备log等级 */

    // 5500~5999为调试用结构体控制命令
    // Device:: get/setStructuredData
    OB_STRUCT_DEBUG_RECORD_RGB_DATA                  = 5500, /**< 设备端 RGB 传图控制（调试功能） */
    OB_STRUCT_DEBUG_RECORD_PHASE_DATA                = 5501, /**< 设备端 raw Phase 传图控制（调试功能） */
    OB_STRUCT_DEBUG_RECORD_IR_DATA                   = 5502, /**< 设备端 IR 传图控制（调试功能） */
    OB_STRUCT_DEBUG_RECORD_DEPTH_DATA                = 5503, /**< 设备端 depth 传图控制（调试功能） */
    OB_STRUCT_DEBUG_SENSOR_EXPOSURE_TIME             = 5504, /**< 曝光时间读写 */
    OB_PROP_DEBUG_DEVICELOG_SAVE_BOOL                = 5505, /**< 设备log保存 */
    OB_STRUCT_DEPTH_NOISE_REMOVAL_FILTER_PARAMS      = 5516, /**< 配置参数，深度去噪 */
    OB_STRUCT_DEPTH_EDGE_NOISE_REMOVAL_FILTER_PARAMS = 5517, /**< 配置参数，深度边缘去噪 */
    OB_STRUCT_DEPTH_SPATIAL_FAST_FILTER_PARAMS       = 5518, /**< 配置参数，深度快速空间滤波 */
    OB_STRUCT_DEPTH_SPATIAL_MODERATE_FILTER_PARAMS   = 5519, /**< 配置参数，深度适中空间滤波 */
    OB_STRUCT_DEPTH_SPATIAL_ADVANCED_FILTER_PARAMS   = 5520, /**< 配置参数，深度增强空间滤波 */
    OB_STRUCT_DEPTH_HOLE_FILLING_FILTER_PARAMS       = 5521, /**< 配置参数，深度填洞滤波 */
    OB_STRUCT_DEPTH_TEMPORAL_FILTER_PARAMS           = 5522, /**< 配置参数，深度时域滤波 */
    OB_PROP_DEPTH_FILTER_DUMP_DIFF_LUT_INT           = 5523, /**< 库参数调试，深度滤波库的调调试；*/

} OBPropertyID,
    ob_property_id;

/**
 * @brief 用于描述所有属性设置的数据类型
 */
typedef enum OBPropertyType {
    OB_BOOL_PROPERTY     = 0, /**< bool类型数据的属性 */
    OB_INT_PROPERTY      = 1, /**< int类型数据的属性 */
    OB_FLOAT_PROPERTY    = 2, /**< float类型数据的属性 */
    OB_STRUCT_PROPERTY   = 3, /**< struct类型数据的属性 */
    OB_RAW_DATA_PROPERTY = 4, /**< raw data数据的属性 */
} OBPropertyType,
    ob_property_type;

/**
 * @brief 用于描述每一个属性的特性
 */
typedef struct OBPropertyItem {
    OBPropertyID     id;          ///< 属性id
    const char      *name;        ///< 属性名字
    OBPropertyType   type;        ///< 属性类型
    OBPermissionType permission;  ///< 属性读写权限
} OBPropertyItem, ob_property_item;

#ifdef __cplusplus
}
#endif

#endif  // _OB_PROPERTY_H_
#endif  // OB_SENSOR_SDK_DEVELOPER