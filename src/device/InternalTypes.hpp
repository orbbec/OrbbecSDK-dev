#pragma once

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)

/**
 *@brief device version information
 */
typedef struct {
    char    firmwareVersion[16];   ///< Such as: 1.2.18
    char    hardwareVersion[16];   ///< Such as: 1.0.18
    char    sdkVersion[16];        ///< The lowest supported SDK version number, SDK version number: 2.3.2 (major.minor.revision)
    char    depthChip[16];         ///< Such as：mx6000, mx6600
    char    systemChip[16];        ///< Such as：ar9201
    char    serialNumber[16];      ///< serial number
    int32_t deviceType;            ///< 1:Monocular 2:binocular 3:tof. enumeration value
    char    deviceName[16];        ///< device name，such as: astra+
    char    subSystemVersion[16];  ///< For example，such as：Femto’s MCU firmware version 1.0.23
    char    reserved[32];          ///< Reserved
} OBVersionInfo;

/**
 *@brief device time
 */
typedef struct {
    uint64_t time;  ///< sdk->dev: timing time; dev->sdk: current time of device;
    uint64_t rtt;   ///< sdk->dev: command round-trip time, the device sets the time to time+rtt/2 after receiving it; dev->sdk: reserved; unit: ms
} DEVICE_TIME, OBDeviceTime, ob_device_time;

/**
 * @brief Post-process parameters after depth align to color
 *
 */
typedef struct {
    float   depthScale;   // Depth frame scaling ratio
    int16_t alignLeft;    // Depth aligned to left after Color
    int16_t alignTop;     // Depth aligned to the top after Color
    int16_t alignRight;   // Depth aligned to right after Color
    int16_t alignBottom;  // Depth aligned to the bottom after Color
} OBD2CPostProcessParam, ob_d2c_post_process_param;

typedef enum {
    ALIGN_UNSUPPORTED    = 0,
    ALIGN_D2C_HW         = 1,
    ALIGN_D2C_SW         = 2,
    ALIGN_D2C_HW_SW_BOTH = 3,
} OBAlignSupportedType,
    ob_align_supported_type;

/**
 *@brief Supported depth align color profile
 *
 */
typedef struct {
    uint16_t              colorWidth;
    uint16_t              colorHeight;
    uint16_t              depthWidth;
    uint16_t              depthHeight;
    uint8_t               alignType;
    uint8_t               paramIndex;
    OBD2CPostProcessParam postProcessParam;
} OBD2CProfile, ob_d2c_supported_profile_info;

typedef struct {
    uint32_t depthMode;    ///< 单目/双目
    float    baseline;     ///< 基线距离
    float    z0;           ///< 标定距离
    float    focalPix;     ///< 用于深度计算的焦距或rectify后的焦距f
    float    unit;         ///< 单位 x1 mm，如：unit=0.25, 表示0.25*1mm=0.25mm
    float    dispOffset;   ///< 视差偏移，真实视差=芯片输出的视差 + disp_offset
    int32_t  invalidDisp;  ///< 无效视差，一般情况下为0；当芯片min_disp 不等于0或者-128时，无效视差不再等于0
} OBDepthCalibrationParam;

#pragma pack(pop)