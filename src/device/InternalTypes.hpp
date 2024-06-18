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

#pragma pack(pop)