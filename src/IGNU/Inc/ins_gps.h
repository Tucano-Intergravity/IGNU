/**
 * @file ins_gps.h
 * @brief Inertial Navigation System & GPS Data Processing Header
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @date 2026-01-09
 */

#ifndef __INS_GPS_H__
#define __INS_GPS_H__

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "FreeRTOS.h"
#include "../../common/common.h"

/*==============================================================================
 * Define
 *============================================================================*/
/* IMU Packet Definitions */
#define IMU_PACKET_SIZE     42
#define IMU_SYNC_BYTE       0xA5

/* GPS Packet Definitions */
#define GPS_RAW_PACKET_SIZE 91
#define GPS_SYNC_WORD       0x2440

/* Scale Factors */
#define ACCEL_SCALE_FACTOR  524288.0f 
#define GYRO_SCALE_FACTOR   524288.0f 

/*==============================================================================
 * Type Definition
 *============================================================================*/

/* 
 * 1. Raw GPS Data Structure (Packed, 91 Bytes)
 * Used only for layout reference or safe copying, NOT for direct access.
 */
#pragma pack(push, 1)
typedef struct {
    UInt16 syncWord;    // Offset 0 (0x2440, Big Endian?)
    UInt32 tow;         // Offset 2
    UInt16 wnc;         // Offset 6
    UInt8  mode;        // Offset 8
    UInt8  error;       // Offset 9
    double latitude;    // Offset 10 (Unaligned!)
    double longitude;   // Offset 18
    double height;      // Offset 26
    float  undulation;  // Offset 34
    float  vn;          // Offset 38
    float  ve;          // Offset 42
    float  vu;          // Offset 46
    float  gog;         // Offset 50
    double rxClkBias;   // Offset 54
    float  rxClkDrift;  // Offset 62
    UInt8  timeSystem;  // Offset 66
    UInt8  datum;       // Offset 67
    UInt8  nrSv;        // Offset 68
    UInt8  waCorrInfo;  // Offset 69
    UInt16 referenceId; // Offset 70
    UInt16 meanCorrAge; // Offset 72
    UInt32 signalInfo;  // Offset 74
    UInt8  alertFlag;   // Offset 78
    UInt8  nrBases;     // Offset 79
    UInt16 pppInfo;     // Offset 80
    UInt16 latency;     // Offset 82
    UInt16 hAccuracy;   // Offset 84
    UInt16 vAccuracy;   // Offset 86
    UInt8  misc;        // Offset 88
    UInt8  reserved;    // Offset 89
    UInt8  sendingCnt;  // Offset 90
} GpsRawData_t;
#pragma pack(pop)

/*
 * 2. Aligned GPS Data Structure
 * Safe for application usage.
 */
typedef struct {
    UInt32 tow;         // Time of Week (ms)
    UInt16 wnc;         // Week Number Count
    double latitude;    // deg
    double longitude;   // deg
    double height;      // m
    float  vn;          // Velocity North (m/s)
    float  ve;          // Velocity East (m/s)
    float  vu;          // Velocity Up (m/s)
    UInt8  mode;        // Position Mode
    UInt8  error;       // Error status
    UInt8  nrSv;        // Number of SVs
    
    /* Additional Fields from Custom Packet */
    float  undulation;
    float  gog;
    double rxClkBias;
    float  rxClkDrift;
    UInt16 hAccuracy;
    UInt16 vAccuracy;
} GpsData_t;


typedef struct {
    float fGyroX; // degree/s
    float fGyroY;
    float fGyroZ;
    float fAccX;  // g
    float fAccY;
    float fAccZ;
    UInt8 ucCounter;
} ImuData_t;

/*==============================================================================
 * Global Function Declarations
 *============================================================================*/
/* IMU Functions */
void ProcessImuPacket(UInt8 *pRawData, ImuData_t *pOutput);
float ConvertRaw24(UInt8 *pRaw, float fScale);
void PrintFloat(float val);
void SetImuData(ImuData_t *pData);
void GetImuData(ImuData_t *pData);

/* GPS Functions */
/* 
 * Safe Parsing Function:
 * Uses memcpy to prevent Hard Faults from unaligned access.
 */
SInt32 ParseGpsPacket(const UInt8 *pRawData, GpsData_t *pOutput);
void SetGpsData(GpsData_t *pData);
void GetGpsData(GpsData_t *pData);
void PrintDouble(double val);

#endif /* __INS_GPS_H__ */
