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

/* Scale Factors */
/* Accel (10g Range): 2^19 = 524288.0 */
#define ACCEL_SCALE_FACTOR  524288.0f 

/* Gyro (450 deg/s Range): 2^19 = 524288.0 (Assumed same as Accel based on "similar method") 
   Note: If Gyro range is different, this macro needs to be updated. 
   Common MEMS Gyro scales: 
   - 250 dps: 131 LSB/dps
   - 500 dps: 65.5 LSB/dps
   - or similar 2^N divisors. 
   Currently applying 2^19 as requested ("similar method").
*/
#define GYRO_SCALE_FACTOR   524288.0f 

/*==============================================================================
 * Type Definition
 *============================================================================*/
typedef struct {
    float fGyroX; // degree/s
    float fGyroY;
    float fGyroZ;
    float fAccX;  // g
    float fAccY;
    float fAccZ;
    UInt8 ucCounter;
} ImuData_t;

/* GPS Data Structure */
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
} GpsData_t;



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
/* Updated Signature to match usage in ignu_task.c */
SInt32 ParseGpsPacket(UInt8 *pData, GpsData_t *pOutput); 
void SetGpsData(GpsData_t *pData);
void GetGpsData(GpsData_t *pData);
void PrintDouble(double val);

#endif /* __INS_GPS_H__ */
