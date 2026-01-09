/**
 * @file ins_gps.c
 * @brief Inertial Navigation System & GPS Data Processing Source
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @date 2026-01-09
 */

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "../Inc/ins_gps.h"
#include "xil_printf.h"

/*==============================================================================
 * Local Variables
 *============================================================================*/
static ImuData_t stGlobalImuData;

/*==============================================================================
 * Functions
 *============================================================================*/

void SetImuData(ImuData_t *pData)
{
    if (pData) {
        memcpy(&stGlobalImuData, pData, sizeof(ImuData_t));
    }
}

void GetImuData(ImuData_t *pData)
{
    if (pData) {
        memcpy(pData, &stGlobalImuData, sizeof(ImuData_t));
    }
}

/**
 * @brief Convert 3-byte Big Endian 24-bit Signed Integer to Float
 */
float ConvertRaw24(UInt8 *pRaw, float fScale)
{
    /* 1. Combine 3 Bytes (Big Endian) into 32-bit Integer */
    /* pRaw[0]: MSB, pRaw[2]: LSB */
    SInt32 raw24 = (pRaw[0] << 16) | (pRaw[1] << 8) | pRaw[2];
    
    /* 2. Sign Extension: 24-bit to 32-bit */
    if (raw24 & 0x800000) {
        raw24 |= 0xFF000000;
    }

    /* 3. Apply Scale Factor */
    return (float)raw24 / fScale;
}

/**
 * @brief Process Raw IMU Packet (42 Bytes)
 */
void ProcessImuPacket(UInt8 *pRawData, ImuData_t *pOutput)
{
    if (pRawData == NULL || pOutput == NULL) return;

    /* Verify Sync Byte */
    if (pRawData[0] != IMU_SYNC_BYTE) return;

    /* Gyro Parsing (Offset 1, 4, 7) */
    pOutput->fGyroX = ConvertRaw24(&pRawData[1], GYRO_SCALE_FACTOR);
    pOutput->fGyroY = ConvertRaw24(&pRawData[4], GYRO_SCALE_FACTOR);
    pOutput->fGyroZ = ConvertRaw24(&pRawData[7], GYRO_SCALE_FACTOR);

    /* Accel Parsing (Offset 11, 14, 17) */
    pOutput->fAccX = ConvertRaw24(&pRawData[11], ACCEL_SCALE_FACTOR);
    pOutput->fAccY = ConvertRaw24(&pRawData[14], ACCEL_SCALE_FACTOR);
    pOutput->fAccZ = ConvertRaw24(&pRawData[17], ACCEL_SCALE_FACTOR);

    /* Counter (Offset 35) */
    pOutput->ucCounter = pRawData[35];
}

/**
 * @brief Helper to print float values using xil_printf
 */
void PrintFloat(float val)
{
    if (val < 0) {
        xil_printf("-");
        val = -val;
    }
    
    int iPart = (int)val;
    int fPart = (int)((val - iPart) * 10000);
    xil_printf("%d.%04d", iPart, fPart);
}

