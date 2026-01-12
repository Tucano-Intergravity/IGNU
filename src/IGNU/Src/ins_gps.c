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
static GpsData_t stGlobalGpsData;

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

void SetGpsData(GpsData_t *pData)
{
    if (pData) {
        memcpy(&stGlobalGpsData, pData, sizeof(GpsData_t));
    }
}

void GetGpsData(GpsData_t *pData)
{
    if (pData) {
        memcpy(pData, &stGlobalGpsData, sizeof(GpsData_t));
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

/**
 * @brief Helper to print double values using xil_printf
 */
void PrintDouble(double val)
{
    if (val < 0) {
        xil_printf("-");
        val = -val;
    }
    
    int iPart = (int)val;
    int fPart = (int)((val - iPart) * 1000000);
    xil_printf("%d.%06d", iPart, fPart);
}

/**
 * @brief Safe Parsing of Custom Raw GPS Packet
 * Uses memcpy to copy fields byte-by-byte to prevent Hard Faults 
 * due to Unaligned Memory Access (e.g. double at offset 10).
 * 
 * @param pRawData Pointer to raw packet buffer (91 bytes)
 * @param pOutput Pointer to Aligned GpsData_t
 * @return SInt32 0 on Success, -1 on Error
 */
SInt32 ParseGpsPacket(const UInt8 *pRawData, GpsData_t *pOutput)
{
    if (pRawData == NULL || pOutput == NULL) return -1;

    /* 1. Check Sync Word (0x2440 at Offset 0) */
    /* Note: Sync Word is Big Endian? Or Little Endian 0x24 0x40? 
       User spec says: Sync Word is Big Endian 0x2440. 
       Assuming byte stream is 0x24 0x40. 
    */
    if (pRawData[0] != 0x24 || pRawData[1] != 0x40) {
        /* Sync Failed */
        return -1; 
    }

    /* 
     * 2. Safe Parsing using memcpy 
     * We MUST use memcpy for multi-byte fields because pRawData + Offset 
     * might not be aligned (e.g. Latitude double at Offset 10).
     * Direct casting like *(double*)(pRawData+10) WILL cause Hard Fault on ARM M0/M3/M4/M7.
     */

    /* Offset 2: TOW (UInt32) */
    memcpy(&pOutput->tow, &pRawData[2], 4);

    /* Offset 6: WNC (UInt16) */
    memcpy(&pOutput->wnc, &pRawData[6], 2);

    /* Offset 8, 9: Mode, Error (UInt8 - Safe) */
    pOutput->mode = pRawData[8];
    pOutput->error = pRawData[9];

    /* Offset 10: Latitude (Double - 8 Bytes - Unaligned!) */
    memcpy(&pOutput->latitude, &pRawData[10], 8);

    /* Offset 18: Longitude (Double - 8 Bytes) */
    memcpy(&pOutput->longitude, &pRawData[18], 8);

    /* Offset 26: Height (Double - 8 Bytes) */
    memcpy(&pOutput->height, &pRawData[26], 8);

    /* Offset 34: Undulation (Float - 4 Bytes) */
    memcpy(&pOutput->undulation, &pRawData[34], 4);

    /* Offset 38: Vn (Float - 4 Bytes) */
    memcpy(&pOutput->vn, &pRawData[38], 4);

    /* Offset 42: Ve (Float - 4 Bytes) */
    memcpy(&pOutput->ve, &pRawData[42], 4);

    /* Offset 46: Vu (Float - 4 Bytes) */
    memcpy(&pOutput->vu, &pRawData[46], 4);

    /* Offset 50: Gog (Float - 4 Bytes) */
    memcpy(&pOutput->gog, &pRawData[50], 4);

    /* Offset 54: RxClkBias (Double - 8 Bytes) */
    memcpy(&pOutput->rxClkBias, &pRawData[54], 8);

    /* Offset 62: RxClkDrift (Float - 4 Bytes) */
    memcpy(&pOutput->rxClkDrift, &pRawData[62], 4);

    /* Offset 68: NrSV (UInt8) */
    pOutput->nrSv = pRawData[68];

    /* Offset 84: H-Accuracy (UInt16) */
    memcpy(&pOutput->hAccuracy, &pRawData[84], 2);

    /* Offset 86: V-Accuracy (UInt16) */
    memcpy(&pOutput->vAccuracy, &pRawData[86], 2);

    return 0; // Success
}
