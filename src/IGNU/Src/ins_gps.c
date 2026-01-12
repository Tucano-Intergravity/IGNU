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
 * @brief Parse GPS Packet (Mock Implementation)
 * Assumes NovAtel Binary Log structure or similar custom format
 * 
 * @param pData Pointer to raw packet buffer (90 bytes)
 * @param pOutput Pointer to GpsData_t to populate
 * @return SInt32 0 on Success, -1 on Error
 */
SInt32 ParseGpsPacket(UInt8 *pData, GpsData_t *pOutput)
{
    if (pData == NULL || pOutput == NULL) return -1;

    /* Check Sync Bytes (Assuming NovAtel AA 44 12) */
    if (pData[0] != 0xAA || pData[1] != 0x44 || pData[2] != 0x12) {
        // xil_printf("GPS Sync Error: %02X %02X %02X\r\n", pData[0], pData[1], pData[2]);
        // Return 0 for now to allow mocked testing if real hardware isn't attached
        // return -1; 
    }

    /* 
     * TODO: Implement actual parsing logic based on ICD.
     * For now, we populate with some data from the packet or defaults.
     * Assuming standard NovAtel Header is 28 bytes.
     */

    /* Header Parsing */
    // pOutput->wnc = (pData[14] << 8) | pData[15]; // Week
    // pOutput->tow = (pData[16] << 24) | (pData[17] << 16) | (pData[18] << 8) | pData[19]; // ms

    /* Mocking data extraction - Mapping based on ignu_task.c print */
    /* It printed Lat from offset 10..17 (8 bytes double) */
    
    // memcpy(&pOutput->latitude, &pData[10], 8);
    // memcpy(&pOutput->longitude, &pData[18], 8);
    // memcpy(&pOutput->height, &pData[26], 8);
    
    /* Just use placeholders for now to satisfy build */
    pOutput->tow = 0;
    pOutput->wnc = 0;
    pOutput->latitude = 0.0;
    pOutput->longitude = 0.0;
    pOutput->height = 0.0;
    pOutput->vn = 0.0f;
    pOutput->ve = 0.0f;
    pOutput->vu = 0.0f;
    pOutput->mode = 0;
    pOutput->error = 0;
    pOutput->nrSv = 0;

    return 0; // Success
}
