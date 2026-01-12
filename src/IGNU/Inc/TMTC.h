/**
 * @file TMTC.h
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief Telemetry & Telecommand Processing Header
 * @version 1.5.0 (Fix: Use __attribute__((packed)) for Safe ICD Compliance)
 * @date 2026-01-09
 */

#ifndef __TMTC_H__
#define __TMTC_H__

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "FreeRTOS.h"
#include "../../common/common.h"

/*==============================================================================
 * Define
 *============================================================================*/
#define MAX_KISS_BUF    1024 // Max payload size for KISS frames
#define MAX_TM_DATA     512  // Max Telemetry Data Size

/* CSP Definitions */
#define CSP_HEADER_SIZE 4
#define CSP_CRC32_SIZE  4
#define CSP_MY_ADDR     19   // IGNU Address (Page 13)
#define CSP_PDHS_ADDR   6    // PDHS Address

/* CSP Port Definitions (ICD Table 12) */
#define CSP_PORT_CMD_RX     10   // RX: Standard Command Service
#define CSP_PORT_ASYNC_TX   11   // TX: Asynchronous Response (To PDHS)

/* CCSDS Definitions */
#define CCSDS_APID_IGNU     0x550 // IGNU Application Process ID (Table 5)
#define CCSDS_PRI_HEADER_SIZE 6
#define CCSDS_TC_SEC_HEADER_SIZE 4   // TC: Svc(1)+Sub(1)+Src(2)
#define CCSDS_TM_SEC_HEADER_SIZE 12  // TM: Svc(1)+Sub(1)+Src(2)+Time(6)+Flags(1)+Spare(1) = 12 Bytes

/* Response Constants (Table 9) */
#define TM_ACK_VALID        0xFF // Valid TC
#define TM_ACK_INVALID      0x00 // Invalid TC

/* PUS Service IDs (Based on E3T ICD R06.6) */
#define PUS_SVC_TEST        1    // Test Service
#define PUS_SVC_HK          5    // Housekeeping
#define PUS_SVC_FUNCTION    8    // Function Management
#define PUS_SVC_DIAGNOSE    20   // Diagnose

/* PUS Message Subtype IDs */
/* Service 1: Test */
#define PUS_SUB_TEST_START      1    // Start Test
#define PUS_SUB_TEST_STOP       2    // Stop Test
#define PUS_SUB_TEST_SET_PARAM  4    // Set Test Parameters
#define PUS_SUB_TEST_SEND_TPVAW 5    // Send TPVAW
#define PUS_SUB_TEST_REQ_DATA   10   // Request Test Data (Start)
#define PUS_SUB_TEST_DATA_MIN   10   // Test Data Subtype Range Start
#define PUS_SUB_TEST_DATA_MAX   127  // Test Data Subtype Range End

/* Service 5: Housekeeping */
#define PUS_SUB_HK_REQ      1    // One Shot HK Request / Report

/* Service 8: Function Management */
#define PUS_SUB_FUNC_EXEC   1    // Perform Function

/* Service 20: Diagnose */
#define PUS_SUB_DIAG_PING   1    // Ping Request
#define PUS_SUB_DIAG_PONG   1    // Ping Reply (Pong)

/*==============================================================================
 * Type Definition
 *============================================================================*/
typedef struct {
    UInt8 pri;
    UInt8 dest;
    UInt8 src;
    UInt8 dport;
    UInt8 sport;
    UInt8 flags;
} csp_header_t;

/* ============================================================================
 * 6.2.1 Payload Status Telemetry (Reply Status)
 * Total Size: 6 Bytes (ICD Compliant)
 * ============================================================================ */
typedef struct __attribute__((packed)) {
    UInt8  payloadStatus;   // 1 byte
    // _reserved1 removed
    
    SInt16  boardTemp;       // 2 bytes
    
    UInt8  imuStatus;       // 1 byte
    UInt8  gpsStatus;       // 1 byte
    UInt8  gpsTrackStatus;  // 1 byte
    
    // _reserved2 removed
} PayloadStatus_t;

/* ============================================================================
 * 6.2.2 Test Data Telemetry (Reply Test Data)
 * Total Size: 76 Bytes (ICD Compliant)
 * ============================================================================ */
//typedef struct __attribute__((packed)) {
//    /* GPS Time Information */
//    UInt32 gpsWeek;         // 4 bytes
//    UInt32 gpsTime;         // 4 bytes
//
//    /* Position */
//    double   lat;             // 8 bytes
//    double   lon;             // 8 bytes
//    float    alt;             // 4 bytes (ICD says 32bit float)
//
//    /* Velocity */
//    float    velN;            // 4 bytes
//    float    velE;            // 4 bytes
//    float    velU;            // 4 bytes
//
//    /* Status */
//    UInt8  mode;            // 1 byte
//    UInt8  error;           // 1 byte
//    UInt8  NrSV;            // 1 byte
//    // _reserved_align removed
//
//    /* IMU Data (Gyro) */
//    float    meanGyroX;       // 4 bytes
//    float    meanGyroY;       // 4 bytes
//    float    meanGyroZ;       // 4 bytes
//
//    /* IMU Data (Accel) */
//    float    meanAccX;        // 4 bytes
//    float    meanAccY;        // 4 bytes
//    float    meanAccZ;        // 4 bytes
//
//    /* Attitude */
//    float    roll;            // 4 bytes
//    float    pitch;           // 4 bytes
//    float    yaw;             // 4 bytes
//
//    /* Reserved Area REMOVED */
//    // UInt32 reserved[5]; removed to match ICD
//} TestData_t;

/* ============================================================================
 * 6.2.2 Test Data Telemetry (Reply Test Data)
 * Service: 1, Subtype: 10
 * Total Size: 100 Bytes (Modified for Alignment)
 * ============================================================================ */
typedef struct {
    /* GPS Time Information */
    UInt32 gpsWeek;         // Offset: 0, 32 bits (Modified from 16 bits)
    UInt32 gpsTime;         // Offset: 4, 32 bits (Unit: sec, Little Endian)

    /* Position (8-byte aligned) */
    double   lat;             // Offset: 8,  64 bits (Unit: degree, Little Endian)
    double   lon;             // Offset: 16, 64 bits (Unit: degree, Little Endian)

    /* Altitude & Velocity */
    float    alt;             // Offset: 24, 32 bits (Unit: m, Little Endian)
    float    velN;            // Offset: 28, 32 bits (Unit: m/s, Little Endian)
    float    velE;            // Offset: 32, 32 bits (Unit: m/s, Little Endian)
    float    velU;            // Offset: 36, 32 bits (Unit: m/s, Little Endian)

    /* Status & Padding */
    UInt8  mode;            // Offset: 40, 8 bits (GPS PVT mode)
    UInt8  error;           // Offset: 41, 8 bits (GPS PVT error)
    UInt8  NrSV;            // Offset: 42, 8 bits (GPS observable satellite)
    UInt8  _reserved_align; // Offset: 43, 8 bits (Padding for 4-byte alignment)

    /* IMU Data (Gyro) */
    float    meanGyroX;       // Offset: 44, 32 bits (Unit: degree/s, Little Endian)
    float    meanGyroY;       // Offset: 48, 32 bits (Unit: degree/s, Little Endian)
    float    meanGyroZ;       // Offset: 52, 32 bits (Unit: degree/s, Little Endian)

    /* IMU Data (Accel) */
    float    meanAccX;        // Offset: 56, 32 bits (Unit: m/s^2, Little Endian)
    float    meanAccY;        // Offset: 60, 32 bits (Unit: m/s^2, Little Endian)
    float    meanAccZ;        // Offset: 64, 32 bits (Unit: m/s^2, Little Endian)

    /* Attitude */
    float    roll;            // Offset: 68, 32 bits (Unit: degree, Little Endian)
    float    pitch;           // Offset: 72, 32 bits (Unit: degree, Little Endian)
    float    yaw;             // Offset: 76, 32 bits (Unit: degree, Little Endian)

    /* Reserved Area */
    UInt32 reserved[5];     // Offset: 80 ~ 99, 5 * 32 bits
} TestData_t;



/*==============================================================================
 * Global Function Declarations
 *============================================================================*/
SInt32 KissDecode(UInt8 ucByte, UInt8 *pDecodedBuf);
SInt32 CspReceive(UInt8 *pPacket, SInt32 siLen);
SInt32 CspSend(UInt8 dest, UInt8 dport, UInt8 *pData, UInt32 uiLen);
void SendResponse(UInt8 ucSvc, UInt8 ucSub, UInt8 ucAck);
void SendTestData(void);

#endif /* __TMTC_H__ */
