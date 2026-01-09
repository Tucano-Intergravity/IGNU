/**
 * @file TMTC.h
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief Telemetry & Telecommand Processing Header
 * @version 1.3.0 (Fix: R06.6 Compliance - 12B Header, 4B Response)
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
#define CCSDS_TM_SEC_HEADER_SIZE 12  // TM: Svc(1)+Sub(1)+Src(2)+Time(6)+Flags(1)+Pad(1) = 12 Bytes (R06.6)

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
 * Service: 5, Subtype: 1
 * Total Size: 8 Bytes (Previously 6 Bytes)
 * ============================================================================ */
typedef struct {
    UInt8  payloadStatus;   // Offset: 0, 8 bits (0=Idle, 1=Testing, ...)
    UInt8  _reserved1;      // Offset: 1, 8 bits (Padding for alignment)

    SInt16  boardTemp;       // Offset: 2, 16 bits (Unit: 0.1 degC, Little Endian)

    UInt8  imuStatus;       // Offset: 4, 8 bits (0=Normal, 1=Fault)
    UInt8  gpsStatus;       // Offset: 5, 8 bits (0=Normal, 1=Fault)
    UInt8  gpsTrackStatus;  // Offset: 6, 8 bits (GPS tracking status code)

    UInt8  _reserved2;      // Offset: 7, 8 bits (Padding to match 4/8 byte alignment)
} PayloadStatus_t;


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
    UInt8  trackStatus;     // Offset: 40, 8 bits (GPS tracking status code)
    UInt8  _reserved_align[3]; // Offset: 41, 24 bits (Padding for 4-byte alignment)

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
