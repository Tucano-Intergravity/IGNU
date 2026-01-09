/**
 * @file TMTC.c
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief Telemetry & Telecommand Processing Source File (KISS, CSP, CCSDS)
 * @version 1.3.0 (Fix: R06.6 Compliance - 12B Header, 4B Response)
 * @date 2026-01-09
 */

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "../Inc/TMTC.h"
#include "../../OPU/opu_task.h" // For SendToCom1
#include "xil_printf.h"

/*==============================================================================
 * Define (KISS Protocol)
 *============================================================================*/
#define KISS_FEND       0xC0
#define KISS_FESC       0xDB
#define KISS_TFEND      0xDC
#define KISS_TFESC      0xDD
#define KISS_CMD_DATA   0x00

/*==============================================================================
 * Type Definition
 *============================================================================*/
typedef enum {
    KISS_STATE_WAIT_FEND,
    KISS_STATE_DATA,
    KISS_STATE_ESCAPE
} KissState_t;

/*==============================================================================
 * Local Variables
 *============================================================================*/
static UInt8 ucKissBuf[MAX_KISS_BUF];
static UInt32 uiKissIdx = 0;
static KissState_t eKissState = KISS_STATE_WAIT_FEND;

/*==============================================================================
 * Local Function Declarations
 *============================================================================*/
static UInt32 Crc32Check(UInt8 *pData, UInt32 uiLen);
static UInt16 Crc16Check(UInt8 *pData, UInt32 uiLen);
static void CcsdsReceive(UInt8 *pCcsdsPacket, UInt32 uiLen);
static UInt32 KissEncode(UInt8 *pInput, UInt32 uiInputLen, UInt8 *pOutput);
static void SendCcsdsTm(UInt8 ucSvc, UInt8 ucSub, UInt8 *pData, UInt32 uiDataLen);

/* Handler Functions */
static void ProcTestStart(void);
static void ProcTestStop(void);
static void ProcSetTestParam(void);
static void ProcSaveTpvaw(void);
static void ProcReqTestData(UInt8 ucType);
static void ProcHkReq(void);
static void ProcFuncExec(void);
static void ProcPing(void);

/*==============================================================================
 * Functions
 *============================================================================*/

SInt32 KissDecode(UInt8 ucByte, UInt8 *pDecodedBuf)
{
    SInt32 siRetLen = 0;

    switch (eKissState)
    {
    case KISS_STATE_WAIT_FEND:
        if (ucByte == KISS_FEND) {
            uiKissIdx = 0;
            eKissState = KISS_STATE_DATA;
        }
        break;

    case KISS_STATE_DATA:
        if (ucByte == KISS_FEND) {
            if (uiKissIdx > 0) {
                /* Allow any CMD_CODE for debugging */
                if (1) {
                    if (uiKissIdx > 1) {
                        siRetLen = uiKissIdx - 1;
                        memcpy(pDecodedBuf, &ucKissBuf[1], siRetLen);
                    }
                }
            }
            uiKissIdx = 0;
        }
        else if (ucByte == KISS_FESC) {
            eKissState = KISS_STATE_ESCAPE;
        }
        else {
            if (uiKissIdx < MAX_KISS_BUF) ucKissBuf[uiKissIdx++] = ucByte;
            else { uiKissIdx = 0; eKissState = KISS_STATE_WAIT_FEND; }
        }
        break;

    case KISS_STATE_ESCAPE:
        if (ucByte == KISS_TFEND) {
            if (uiKissIdx < MAX_KISS_BUF) ucKissBuf[uiKissIdx++] = KISS_FEND;
        }
        else if (ucByte == KISS_TFESC) {
            if (uiKissIdx < MAX_KISS_BUF) ucKissBuf[uiKissIdx++] = KISS_FESC;
        }
        else {
             if (uiKissIdx < MAX_KISS_BUF) ucKissBuf[uiKissIdx++] = ucByte;
        }
        eKissState = KISS_STATE_DATA;
        break;

    default:
        eKissState = KISS_STATE_WAIT_FEND;
        break;
    }

    return siRetLen;
}

/* CCSDS CRC-16 (CCITT-FALSE) */
static UInt16 Crc16Check(UInt8 *pData, UInt32 uiLen)
{
    UInt16 crc = 0xFFFF;
    UInt32 i, j;
    
    for (i = 0; i < uiLen; i++) {
        crc ^= (UInt16)(pData[i] << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

/* CSP CRC-32C (Castagnoli Reflected) */
static UInt32 Crc32Check(UInt8 *pData, UInt32 uiLen)
{
    UInt32 crc = 0xFFFFFFFF;
    UInt32 i, j;
    
    for (i = 0; i < uiLen; i++) {
        crc ^= pData[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x82F63B78;
            else crc = (crc >> 1);
        }
    }
    return ~crc;
}

static UInt32 KissEncode(UInt8 *pInput, UInt32 uiInputLen, UInt8 *pOutput)
{
    UInt32 i;
    UInt32 uiIdx = 0;
    pOutput[uiIdx++] = KISS_FEND;
    pOutput[uiIdx++] = KISS_CMD_DATA;
    for(i=0; i<uiInputLen; i++) {
        if(pInput[i] == KISS_FEND) {
            pOutput[uiIdx++] = KISS_FESC; pOutput[uiIdx++] = KISS_TFEND;
        } else if(pInput[i] == KISS_FESC) {
            pOutput[uiIdx++] = KISS_FESC; pOutput[uiIdx++] = KISS_TFESC;
        } else {
            pOutput[uiIdx++] = pInput[i];
        }
    }
    pOutput[uiIdx++] = KISS_FEND;
    return uiIdx;
}

SInt32 CspSend(UInt8 dest, UInt8 dport, UInt8 *pData, UInt32 uiLen)
{
    UInt8 ucRawPkt[MAX_KISS_BUF];
    UInt8 ucKissFrame[MAX_KISS_BUF * 2];
    UInt32 uiPktLen = 0;
    UInt32 uiKissLen = 0;

    /* 1. CSP Header (4 Bytes, Big Endian) */
    UInt32 uiHeader = 0;
    uiHeader |= (2 & 0x03) << 30; // Priority=2
    uiHeader |= (dest & 0x1F) << 25;
    uiHeader |= (CSP_MY_ADDR & 0x1F) << 20;
    uiHeader |= (dport & 0x3F) << 14;
    uiHeader |= (CSP_PORT_CMD_RX & 0x3F) << 8; // Source Port
    uiHeader |= 0x00; // Flags

    ucRawPkt[0] = (uiHeader >> 24) & 0xFF;
    ucRawPkt[1] = (uiHeader >> 16) & 0xFF;
    ucRawPkt[2] = (uiHeader >> 8) & 0xFF;
    ucRawPkt[3] = uiHeader & 0xFF;
    uiPktLen += 4;

    /* 2. Payload */
    memcpy(&ucRawPkt[uiPktLen], pData, uiLen);
    uiPktLen += uiLen;

    /* 3. CRC32 */
    UInt32 uiCrc = Crc32Check(ucRawPkt, uiPktLen);
    ucRawPkt[uiPktLen++] = (uiCrc >> 24) & 0xFF;
    ucRawPkt[uiPktLen++] = (uiCrc >> 16) & 0xFF;
    ucRawPkt[uiPktLen++] = (uiCrc >> 8) & 0xFF;
    ucRawPkt[uiPktLen++] = uiCrc & 0xFF;

    /* 4. Encode & Send */
    uiKissLen = KissEncode(ucRawPkt, uiPktLen, ucKissFrame);

    SendToCom1(ucKissFrame, uiKissLen);

    return 0;
}

/**
 * @brief Universal Helper to send correct CCSDS Telemetry Packet (R06.6)
 * Handles:
 * - 12 Bytes Secondary Header (Svc, Sub, Src, Time, Flags, Pad)
 * - Correct APID (0x550)
 * - CRC-16 (2 Bytes) at the end
 */
static void SendCcsdsTm(UInt8 ucSvc, UInt8 ucSub, UInt8 *pData, UInt32 uiDataLen)
{
    UInt8 ucBuffer[MAX_TM_DATA + 32];
    UInt32 uiLen = 0;

    /* 1. Primary Header (6 Bytes) */
    /* Packet ID: Version(0) | Type(0=TM) | SecHdr(1) | APID(11) */
    UInt16 usPacketId = 0x0800 | (CCSDS_APID_IGNU & 0x07FF);
    ucBuffer[uiLen++] = (usPacketId >> 8) & 0xFF;
    ucBuffer[uiLen++] = usPacketId & 0xFF;

    /* Sequence Control (Unsegmented) */
    ucBuffer[uiLen++] = 0xC0;
    ucBuffer[uiLen++] = 0x00;
    
    /* Length Field = SecHdr(12) + UserData(N) + CRC(2) - 1 */
    UInt16 usPktLen = (UInt16)(CCSDS_TM_SEC_HEADER_SIZE + uiDataLen + 2 - 1);
    ucBuffer[uiLen++] = (usPktLen >> 8) & 0xFF;
    ucBuffer[uiLen++] = usPktLen & 0xFF;

    /* 2. Secondary Header (12 Bytes - R06.6) */
    ucBuffer[uiLen++] = ucSvc;
    ucBuffer[uiLen++] = ucSub;
    /* Source ID (2B) */
    ucBuffer[uiLen++] = (CCSDS_APID_IGNU >> 8) & 0xFF;
    ucBuffer[uiLen++] = CCSDS_APID_IGNU & 0xFF;
    
    /* Time Stamp (6 Bytes) - Zero Padding */
    memset(&ucBuffer[uiLen], 0, 6);
    uiLen += 6;
    
    /* Flags(1B) + Padding(1B) = 2 Bytes */
    ucBuffer[uiLen++] = 0x00; // Flags (E_CRC=0) + Pad
    ucBuffer[uiLen++] = 0x00; // Padding (Spare)

    /* 3. User Data */
    if (pData != NULL && uiDataLen > 0) {
        memcpy(&ucBuffer[uiLen], pData, uiDataLen);
        uiLen += uiDataLen;
    }

    /* 4. CRC-16 (Packet Error Control) */
    UInt16 usCrc = Crc16Check(ucBuffer, uiLen);
    ucBuffer[uiLen++] = (usCrc >> 8) & 0xFF;
    ucBuffer[uiLen++] = usCrc & 0xFF;

    /* 5. Send via CSP */
    CspSend(CSP_PDHS_ADDR, CSP_PORT_ASYNC_TX, ucBuffer, uiLen);
}

/**
 * @brief Send Default Response (4 Bytes User Data as per R06.6 Table 9)
 * Payload: [Ack(1B) | Response Code(3B)]
 */
void SendResponse(UInt8 ucSvc, UInt8 ucSub, UInt8 ucAck)
{
    UInt8 ucPayload[4];

    ucPayload[0] = ucAck; // 0xFF (Valid) or 0x00 (Invalid)
    ucPayload[1] = 0x00;  // Code High
    ucPayload[2] = 0x00;  // Code Mid
    ucPayload[3] = 0x00;  // Code Low

    SendCcsdsTm(ucSvc, ucSub, ucPayload, 4);
}

SInt32 CspReceive(UInt8 *pPacket, SInt32 siLen)
{
    if (siLen < (CSP_HEADER_SIZE + CSP_CRC32_SIZE)) return -1;

    UInt32 uiPayloadLen = siLen - CSP_CRC32_SIZE;
    UInt32 uiCalcCrc = Crc32Check(pPacket, uiPayloadLen);
    UInt32 uiRecvCrc = (pPacket[siLen-4] << 24) | (pPacket[siLen-3] << 16) | 
                       (pPacket[siLen-2] << 8) | pPacket[siLen-1];

    if (uiCalcCrc != uiRecvCrc) {
        xil_printf("[CSP] Error: CRC Mismatch\r\n");
        return -2;
    }

    UInt32 uiHeaderVal = (pPacket[0] << 24) | (pPacket[1] << 16) | (pPacket[2] << 8) | pPacket[3];
    UInt8 dest = (uiHeaderVal >> 25) & 0x1F;
    UInt8 dport = (uiHeaderVal >> 14) & 0x3F;

    if (dest != CSP_MY_ADDR) return -3;

    if (dport == CSP_PORT_CMD_RX) {
        CcsdsReceive(&pPacket[CSP_HEADER_SIZE], uiPayloadLen - CSP_HEADER_SIZE);
    }
    return 0;
}

static void CcsdsReceive(UInt8 *pCcsdsPacket, UInt32 uiLen)
{
    /* Check Minimum Length: Pri(6) + TC_Sec(4) = 10 */
    if (uiLen < (CCSDS_PRI_HEADER_SIZE + CCSDS_TC_SEC_HEADER_SIZE)) return;

    UInt16 usApid = ((pCcsdsPacket[0] & 0x07) << 8) | pCcsdsPacket[1];
    UInt8 *pSecHeader = &pCcsdsPacket[CCSDS_PRI_HEADER_SIZE];
    UInt8 ucServiceId = pSecHeader[0];
    UInt8 ucSubtypeId = pSecHeader[1];

    xil_printf("[CCSDS] APID:0x%X Svc:%d Sub:%d\r\n", usApid, ucServiceId, ucSubtypeId);

    /* Use 0xFF for Success/Valid Ack */
    switch (ucServiceId)
    {
    case PUS_SVC_TEST:
        if (ucSubtypeId == PUS_SUB_TEST_START) ProcTestStart();
        else if (ucSubtypeId == PUS_SUB_TEST_STOP) ProcTestStop();
        else if (ucSubtypeId == PUS_SUB_TEST_SET_PARAM) ProcSetTestParam();
        else if (ucSubtypeId == PUS_SUB_TEST_SEND_TPVAW) ProcSaveTpvaw();
        else if (ucSubtypeId >= PUS_SUB_TEST_DATA_MIN && ucSubtypeId <= PUS_SUB_TEST_DATA_MAX) ProcReqTestData(ucSubtypeId);
        else SendResponse(ucServiceId, ucSubtypeId, TM_ACK_INVALID);
        break;
    case PUS_SVC_HK:
        if (ucSubtypeId == PUS_SUB_HK_REQ) ProcHkReq();
        else SendResponse(ucServiceId, ucSubtypeId, TM_ACK_INVALID);
        break;
    case PUS_SVC_FUNCTION:
        if (ucSubtypeId == PUS_SUB_FUNC_EXEC) ProcFuncExec();
        else SendResponse(ucServiceId, ucSubtypeId, TM_ACK_INVALID);
        break;
    case PUS_SVC_DIAGNOSE:
        if (ucSubtypeId == PUS_SUB_DIAG_PING) ProcPing();
        else SendResponse(ucServiceId, ucSubtypeId, TM_ACK_INVALID);
        break;
    default:
        /* Invalid Service */
        SendResponse(ucServiceId, ucSubtypeId, TM_ACK_INVALID);
        break;
    }
}

static void ProcTestStart(void) {
    xil_printf("[CMD] Start Test\r\n");
    SendResponse(PUS_SVC_TEST, PUS_SUB_TEST_START, TM_ACK_VALID);
}
static void ProcTestStop(void) {
    xil_printf("[CMD] Stop Test\r\n");
    SendResponse(PUS_SVC_TEST, PUS_SUB_TEST_STOP, TM_ACK_VALID);
}
static void ProcSetTestParam(void) {
    xil_printf("[CMD] Set Param\r\n");
    SendResponse(PUS_SVC_TEST, PUS_SUB_TEST_SET_PARAM, TM_ACK_VALID);
}
static void ProcSaveTpvaw(void) {
    xil_printf("[CMD] TPVAW\r\n");
    SendResponse(PUS_SVC_TEST, PUS_SUB_TEST_SEND_TPVAW, TM_ACK_VALID);
}
static void ProcFuncExec(void) {
    xil_printf("[CMD] Func Exec\r\n");
    SendResponse(PUS_SVC_FUNCTION, PUS_SUB_FUNC_EXEC, TM_ACK_VALID);
}
static void ProcPing(void) {
    xil_printf("[CMD] Ping\r\n");
    SendResponse(PUS_SVC_DIAGNOSE, PUS_SUB_DIAG_PONG, TM_ACK_VALID);
}

static void ProcReqTestData(UInt8 ucType) {
    xil_printf("[CMD] Req Test Data %d\r\n", ucType);
    UInt8 ucDummy[16];
    memset(ucDummy, 0xAA, 16);
    /* Svc 1, Sub ucType, Data 16 bytes */
    SendCcsdsTm(PUS_SVC_TEST, ucType, ucDummy, 16);
}

static void ProcHkReq(void) {
    xil_printf("[CMD] HK Req\r\n");
    UInt8 ucDummy[4];
    memset(ucDummy, 0x55, 4);
    /* Svc 5, Sub 1, Data 4 bytes */
    SendCcsdsTm(PUS_SVC_HK, 1, ucDummy, 4);
}
