/**
 * @file opu_task.h
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2022-11-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */

#ifndef __OPUTASK_H__
#define __OPUTASK_H__

#include "../common/common.h"

/*
* Define
*/

#define UART_DEVICE_ID                  XPAR_XUARTPS_0_DEVICE_ID

#define XPAR_FABRIC_LN_IRQ0_INTR		61U			// IRQ0 : 동기 IRQ
#define XPAR_FABRIC_LN_IRQ1_INTR		62U			// IRQ1 : PCM 출력 IRQ
#define XPAR_FABRIC_LN_IRQ2_INTR		63U			// IRQ2 : 10 ms
#define XPAR_FABRIC_LN_IRQ3_INTR		64U			// IRQ3 : 1 ms
#define XPAR_FABRIC_LN_IRQ4_INTR		65U			// IRQ4 : 50 ms
#define XPAR_FABRIC_LN_IRQ5_INTR		66U			// IRQ5 : 100 ms
#define XPAR_FABRIC_LN_IRQ6_INTR		67U			// IRQ6 : 1000 ms

#define XPAR_FABRIC_IRQ_PRIORITY		0x90		// ZYNQ 7000계열에서 IRQ 우선순위 0x90부터 설정 가능
#define XPAR_FABRIC_IRQ_RISING_EDGE		3			// 인터럽트 Rising edge 설정
#define XADC_BASE 						(0x43C00000)	// XADC Base Address

/* Ring Buffer Define */
#define MAX_RB_IDX			50					// RingBuffer index 최댓값
#define MAX_RB_DATA			1528				// RingBuffer Data 사이즈 최댓값

#define UART_MAX_CH		4
#define DIG_MAX_CH		8

/* IMU */
#define HEADER_SIZE     2
#define MESSAGE_SIZE    42
#define MESSAGE_COUNT   5

/* Ring buffer 저장 데이터 구조체 */
typedef struct
{
	UInt32 usSize;					// 데이터 사이즈
	UInt8 ucData[MAX_RB_DATA];		// 데이터
} __attribute__((packed)) sRbData;

/* Ring buffer 정보 */
typedef struct
{
	UInt32 uiAddr;			// DDR3 시작 주소
	SInt32 siFront;			// Ring buffer Front
	SInt32 siRear;			// Ring buffer Rear
	SInt32 siCount;			// Ring buffer Count
} __attribute__((packed)) sRingBufInfo;

/* RS422 수신 구조체 */
typedef struct
{
	UInt32 uiBufSize;			// 수신 버퍼 크기
	UInt8 ucRecvBuf[UART_BRAM_SIZE];
} __attribute__((packed)) sSerialRecvMsg;


typedef union {
    UInt8 ucMsgBuf[UART_BRAM_SIZE+4];
    sSerialRecvMsg stSerialRecvMsg;
} __attribute__((packed)) plSerialPacket_t;

extern void OpuTask( void *pvParameters );
SInt32 SendToCom1(UInt8 *pData, UInt32 uiLen);

#endif //__OPUTASK_H__
