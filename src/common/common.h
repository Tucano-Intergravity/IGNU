/**
 * @file common.h
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include "lwip/inet.h"

#pragma pack(1)

/*==============================================================================
 * Common Define
 *============================================================================*/

/* --- Test --- */
#define TASK_STACK_SIZE_CHECK		0	// Task Stack Size 계산

/* --- Data Type --- */
typedef unsigned char    	UInt8;
typedef signed char    		SInt8;
typedef unsigned short   	UInt16;
typedef signed short   		SInt16;
typedef unsigned int    	UInt;
typedef signed int    		SInt;
typedef unsigned long    	UInt32;         // 64비트 OS에서 64bits
typedef signed long    		SInt32;         // 64비트 OS에서 64bits
typedef unsigned long long  UInt64;
typedef signed long long    SInt64;

/* --- PS MODE --- */
enum {
    PS_MODE_INIT = 0,
    PS_MODE_OP,
    PS_MODE_DOWN,
    PS_MODE_CHECK,
};

/* --- Status --- */
enum {
    STATUS_NORMAL = 0,
    STATUS_FAILURE,
};

/* --- Device --- */
#define GPIO_DEVICE_ID			XPAR_XGPIOPS_0_DEVICE_ID	// GPIO Device ID

#define MAX_UART_CH		6		// UART 채널수
#define MAX_DIG_CH		8		// 디지털모쒮(RS422) 채널수
#define MAX_CAN_CH		2		// CAN 모듈 채널수

/*****************************************************************************/
/**
* @def MAX_PACKET_SIZE
* @brief 최대 패킷 크기 정의
*
*  최대 패킷의 크기는 UDP 패킷 구조를 기준으로 설정.
*   - 프리엠블 : 7 bytes
*   - SFD : 1 byte
*   - 이더넷 헤더 : 14 bytes
*   - IP 헤더 : 20 bytes
*   - UDP 헤더 : 8 bytes
*   - 최대 UDP 데이터 (Payload) : 1472 bytes
*   - FCS : 4 bytes
*
*  프리엠블과 SFD를 포함한 패킷의 최대 크기는 1526 bytes이며,
*  4바이트로 정렬된 메모리 접근이 더 효율적인 하드웨어 아키텍처에서 성능 최적화를 위해 2bytes의 패딩이 포함됨.
*
******************************************************************************/
/* --- SLOT --- */
#define MAX_PACKET_SIZE			1528
#define MAX_IDX					128		// BRAM index 최댓값

#define MAX_SLOT_CH_SIZE    	128		// BRAM 패킷 사이즈
#define MAX_SLOT_PACKET			31		// BRAM 패킷 버퍼 개수

#define UART_BRAM_SIZE			1528	// BRAM 패킷 사이즈
#define UART_BRAM_PACKET		10		// BRAM 패킷 버퍼 개수

#define DIGITAL_BRAM_SIZE		1528	// BRAM 패킷 사이즈
#define DIGITAL_BRAM_PACKET		42		// BRAM 패킷 버퍼 개수

#define CAN_BRAM_SIZE			1528	// BRAM 패킷 사이즈
#define CAN_BRAM_PACKET			5		// BRAM 패킷 버퍼 개수

#define PCM_BRAM_SIZE			2600	// BRAM 패킷 사이즈
#define PCM_BRAM_PACKET			6		// BRAM 패킷 버퍼 개수

#define NETWORK_BRAM_SIZE		1528	// BRAM 패킷 사이즈
#define NETWORK_BRAM_PACKET		42		// BRAM 패킷 버퍼 개수

#define GPS_BRAM_SIZE			1528	// BRAM 패킷 사이즈
#define GPS_BRAM_PACKET			42		// BRAM 패킷 버퍼 개수

#define IMU_BRAM_SIZE			1528	// BRAM 패킷 사이즈
#define IMU_BRAM_PACKET			42		// BRAM 패킷 버퍼 개수



/* --- Wait time --- */
#define TIME_REQ            1000         // 요청에 대한 수신 대기 시간

/* --- Delay Define --- */
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define DELAY_10_MSECOND	10UL
#define DELAY_5_MSECOND		5UL
#define DELAY_1_MSECOND		1UL

/* --- System Configuration --- */
#define NULL						0U
#define TRUE						1
#define FALSE						0
#define SCDAU_STACK_SIZE			1024

/* --- Checksum --- */
#define CRC_POLY_32					0xEDB88320L

/* --- Message Max Size --- */
#define MSG_MAX                     256

/* --- PL Status Define --- */
#define PL_BRAM_WR_STS				0xFF

/* --- Encoder Command --- */
#define CMD_PL_READY                0x00000001
#define CMD_PL_STOP                 0x00000020

/* --- RS422 TX Command --- */
#define CMD_RS422_CH01_TX_ENABLE	0x00000001
#define CMD_RS422_CH02_TX_ENABLE	0x00000002
#define CMD_RS422_CH03_TX_ENABLE	0x00000003
#define CMD_RS422_CH04_TX_ENABLE	0x00000004
#define CMD_RS422_CH05_TX_ENABLE	0x00000005
#define CMD_RS422_CH06_TX_ENABLE	0x00000006

/* --- Discrete --- */
#define PACKET_SIZE_I12O8			7
#define PACKET_SIZE_O24				8

/* --- PCM Data 생성 --- */
#define PCM_RAWDATA_SIZE	2500
//#define PCM_RAWDATA_SIZE	1440
#define PCM_PACKET_SIZE		(PCM_RAWDATA_SIZE+4)		// CRC 포함


/* --- LVDS TX Command --- */
enum {
	CMD_SLOT_CONF_LVDS1 = 1,
	CMD_SLOT_CONF_LVDS2,
	CMD_SLOT_CONF_LVDS3,
	CMD_SLOT_CONF_LVDS4,
	CMD_SLOT_CONF_LVDS5,
	CMD_SLOT_CONF_LVDS6,
	CMD_SLOT_CONF_LVDS7,
	CMD_SLOT_CONF_LVDS8,
	CMD_SLOT_CONF_LVDS9,
	CMD_SLOT_CONF_LVDS10,
};


/* --- TX Command --- */
enum {
	CMD_SLOT_CONF_UART1 = 1,
	CMD_SLOT_CONF_UART2,
	CMD_SLOT_CONF_UART3,
	CMD_SLOT_CONF_UART4,
	CMD_SLOT_CONF_UART5,
	CMD_SLOT_CONF_UART6,
};

/* 엔코더 속도 - PCM BitRate */
enum
{
	ENC_SPEED_256K = 1,
	ENC_SPEED_512K,
	ENC_SPEED_1024K,
	ENC_SPEED_2048K,
	ENC_SPEED_3072K,
	ENC_SPEED_4096K,
	ENC_SPEED_5120K,
	ENC_SPEED_6144K,
	ENC_SPEED_7168K,
	ENC_SPEED_8192K,
	ENC_SPEED_9216K,
	ENC_SPEED_10240K,
};


/*==============================================================================
 * UART Define
 *============================================================================*/

/* --- UART Baudrate --- */
enum {
	UART_CONF_BAUDRATE_1200 = 1,
	UART_CONF_BAUDRATE_2400,
	UART_CONF_BAUDRATE_4800,
	UART_CONF_BAUDRATE_9600,
	UART_CONF_BAUDRATE_14400,
	UART_CONF_BAUDRATE_19200,
	UART_CONF_BAUDRATE_38400,
	UART_CONF_BAUDRATE_57600,
	UART_CONF_BAUDRATE_115200,
	UART_CONF_BAUDRATE_230400,
	UART_CONF_BAUDRATE_460800,
	UART_CONF_BAUDRATE_921600,
};

/*==============================================================================
 * Network 모듈 Define
 *============================================================================*/

/* --- Network 모듈 설정 --- */
#define NET_MOD_PROTOCOL		PROTOCOL_UDP
#define NET_MOD_COMMUNICATE		COMMUNICATE_UNICAST
#define NET_MOD_DST_IP			"192.168.1.30"
#define NET_MOD_RECV_PORT		5001
#define NET_MOD_SEND_PORT		5002

#define NET_MOD_IP				"192.168.1.20"
#define NET_MOD_NETMASK			"255.255.255.0"
#define NET_MOD_GATEWAY			"192.168.1.0"


/*==============================================================================
 * CAN 모듈 Define
 *============================================================================*/

/* --- CAN Baudrate --- */
enum {
	CAN_CONF_BAUDRATE_100 = 1,
	CAN_CONF_BAUDRATE_125,
	CAN_CONF_BAUDRATE_250,
	CAN_CONF_BAUDRATE_500,
	CAN_CONF_BAUDRATE_800,
	CAN_CONF_BAUDRATE_1000,
};


/*==============================================================================
 * BRAM Define (Memory Physical Address)
 *============================================================================*/

/* --- 제어 명령 Register --- */
#define BRAM_ADDR_CTL_PL			0x40000000		// PS->PL 명령, PL 설정 명령
#define BRAM_ADDR_CTL_LVDS_TX		0x40000010		// PS->PL 명령 (LVDS-TX Enable)
#define BRAM_ADDR_CTL_UART_TX		0x40000020		// PS->PL 명령 (RS422-TX Enable)

/* --- 제어 설정 Register --- */
#define BRAM_ADDR_SET_PCM			0x40000400		// PS->PL 명령 (PCM 설정)
#define BRAM_ADDR_SET_RB_SLOT_01	0x40000410		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_02	0x40000414		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_03	0x40000418		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_04	0x4000041C		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_05	0x40000420		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_06	0x40000424		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_07	0x40000428		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_08	0x4000042C		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_09	0x40000430		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_10	0x40000434		// PS->PL 명령 (SLOT RingBuffer 설정)
#define BRAM_ADDR_SET_RB_SLOT_IM	0x40000438		// PS->PL 명령 (IM RingBuffer 설정)

#define BRAM_ADDR_SET_RB_UART_01	0x40000440		// PS->PL 명령 (UART RingBuffer 설정)
#define BRAM_ADDR_SET_RB_UART_02	0x40000444		// PS->PL 명령 (UART RingBuffer 설정)
#define BRAM_ADDR_SET_RB_UART_03	0x40000448		// PS->PL 명령 (UART RingBuffer 설정)
#define BRAM_ADDR_SET_RB_UART_04	0x4000044C		// PS->PL 명령 (UART RingBuffer 설정)
#define BRAM_ADDR_SET_RB_UART_05	0x40000450		// PS->PL 명령 (UART RingBuffer 설정)
#define BRAM_ADDR_SET_RB_UART_06	0x40000454		// PS->PL 명령 (UART RingBuffer 설정)

#define BRAM_ADDR_SET_UART_CH1		0x40000460		// PS->PL 명령 (UART CH1 설정)
#define BRAM_ADDR_SET_UART_CH2		0x40000480	  	// PS->PL 명령 (UART CH2 설정)
#define BRAM_ADDR_SET_UART_CH3		0x400004A0		// PS->PL 명령 (UART CH3 설정)
#define BRAM_ADDR_SET_UART_CH4		0x400004C0		// PS->PL 명령 (UART CH4 설정)
#define BRAM_ADDR_SET_UART_CH5		0x400004E0		// PS->PL 명령 (UART CH5 설정)
#define BRAM_ADDR_SET_UART_CH6		0x40000500		// PS->PL 명령 (UART CH6 설정)

/* --- 제어 상태 Register --- */
#define BRAM_ADDR_STS_SLOT_01		0x40000A00		// PL->PS 명령 (SLOT#01 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_02		0x40000A10		// PL->PS 명령 (SLOT#02 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_03		0x40000A20		// PL->PS 명령 (SLOT#03 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_04		0x40000A30		// PL->PS 명령 (SLOT#04 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_05		0x40000A40		// PL->PS 명령 (SLOT#05 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_06		0x40000A50		// PL->PS 명령 (SLOT#06 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_07		0x40000A60		// PL->PS 명령 (SLOT#07 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_08		0x40000A70		// PL->PS 명령 (SLOT#08 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_09		0x40000A80		// PL->PS 명령 (SLOT#09 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_10		0x40000A90		// PL->PS 명령 (SLOT#10 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_SLOT_IM		0x40000AA0		// PL->PS 명령 (UART#01 Status-Rolling Count, Write Index, Tx Status)

#define BRAM_ADDR_STS_UART_01		0x40000AB0		// PL->PS 명령 (UART#01 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_UART_02		0x40000AC0		// PL->PS 명령 (UART#02 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_UART_03		0x40000AD0		// PL->PS 명령 (UART#03 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_UART_04		0x40000AE0		// PL->PS 명령 (UART#04 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_UART_05		0x40000AF0		// PL->PS 명령 (UART#05 Status-Rolling Count, Write Index, Tx Status)
#define BRAM_ADDR_STS_UART_06		0x40000B00		// PL->PS 명령 (UART#06 Status-Rolling Count, Write Index, Tx Status)


/* --- SLOT TX Address --- */
#define BRAM_ADDR_WR_SLOT_01		0x40002000		// 0x1000, SLOT#01 출력
#define BRAM_ADDR_WR_SLOT_02		0x40004000		// 0x1000, SLOT#02 출력
#define BRAM_ADDR_WR_SLOT_03		0x40006000		// 0x1000, SLOT#03 출력
#define BRAM_ADDR_WR_SLOT_04		0x40008000		// 0x1000, SLOT#04 출력
#define BRAM_ADDR_WR_SLOT_05		0x4000A000		// 0x1000, SLOT#05 출력
#define BRAM_ADDR_WR_SLOT_06		0x4000C000		// 0x1000, SLOT#06 출력
#define BRAM_ADDR_WR_SLOT_07		0x4000E000		// 0x1000, SLOT#07 출력
#define BRAM_ADDR_WR_SLOT_08		0x40010000		// 0x1000, SLOT#08 출력
#define BRAM_ADDR_WR_SLOT_09		0x40012000		// 0x1000, SLOT#09 출력
#define BRAM_ADDR_WR_SLOT_10		0x40014000		// 0x1000, SLOT#10 출력

/* --- UART TX Address --- */
#define BRAM_ADDR_WR_UART_01		0x40016000		// 0x1000, UART 1채널 출력
#define BRAM_ADDR_WR_UART_02		0x40018000		// 0x1000, UART 2채널 출력
#define BRAM_ADDR_WR_UART_03		0x4001A000		// 0x1000, UART 3채널 출력
#define BRAM_ADDR_WR_UART_04		0x4001C000		// 0x1000, UART 4채널 출력
#define BRAM_ADDR_WR_UART_05		0x4001E000		// 0x1000, UART 5채널 출력
#define BRAM_ADDR_WR_UART_06		0x40020000		// 0x1000, UART 6채널 출력

/* --- SLOT 임무TX Address --- */
#define BRAM_ADDR_WE_IM				0x40022000		// 0x1000, 전방임무 출력

/* --- SLOT RX Address --- */
#define BRAM_ADDR_RE_SLOT_01		0x40040000		// 0x10000, LVDS1 수신
#define BRAM_ADDR_RE_SLOT_02		0x40060000		// 0x10000, LVDS2 수신
#define BRAM_ADDR_RE_SLOT_03		0x40080000		// 0x10000, LVDS3 수신
#define BRAM_ADDR_RE_SLOT_04		0x400A0000		// 0x1000, LVDS4 수신
#define BRAM_ADDR_RE_SLOT_05		0x400C0000		// 0x1000, LVDS5 수신
#define BRAM_ADDR_RE_SLOT_06		0x400E0000		// 0x2000, LVDS6 수신
#define BRAM_ADDR_RE_SLOT_07		0x40100000		// 0x20000, LVDS7 수신
#define BRAM_ADDR_RE_SLOT_08		0x40120000		// 0x1000, LVDS8 수신
#define BRAM_ADDR_RE_SLOT_09		0x40140000		// 0x1000, LVDS9 수신
#define BRAM_ADDR_RE_SLOT_10		0x40160000		// 0x1000, LVDS10 수신

/* --- UART RX Address --- */
#define BRAM_ADDR_RE_UART_01		0x40180000		// 0x2000, RS422 1채널 수신
#define BRAM_ADDR_RE_UART_02		0x401A0000		// 0x2000, RS422 2채널 수신
#define BRAM_ADDR_RE_UART_03		0x401C0000		// 0x2000, RS422 3채널 수신
#define BRAM_ADDR_RE_UART_04		0x401E0000		// 0x2000, RS422 4채널 수신
#define BRAM_ADDR_RE_UART_05		0x40200000		// 0x2000, RS422 5채널 수신
#define BRAM_ADDR_RE_UART_06		0x40220000		// 0x2000, RS422 6채널 수신

/* --- SLOT 임무RX Address --- */
#define BRAM_ADDR_RE_IM				0x40240000		// 0x1000, 전방임무 수신

/* --- PCM TX Address --- */
#define BRAM_ADDR_CTL_PCM			0x40243000		// 0x2000, PCM 설정


/*==============================================================================
 * LVDS 인터페이스 : PS <-> PL/Module Packet Header
 *============================================================================*/

//==================== Packet Header Structure ====================//
/* --- Ethernet Packet-Header --- */
typedef struct {
    /* Ethernet Structure */
    UInt8 ucEthPreamble[7];             // Preamble
    UInt8 ucEthSfd;                     // SFD

    UInt8 ucEthDesMac[6];               // Destination MAC
    UInt8 ucEthSrcMac[6];               // Source MAC

    UInt8 ucEthType[2];                 // Ethernet Type (IPv4 : 0x0800, ARP : 0x0806, VLAN : 0x8100)

    /* IP Structure */
    /* UDP Structure */
}__attribute__((packed)) sEthStructure;

/* --- IP Structure --- */
typedef struct {
    UInt8 bit_ucHeaderLen : 4;
    UInt8 bit_ucVersion : 4;

    UInt8 ucTypeOfServ;                 // Type Of Servcie
    UInt16 usTotalLen;                	// Total Length

    UInt8 ucId[2];                      // Total Length

    UInt16 bit_usFragmentOffset : 13;   // MSB
    UInt16 bit_usIpFlag : 3;            // LSB

    UInt8 ucTimeToLive;
    UInt8 ucProtocol;
    UInt16 usCheckSum;
    UInt8 ucSrcAddr[4];
    UInt8 ucDesAddr[4];
    // IP Option은 사용되지 않음
}__attribute__((packed)) sIpStructure;

/* --- UDP Structure --- */
typedef struct {
    UInt16 usSrcPort;
    UInt16 usDesPort;
    UInt16 usUdpLen;            // Header + Payload
    UInt16 usCheckSum;          // '0' 이면, 수신측 계산도 생략
}__attribute__((packed)) sUdpStructure;

/* 계측데이터 Packet - Head */
typedef struct {
    /* Ethernet Structure */
    sEthStructure stEthStructure;

    /* IP Structure */
    sIpStructure stIpStructure;

    /* UDP Structure */
    sUdpStructure stUdpStructure;
}__attribute__((packed)) sPacketHead;


/*==============================================================================
 * PS <-> PL Command
 *============================================================================*/
/* --- PL 제어 명령 구조체 --- */
typedef struct {
    UInt32 uiPlCmd;
}__attribute__((packed)) sPlCmd;		// PL 제어 명령


/*==============================================================================
 * PS <-> PL Configure
 *============================================================================*/

//====================== PCM Configure ======================//
/* --- PL Configure Message --- */
typedef struct {
    UInt16 usPcmClkMeas;					// PCM Output - System Clock 계측용
    UInt16 usFrameCntMeas;					// Minor Frame Count 계측용
    UInt32 uiSyncClkCycMeas;				// Sync Clock Cycle 계측용

    UInt16 usPcmClkOut;						// PCM Output - System Clock 출력용
    UInt16 usFrameCntOut;					// Minor Frame Count 출력용
    UInt32 uiSyncClkCycOut;					// Sync Clock Cycle 출력용
}__attribute__((packed)) sPlPcmConfMsg;		// PL PCM 설정 메시지


//====================== SLOT Configure ======================//
/* --- SLOT 설정 구조체 --- */
typedef struct {
    UInt8 ucCmd;
    UInt8 ucIdx;
    UInt16 usSize;
}__attribute__((packed)) sPlSlotConfMsg;	// SLOT 설정


typedef union {
    UInt32 uiSlotConf;
    sPlSlotConfMsg stPlSlotConfMsg;
} __attribute__((packed)) plSlotConf_t;


//====================== UART Configure ======================//
/* --- UART 설정 구조체 --- */
typedef struct {
    /* 기능설정 */
	UInt8 ucBaudRate;
    UInt8 ucCrcEnable;
    UInt8 ucEofEnable;
    UInt8 ucReserved_1;

    /* ESICD */
    UInt8 ucHeader;				// Header
    UInt8 ucReserved_2;
    UInt16 usReserved_3;

    UInt8 ucFrom;				// From
    UInt8 ucReserved_4;
    UInt16 usReserved_5;

    UInt8 ucTo;					// To
    UInt8 ucReserved_6;
    UInt16 usReserved_7;

    UInt16 usEof;				// EOF
    UInt16 usReserved_8;

}__attribute__((packed)) sUartConf;

typedef union {
	UInt8 ucBuf[20];
	sUartConf stUartConf;
} __attribute__((packed)) pUartConf_t;

/*==============================================================================
 * PS <-> PL/Module Status
 *============================================================================*/
/* --- 상태 구조체 --- */
typedef struct {
    /* 기능설정 */
	UInt8 ucRollingCnt;
    UInt8 ucWrIndex;
    UInt8 ucTxStatus;
    UInt8 ucReserved;
}__attribute__((packed)) sSlotStatus;



/*==============================================================================
 * LVDS 인터페이스 : PS <-> Network 모듈
 *============================================================================*/
//====================== Network 모듈  Configure ======================//
/* --- 프로토콜 타입 --- */
typedef enum {
	PROTOCOL_TCP,
	PROTOCOL_UDP
} ProtocolType;

/* --- 통신 방식 타입 --- */
typedef enum {
	COMMUNICATE_UNICAST,
	COMMUNICATE_MULTICAST,
	COMMUNICATE_BROADCAST
} CommunicationType;

/* --- 설정 구조체 --- */
typedef struct {
	SInt8 scModIpAddr[INET_ADDRSTRLEN];		// 모듈 IP 주소
	SInt8 scModSubNet[INET_ADDRSTRLEN];		// 모듈 Subnet Mask 주소
	SInt8 scModGateway[INET_ADDRSTRLEN];	// 모듈 Gateway 주소

	ProtocolType protocol;
	CommunicationType communication;
	SInt8 scDstIpAddr[INET_ADDRSTRLEN];
	UInt32 uiRecvPort;
	UInt32 uiSendPort;

	UInt8 ucLinkSts;
} __attribute__((packed)) sNetConfig;

typedef union {
	UInt8 ucRecvBuf[NETWORK_BRAM_SIZE];
	sNetConfig stNetConfig;
} __attribute__((packed)) plNetConfig_t;


//====================== Network 모듈  송수신 Packet ======================//
/* --- Network 모듈 송수신 Packet --- */
typedef struct {
	/* Packet Header */
	sPacketHead stPacketHead;

    UInt16 usTypeCmd;

    /* Data Buffer */
    UInt8 ucNetBuf[1470];

    /* Ethernet FCS */
    UInt32 uiFcs;
}__attribute__((packed)) sNetPacket;


typedef union {
    UInt8 ucMsgBuf[NETWORK_BRAM_SIZE];
    sNetPacket stNetPacket;
} __attribute__((packed)) plNetPacket_t;


//====================== 온도 정보 저장 구조체 ======================//
typedef struct {
	UInt16 usReserved0:4;
	UInt16 usData:12;
	UInt16 usReserved1;
}__attribute__((packed)) sZynqTemp;

typedef union {
    UInt32 uiData;
    sZynqTemp stZynqTemp;
} __attribute__((packed)) plZynqTemp_t;

/*==============================================================================
 * LVDS 인터페이스 : Processing 모듈 <-> GNS 모듈
 *============================================================================*/

typedef struct {
    /* Ethernet Structure */
    sEthStructure stEthStructure;

    /* IP Structure */
    sIpStructure stIpStructure;

    /* UDP Structure */
    sUdpStructure stUdpStructure;

    /* Data */
    UInt8 ucData[1500];
}__attribute__((packed)) sModGpsHead;

/*==============================================================================
 * LVDS 인터페이스 : Processing 모듈 <-> INO Encoder 모듈
 *============================================================================*/

//====================== Discrete 모듈  송수신 Packet ======================//
/* --- INO Encoder 모듈 패킷 구조 --- */
typedef struct {
	/* Packet Header */
    sPacketHead stPacketHead;

    /* Packet Payload */
    UInt8 ucDataBuf[2504];

    /* Ethernet FCS */
    UInt32 uiFcs;
}__attribute__((packed)) sInoEncPacket;


typedef union {
    UInt8 ucMsgBuf[2558];						// INO Data 2504 + Packet Header 54
    sInoEncPacket stInoEncPacket;
} __attribute__((packed)) plInoEncData_t;


typedef struct {
	UInt8 ucCycId;							// 주기 식별 정보
	UInt8 ucSendCnt;						// 송신 카운트
	UInt8 ucMeasData[PCM_RAWDATA_SIZE];		// 계측 데이터
	UInt16 ucCrc;							// CRC
}__attribute__((packed)) sMeasData;

typedef union {
    UInt8 ucMsgBuf[PCM_PACKET_SIZE];
    sMeasData stMeasData;
} __attribute__((packed)) plMeasMsg_t;


/*==============================================================================
 * Global Valuable
 *============================================================================*/
/* --- PS Mode --- */
extern UInt8 ucPsState;            // PS SW 모드 상태

/* --- Debug 변수 --- */
extern UInt16 usGpsFlag;			// [TEST] GPS 로그
extern UInt16 usImuFlag;			// [TEST] IMU 로그
extern UInt16 usUartFlag;			// [TEST] UART 로그



/*==============================================================================
 * Global Function
 *============================================================================*/
extern void gpioDriverInit(void);															// GPIO ???? ???
extern void gpioSetFunc(void);																// GPIO ?? (???? ?? ??)
extern void ByteSwap_2( SInt8 *cSource );													// 2bytes SWAP ??
extern void ByteSwap_4( SInt8 *cSource );													// 4bytes SWAP 함수
extern void PsToPlCommand( UInt32 uiCmd, UInt32 uiAddr );									// PS->PL 명령 함수
extern void BramRead( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );					// BRAM 1byte Read
extern void BramWrite( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );					// BRAM 1byte Write
extern void BramWrite16( UInt16 *pMsg, UInt16 usMsgLen, UInt32 uiIndexStart );				// BRAM 2bytes Write
extern void BramWrite32( UInt32 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );				// BRAM 4bytes Write
extern UInt16 CalcCKS16(UInt32 uiSumOffset, const UInt16 *uspData, UInt32 uiLength);		// 16bits CRC
extern UInt32 CalcCRC32(const UInt8 *ucpData, UInt32 uiLength);								// 32bits CRC

#endif 			//__COMMON_H__
