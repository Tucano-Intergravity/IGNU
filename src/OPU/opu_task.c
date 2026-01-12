/**
 * @file opu_task.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0.0
 * @date 2024-01-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */

/*==============================================================================
 * Include Files
 *============================================================================*/

/* --- FreeRTOS includes --- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "FreeRTOSconfig.h"

/* --- Xilinx includes --- */
#include "xil_printf.h"
#include "xparameters.h"
#include "xtime_l.h"
#include "xscugic.h"
#include "xuartps.h"

#include "xgpiops.h"
#include "xil_io.h"

/* --- User includes --- */
#include "opu_task.h"
#include "../common/common.h"
#include "../IGNU/Inc/ignu_task.h" // IMU 큐 핸들 참조

/*==============================================================================
 * Gloabal Function
 *============================================================================*/

void OpuTask( void *pvParameters );


/*==============================================================================
 * Gloabal Variables
 *============================================================================*/

/* --- 링버퍼  --- */
sRingBufInfo stGpsRbRx;						// GPS RX Ring Buffer 정보
sRingBufInfo stRbStim;						// IMU RX Ring Buffer 정보
sRingBufInfo stRbInfoUart[MAX_UART_CH];		// UART Channel 1~6 TX Ring Buffer 정보

UInt8 ucGpsRbRx[MAX_RB_IDX][MAX_RB_DATA];				// GPS RX 링버퍼
UInt8 ucImuRbRx[MAX_RB_IDX][MAX_RB_DATA];				// GPS RX 링버퍼
UInt8 ucRbUart[MAX_UART_CH][MAX_RB_IDX][MAX_RB_DATA];	// UART 링버퍼3

sRbData stGpsRbData;
sRbData stImuRbData;

/*==============================================================================
 * Local Variables
 *============================================================================*/

/* --- handler  --- */
static TaskHandle_t xUartTask;		// UART RX task handler
static TaskHandle_t xTxTask;		// UART TX task handler
static TaskHandle_t xGpsTask;		// GPS task handler
static TaskHandle_t xImuTask;		// IMU task handler

/* --- Semaphore  --- */
static xSemaphoreHandle xSemaphore = NULL;		// 20ms 동기 세마포어


/*==============================================================================
 * Local Function
 *============================================================================*/

/* --- 인터럽트  --- */
static UInt8 InitInterrupt( void );						// 인터럽트 초기화
static void ExtIrq_Handler(void *InstancePtr);			// 동기 신호 인터럽트 핸들러


/* --- 기능모듈 수신  --- */
static void ModuleDataRead( void );						// 항법 모듈 수신 데이터 Read
static void Slot1DataRead( UInt8 *pBramInfoData );		// SLOT#1 수신 데이터 Read
static void Slot2DataRead( UInt8 *pBramInfoData );		// SLOT#2 수신 데이터 Read
static void Slot3DataRead( UInt8 *pBramInfoData );		// SLOT#3 수신 데이터 Read
static void Slot4DataRead( UInt8 *pBramInfoData );		// SLOT#4 수신 데이터 Read
static void Slot5DataRead( UInt8 *pBramInfoData );		// SLOT#5 수신 데이터 Read
static void Slot6DataRead( UInt8 *pBramInfoData );		// SLOT#5 수신 데이터 Read
static void Slot7DataRead( UInt8 *pBramInfoData );		// SLOT#5 수신 데이터 Read

/* --- queue  --- */
static SInt32 DdrEnqueue( UInt32 *pBuf, sRingBufInfo *pRingBufInfo, UInt32 uiLen );		// Ring Buffer enqueue
static SInt32 DdrDequeue( sRbData *pRbData, sRingBufInfo *pRingBufInfo );				// Ring Buffer dequeue
static SInt32 SerialDequeue( sRbData *pRbData, sRingBufInfo *pRingBufInfo );			// Ring Buffer dequeue

/* --- 링버퍼  --- */
static void RingBufferInit( void );
static void DdrRingBufferInit( sRingBufInfo *pRingBufInfo );							// Ring Buffer 초기화

/* --- Processing 모듈  --- */
static void SemaphoreCreate( void );
static float GetZynqTemperature( void );														// 온도 수신 함수
static void UartRead( UInt32 uiComAddr, SInt8 *pWrAddrBefore, sRingBufInfo *pRingBufInfo );		// UART(RS422) Read 함수
static UInt8 UartBramRead( UInt32 uiChannel, UInt8 *pRecvBuf, SInt8 *pBramWrAddrBefore );		// UART(RS422) BRAM Read 함수
static void UartWrite( UInt32 uiChannel, UInt8 *pSendBuf, UInt32 uiSize );						// UART(RS422) Write 함수

/* --- Thread 함수  --- */
static void TaskCreate( void );
static void uart_thread(void *p);		// UART(RS422) 수신 Task (제어 주기 : 100Hz)
static void tx_thread(void *p);			// UART(RS422) 송신 Task (제어 주기 : 100Hz)

/*==============================================================================
 * Functions
 *============================================================================*/

/**
 * @fn GetZynqTemperature
 * @brief ZYNQ 온도 획득 함수
 * @param void
 * @return ZYNQ 온도(C)
 * @date 2024-03-04
 */
static float GetZynqTemperature( void )
{
	plZynqTemp_t stTemp;
	float fTemp = 0.0;
	volatile UInt32 *pAddr = (volatile UInt32 *)(XADC_BASE+0x200);
	stTemp.uiData = *pAddr;

	/* 온도 계산(ºC) */
	fTemp = ((stTemp.stZynqTemp.usData * 503.975)/4096) - 273.15;

	return fTemp;
}

/**
 * @fn		DdrEnqueue
 * @brief	DDR3 Ring Buffer write 함수
 * @param	UInt8 *pBuf : write 데이터 포인터
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	Ring Buffer 상태 (-1: Ring buffer is full, 1 : Normal)
 * @date	2023/02/03
 */
static SInt32 DdrEnqueue( UInt32 *pBuf, sRingBufInfo *pRingBufInfo, UInt32 uiLen )
{
	SInt32 ucSts = 0;				// -1: Ring buffer is full, 1 : Normal
	volatile UInt32 *pAddr = (volatile UInt32 *)pRingBufInfo->uiAddr;

	if( pRingBufInfo->siCount == MAX_RB_IDX )
	{
		/* Ring buffer is full */
		/* 가장오래된 데이터 삭제 */
		pRingBufInfo->siFront = (pRingBufInfo->siFront+1)%MAX_RB_IDX;
		pRingBufInfo->siCount--;
		ucSts = -1;
	}
	else
	{
		/* BRAM to DDR3 write */
		memcpy( &pAddr[pRingBufInfo->siRear*(MAX_RB_DATA/4)+1], pBuf, (uiLen+(4-uiLen%4)) );
		pAddr[pRingBufInfo->siRear*(MAX_RB_DATA/4)] = uiLen;
		pRingBufInfo->siRear = (pRingBufInfo->siRear+1)%MAX_RB_IDX;
		pRingBufInfo->siCount++;
	}

	return ucSts;
}

/**
 * @fn		DdrDequeue
 * @brief	DDR3 Ring Buffer Read 함수
 * @param	UInt8 *pBuf : Read 데이터 포인터
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	Ring Buffer 상태 (-1: Ring buffer is Empty, 0~ : Message Count)
 * @date	2023/02/03
 */
static SInt32 DdrDequeue( sRbData *pRbData, sRingBufInfo *pRingBufInfo )
{
	SInt32 ucSts;																// -1: Ring buffer is Empty, 0~ : Message Count
	volatile UInt8 *pAddr = (volatile UInt8 *)pRingBufInfo->uiAddr;

	UInt32 *pData = (UInt32 *)pAddr+pRingBufInfo->siFront*(MAX_RB_DATA/4);		// 4byte 데이터 포인터

	if( pRingBufInfo->siCount == 0 )
	{
		ucSts = -1;
	}
	else
	{
		/* 메시지 길이 확인 */

		pRbData->usSize = *pData;

		/* BRAM to DDR3 read */
		memcpy( pRbData->ucData, &pAddr[pRingBufInfo->siFront*MAX_RB_DATA+4], pRbData->usSize );
		pRingBufInfo->siFront = (pRingBufInfo->siFront+1)%MAX_RB_IDX;
		pRingBufInfo->siCount--;

		ucSts = pRingBufInfo->siCount;
	}
	return ucSts;
}

/**
 * @fn		DdrDequeue
 * @brief	DDR3 Ring Buffer Read 함수
 * @param	UInt8 *pBuf : Read 데이터 포인터
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	Ring Buffer 상태 (-1: Ring buffer is Empty, 0~ : Message Count)
 * @date	2023/02/03
 */
static SInt32 SerialDequeue( sRbData *pRbData, sRingBufInfo *pRingBufInfo )
{
	SInt32 ucSts;																// -1: Ring buffer is Empty, 0~ : Message Count
	volatile UInt8 *pAddr = (volatile UInt8 *)pRingBufInfo->uiAddr;

	UInt32 *pData = (UInt32 *)pAddr+pRingBufInfo->siFront*(MAX_RB_DATA/4);		// 4byte 데이터 포인터

	if( pRingBufInfo->siCount == 0 )
	{
		ucSts = -1;
	}
	else
	{
		/* 메시지 길이 확인 */

		pRbData->usSize = *pData;

		/* BRAM to DDR3 read */
		memcpy( pRbData->ucData, &pAddr[pRingBufInfo->siFront*MAX_RB_DATA+4], pRbData->usSize );
		pRingBufInfo->siFront = (pRingBufInfo->siFront+1)%MAX_RB_IDX;
		pRingBufInfo->siCount--;

		ucSts = pRingBufInfo->siCount;
	}
	return ucSts;
}


/**
 * @fn		DdrRingBufferInit
 * @brief	DDR3 Ring Buffer 초기화 함수
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	void
 * @date	2023/02/03
 */
static void DdrRingBufferInit( sRingBufInfo *pRingBufInfo )
{
	/* Ring Buffer 초기화 */
	pRingBufInfo->siFront = 0;
	pRingBufInfo->siRear = 0;
	pRingBufInfo->siCount = 0;
}


/**
 * @fn		GpsPacketRead
 * @brief	BRAM 데이터 DDR로 저장하는 함수
 * @param	UInt32 uiAddr : BRAM 시작 주소
 * @param	UInt8 *pBramWrIdxBefore : 이전 BRAM Write Index Count (0~255)
 * @param	UInt8 *pBramWrAddrBefore : 이전 BRAM Address Count (0~20)
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	BRAM 수신 데이터 Count
 * @date	2022/12/19
 */
static UInt8 GpsPacketRead( UInt32 uiAddr, UInt8 *pBramWrIdxBefore, UInt8 *pBramWrAddrBefore, UInt8 *pBramInfo )
{
	UInt32 i;
	SInt8 scSts;
	UInt8 ucRetVal = 0;

	UInt8 ucBramWrIdx;						// BRAM Write 정보 PL Write Index
	UInt8 ucBramWrAddr;						// BRAM Write 정보 PL Write Address

	UInt8 ucIdxRollCnt;						// Index 기준 쌓인 데이터 카운트 (현재 IDX - 이전 IDX)
	UInt8 ucAddrRollCnt;					// Address 기준 쌓인 데이터 카운트 (현재 Address - 이전 Address)

	UInt32 uiBramReAddr;					// BRAM 수신 Address
	sModGpsHead stModGpsHead;				// 모듈 GPS 헤더

	volatile UInt8 *pBramAddr = (volatile UInt8 *)(uiAddr);				// BRAM 시작 주소

	/* PL Write Index 및 Address 수신 */
	ucBramWrIdx = pBramInfo[1];				// PL Write Index Read (0~255)
	ucBramWrAddr = pBramInfo[0];			// PL Write Address Read (0~20)

	/* Rolling Count 확인 */
	ucIdxRollCnt = ((ucBramWrIdx-*pBramWrIdxBefore)<0)?(ucBramWrIdx-*pBramWrIdxBefore)+MAX_IDX:(ucBramWrIdx-*pBramWrIdxBefore);
	ucAddrRollCnt = ((ucBramWrAddr-*pBramWrAddrBefore)<0)?(ucBramWrAddr-*pBramWrAddrBefore)+GPS_BRAM_PACKET:(ucBramWrAddr-*pBramWrAddrBefore);

	if( ucIdxRollCnt != ucAddrRollCnt )
	{
		/* Buffer Overflow */
	}
	else
	{
		if( ucAddrRollCnt > 0 )
		{
			for( i=0; i<ucAddrRollCnt; i++ )
			{
				/* BRAM Address 확인 */
				uiBramReAddr = ((*pBramWrAddrBefore*GPS_BRAM_SIZE)+(i*GPS_BRAM_SIZE)) % (GPS_BRAM_PACKET*GPS_BRAM_SIZE);

				//memcpy( &stModGpsHead, (pBramAddr+uiBramReAddr), sizeof(sModGpsHead) );
				memcpy( &stModGpsHead, (pBramAddr+uiBramReAddr), GPS_BRAM_SIZE );

				/* DDR3 메모리 Enqueue */
				//scSts = DdrEnqueue( &pBramAddr[uiBramReAddr+48], &stGpsRbRx, (stModGpsHead.stIpStructure.usTotalLen-28) );
				scSts = DdrEnqueue( stModGpsHead.ucData, &stGpsRbRx, (stModGpsHead.stIpStructure.usTotalLen-28) );
				if( scSts < 0 )
				{
					/* ring buffer is full */
				}
			}

			/* 저장 Packet Return */
			ucRetVal = ucAddrRollCnt;
		}
		else
		{
			/* 수신 데이터 없음 */
		}
	}

	/* 이전 BRAM 정보 저장 */
	*pBramWrIdxBefore = ucBramWrIdx;
	*pBramWrAddrBefore = ucBramWrAddr;

	return ucRetVal;
}


/**
 * @fn		ImuPacketRead
 * @brief	BRAM 데이터 DDR로 저장하는 함수
 * @param	UInt32 uiAddr : BRAM 시작 주소
 * @param	UInt8 *pBramWrIdxBefore : 이전 BRAM Write Index Count (0~255)
 * @param	UInt8 *pBramWrAddrBefore : 이전 BRAM Address Count (0~20)
 * @param	sRingBufInfo *pRingBufInfo : Ring Buffer 정보
 * @return	BRAM 수신 데이터 Count
 * @date	2023/12/19
 */
static UInt8 ImuPacketRead( UInt32 uiAddr, UInt8 *pBramWrIdxBefore, UInt8 *pBramWrAddrBefore, UInt8 *pBramInfo )
{
	UInt32 i;
	SInt8 scSts;
	UInt8 ucRetVal = 0;

	UInt8 ucBramWrIdx;						// BRAM Write 정보 PL Write Index
	UInt8 ucBramWrAddr;						// BRAM Write 정보 PL Write Address

	UInt8 ucIdxRollCnt;						// Index 기준 쌓인 데이터 카운트 (현재 IDX - 이전 IDX)
	UInt8 ucAddrRollCnt;					// Address 기준 쌓인 데이터 카운트 (현재 Address - 이전 Address)

	UInt32 uiBramReAddr;					// BRAM 수신 Address
	sModGpsHead stModGpsHead;				// 모듈 GPS 헤더

	volatile UInt8 *pBramAddr = (volatile UInt8 *)(uiAddr);				// BRAM 시작 주소

	/* PL Write Index 및 Address 수신 */
	ucBramWrIdx = pBramInfo[1];				// PL Write Index Read (0~255)
	ucBramWrAddr = pBramInfo[0];			// PL Write Address Readd (0~20)

	/* Rolling Count 확인 */
	ucIdxRollCnt = ((ucBramWrIdx-*pBramWrIdxBefore)<0)?(ucBramWrIdx-*pBramWrIdxBefore)+MAX_IDX:(ucBramWrIdx-*pBramWrIdxBefore);
	ucAddrRollCnt = ((ucBramWrAddr-*pBramWrAddrBefore)<0)?(ucBramWrAddr-*pBramWrAddrBefore)+IMU_BRAM_PACKET:(ucBramWrAddr-*pBramWrAddrBefore);

	if( ucIdxRollCnt != ucAddrRollCnt )
	{
		/* Buffer Overflow */
	}
	else
	{
		if( ucAddrRollCnt > 0 )
		{
			for( i=0; i<ucAddrRollCnt; i++ )
			{
				/* BRAM Address 확인 */
				uiBramReAddr = ((*pBramWrAddrBefore*IMU_BRAM_SIZE)+(i*IMU_BRAM_SIZE)) % (IMU_BRAM_PACKET*IMU_BRAM_SIZE);

				memcpy( &stModGpsHead, (pBramAddr+uiBramReAddr), IMU_BRAM_SIZE );

				/* DDR3 메모리 Enqueue */
				//scSts = DdrEnqueue( &pBramAddr[uiBramReAddr+48], &stRbStim, (stModGpsHead.stIpStructure.usTotalLen-28) );
				scSts = DdrEnqueue( stModGpsHead.ucData, &stRbStim, (stModGpsHead.stIpStructure.usTotalLen-28) );
				if( scSts < 0 )
				{
					/* ring buffer is full */
				}
			}

			/* 저장 Packet Return */
			ucRetVal = ucAddrRollCnt;
		}
		else
		{
			/* 수신 데이터 없음 */
		}
	}

	/* 이전 BRAM 정보 저장 */
	*pBramWrIdxBefore = ucBramWrIdx;
	*pBramWrAddrBefore = ucBramWrAddr;

	return ucRetVal;
}


/**
 * @fn		Slot1DataRead
 * @brief	SLOT #1 데이터 Read 함수
 * @param	Read Enable
 * @return	void
 * @date	2025/11/07
 */
static void Slot1DataRead( UInt8 *pBramInfoData )
{
	static UInt8 ucBramWrIdxBefore = 0;			// 이전 BRAM Write 정보 PL Write Index
	static UInt8 ucBramWrAddrBefore = 0;		// 이전 BRAM Write 정보 PL Write Address

	/* 데이터 수신 - BRAM to DDR3 */
//	GpsPacketRead( BRAM_ADDR_RE_SLOT_01, &ucBramWrIdxBefore,
//			&ucBramWrAddrBefore, pBramInfoData );
	GpsPacketRead( BRAM_ADDR_RE_SLOT_01, &ucBramWrIdxBefore,
			&ucBramWrAddrBefore, pBramInfoData ); // by Chun 250108
}

/**
 * @fn		Slot2DataRead
 * @brief	SLOT #2 데이터 Read 함수
 * @param	Read Enable
 * @return	void
 * @date	2025/11/07
 */
static void Slot2DataRead( UInt8 *pBramInfoData )
{
	static UInt8 ucBramWrIdxBefore = 0;			// 이전 BRAM Write 정보 PL Write Index
	static UInt8 ucBramWrAddrBefore = 0;		// 이전 BRAM Write 정보 PL Write Address

	/* 데이터 수신 - BRAM to DDR3 */
	ImuPacketRead( BRAM_ADDR_RE_SLOT_02, &ucBramWrIdxBefore,
			&ucBramWrAddrBefore, pBramInfoData );
}


/**
 * @fn		ModuleDataRead
 * @brief	모듈 데이터 수신 함수 (SLOT #1~10 Read)
 * @param	void
 * @return	void
 * @date	2022/12/19
 */
static void ModuleDataRead( void )
{
	UInt32 uiBramInfoSolt1 = Xil_In32(BRAM_ADDR_RE_SLOT_01+65532);		// BRAM Write 정보 주소-64K
	UInt32 uiBramInfoSolt2 = Xil_In32(BRAM_ADDR_RE_SLOT_02+65532);

	/* SLOT #1 모듈 데이터 Read */
	Slot1DataRead( &uiBramInfoSolt1 );			// Network Module Data Read

	/* SLOT #2 모듈 데이터 Read */
	Slot2DataRead( &uiBramInfoSolt2 );			// GPS Module Data Read
}

/**
 * @fn ExtIrq_Handler
 * @brief 동기 신호 인터럽트 핸들러
 * @param InstancePtr 인터럽트 핸들러 매개변수
 * @return void
 * @date 2022-12-19
 */
static void ExtIrq_Handler(void *InstancePtr)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	/* 세마포어 릴리즈 */
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken == pdTRUE )
	{
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}


/**
 * @fn InitInterrupt
 * @brief 인터럽트 설정
 * @param void
 * @return UInt8
 * @date 2022-12-19
 */
static UInt8 InitInterrupt( void )
{
	UInt8 Status;
	extern XScuGic xInterruptController;

	/* 인터럽트 설정 */
	XScuGic_SetPriorityTriggerType( &xInterruptController, XPAR_FABRIC_LN_IRQ0_INTR,
			XPAR_FABRIC_IRQ_PRIORITY, XPAR_FABRIC_IRQ_RISING_EDGE );					// IRQ0 : 0x90(우선순위), Rising edge 설정

	/* 인터럽트 핸들러 설정 */
	Status = XScuGic_Connect( &xInterruptController, XPAR_FABRIC_LN_IRQ0_INTR,
			(Xil_ExceptionHandler)ExtIrq_Handler, (void *)NULL );
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* 인터럽트 Enable */
	XScuGic_Enable( &xInterruptController, XPAR_FABRIC_LN_IRQ0_INTR );

	return XST_SUCCESS;
}



/**
 * @fn		UartBramRead
 * @brief	BRAM to Receive Buffer 저장 함수
 * @param	UInt32 uiChannel : BRAM 주소
 * @param	UInt8 *pRecvBuf : Receive Buffer 포인터
 * @param	UInt8 *pBramWrAddrBefore : BRAM 이전 저장 Address
 * @return	수신 상태
 * @date	2023/03/03
 */
static UInt8 UartBramRead( UInt32 uiChannel, UInt8 *pRecvBuf, SInt8 *pBramWrAddrBefore )
{
	UInt8 i;
	UInt8 ucRetVal = 0;
	SInt8 scBramWrAddr;						// BRAM Write 정보 PL Write Address
	UInt8 ucAddrRollCnt;					// Address 기준 쌓인 데이터 카운트 (현재 Address - 이전 Address)
	UInt32 uiBramReAddr = 0;				// BRAM 수신 Address

	volatile UInt8 *pBramAddr = (volatile UInt8 *)(uiChannel);			// BRAM 시작 주소
	volatile UInt8 *pBramInfo = (volatile UInt8 *)(uiChannel+16380);		// BRAM Write 정보 주소

	/* BRAM 임계영역 설정 */
	if( pBramInfo[3] != PL_BRAM_WR_STS )
	{
		/* PL Write Index 및 Address 수신 */
		scBramWrAddr = pBramInfo[0];			// PL Write Address

		/* Rolling Count 확인 - 비주기 수신으로 Index 삭제 */
		ucAddrRollCnt = ((scBramWrAddr-*pBramWrAddrBefore)<0)?(scBramWrAddr-*pBramWrAddrBefore)+UART_BRAM_PACKET:(scBramWrAddr-*pBramWrAddrBefore);

		if( ucAddrRollCnt > 0 )
		{
			for( i=0; i<ucAddrRollCnt; i++ )
			{
				/* BRAM Address 확인 */
				uiBramReAddr = ((*pBramWrAddrBefore*UART_BRAM_SIZE)+((i+1)*UART_BRAM_SIZE)) % (UART_BRAM_PACKET*UART_BRAM_SIZE);

				/* RS422 데이터 저장 */
				memcpy( pRecvBuf, (void *)(pBramAddr+uiBramReAddr), UART_BRAM_SIZE );
			}

			/* 저장 Packet Return */
			ucRetVal = ucAddrRollCnt;

		}
		else
		{
			/* 수신 데이터 없음 */
		}

		/* 이전 BRAM 정보 저장 */
		*pBramWrAddrBefore = scBramWrAddr;
	}

	return ucRetVal;
}


/**
 * @fn		UartWrite
 * @brief	RS422 Data Send  함수
 * @param	UInt32 uiChannel : BRAM 주소
 * @param	UInt8 *pSendBuf : Send Buffer 포인터
 * @param	UInt32 uiSize : Send Buffer 크기
 * @return	void
 * @date	2023/03/03
 */
static void UartWrite( UInt32 uiChannel, UInt8 *pSendBuf, UInt32 uiSize )
{
	switch( uiChannel )
	{
		case BRAM_ADDR_WR_UART_01:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH01_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		case BRAM_ADDR_WR_UART_02:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH02_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		case BRAM_ADDR_WR_UART_03:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH03_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		case BRAM_ADDR_WR_UART_04:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH04_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		case BRAM_ADDR_WR_UART_05:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH05_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		case BRAM_ADDR_WR_UART_06:
			/* BRAM Write */
			BramWrite16( pSendBuf, uiSize, uiChannel );

			/* RS422 출력 명령 */
			PsToPlCommand( CMD_RS422_CH06_TX_ENABLE, BRAM_ADDR_CTL_UART_TX );
			break;
		default:
			/* RS422 write 주소 입력 오류 */
			break;
	}
}


/**
 * @fn UartRead
 * @brief UART Read 함수
 * @param void *p
 * @return void
 * @date 2024-07-26
 */
static void UartRead( UInt32 uiComAddr, SInt8 *pWrAddrBefore, sRingBufInfo *pRingBufInfo )
{
	UInt8 ucSts;							// 수신 상태 ( 0:수신 데이터 없음, 1:수신 데이터 있음)
	SInt32 scSts;
	plSerialPacket_t stSerialPacket;

	/* --- BRAM Read --- */
	ucSts = UartBramRead( uiComAddr, stSerialPacket.ucMsgBuf, pWrAddrBefore );
	if( ucSts > 0 )
	{
		/* Debug: Print received data from Com1 */
		if( uiComAddr == BRAM_ADDR_RE_UART_01 )
		{
			// xil_printf("[COM1] Recv(%d): ", stSerialPacket.stSerialRecvMsg.uiBufSize);
			// for(int i=0; i<stSerialPacket.stSerialRecvMsg.uiBufSize; i++)
			// {
			// 	xil_printf("%c", stSerialPacket.stSerialRecvMsg.ucRecvBuf[i]);
			// }
			// xil_printf("\r\n");
			
			/* Send Data to IGNU Task (Queue) */
			if( xCom1DataQueue != NULL )
			{
				sRbData stCom1RbData;
				stCom1RbData.usSize = stSerialPacket.stSerialRecvMsg.uiBufSize;
				if(stCom1RbData.usSize > MAX_RB_DATA) stCom1RbData.usSize = MAX_RB_DATA;
				
				memcpy(stCom1RbData.ucData, stSerialPacket.stSerialRecvMsg.ucRecvBuf, stCom1RbData.usSize);
				
				xQueueSend( xCom1DataQueue, &stCom1RbData, 0 );
			}
            else
            {
                xil_printf("[OPU] Error: xCom1DataQueue is NULL! Data Lost (%d bytes)\r\n", stSerialPacket.stSerialRecvMsg.uiBufSize);
            }
		}

#if 0
		printf( "recv : " );
		for( int i=0; i<stSerialPacket.stSerialRecvMsg.uiBufSize; i++ )
		{
			printf( "%02X ", stSerialPacket.stSerialRecvMsg.ucRecvBuf[i] );
		}
		printf( "\n" );
#endif
		/* 수락시험용-loopback */
		if( usUartFlag == 1 )
		{
			/* DDR3 메모리 Enqueue */
			if( stSerialPacket.stSerialRecvMsg.uiBufSize > 0 )
			{
				scSts = DdrEnqueue( stSerialPacket.stSerialRecvMsg.ucRecvBuf, pRingBufInfo, stSerialPacket.stSerialRecvMsg.uiBufSize );
				if( scSts < 0 )
				{
					/* ring buffer is full */
				}
			}
		}
	}
	else
	{
		/* 수신데이터 없음 */
	}
}


/**
 * @fn novatel_thread
 * @brief Novatel OEM7600 #1 처리 Thread
 * @param void *p
 * @return void
 * @date 2023-02-27
 */
static void novatel_thread(void *p)
{
	const TickType_t x5ms = pdMS_TO_TICKS( DELAY_1_MSECOND );
	sRbData stRbData;

	while(1)
	{
		/* Dequeue */
		if( DdrDequeue( &stRbData, &stGpsRbRx ) < 0 )
		{
			/* Empty */
			break;
		}
		else
		{
			if( usGpsFlag == 1 )
			{
				printf( "gps %d : %02X %02X %02X\n", stRbData.usSize, stRbData.ucData[0], stRbData.ucData[1], stRbData.ucData[2] );
			}

			/* 유저 영역 */
		}

		vTaskDelay( x5ms );
	}
}


/**
 * @fn uart_thread
 * @brief UART COM1~6 port 처리 Thread
 * @param void *p
 * @return void
 * @date 2024-07-26
 */
static void uart_thread(void *p)
{
	const TickType_t x5ms = pdMS_TO_TICKS( DELAY_5_MSECOND );

	SInt8 scBramWrAddrBeforeCom1 = 0;			// 이전 BRAM Write 정보 PL Write Address
	SInt8 scBramWrAddrBeforeCom2 = 0;			// 이전 BRAM Write 정보 PL Write Address
	SInt8 scBramWrAddrBeforeCom3 = 0;			// 이전 BRAM Write 정보 PL Write Address
	SInt8 scBramWrAddrBeforeCom4 = 0;			// 이전 BRAM Write 정보 PL Write Address
	SInt8 scBramWrAddrBeforeCom5 = 0;			// 이전 BRAM Write 정보 PL Write Address
	SInt8 scBramWrAddrBeforeCom6 = 0;			// 이전 BRAM Write 정보 PL Write Address

	while(1)
	{
		/* --- UART COM#01-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_01, &scBramWrAddrBeforeCom1, &stRbInfoUart[0] );

		/* --- UART COM#02-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_02, &scBramWrAddrBeforeCom2, &stRbInfoUart[1] );

		/* --- UART COM#03-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_03, &scBramWrAddrBeforeCom3, &stRbInfoUart[2] );

		/* --- UART COM#04-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_04, &scBramWrAddrBeforeCom4, &stRbInfoUart[3] );

		/* --- UART COM#05-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_05, &scBramWrAddrBeforeCom5, &stRbInfoUart[4] );

		/* --- UART COM#06-BRAM Read --- */
		UartRead( BRAM_ADDR_RE_UART_06, &scBramWrAddrBeforeCom6, &stRbInfoUart[5] );

		vTaskDelay( x5ms );
	}
}

/**
 * @fn tx_thread
 * @brief RS422 TX 처리 Thread
 * @param void *p
 * @return void
 * @date 2023-04-16
 */
static void tx_thread(void *p)
{
	const TickType_t x5ms = pdMS_TO_TICKS( DELAY_5_MSECOND );
	sRbData stRbData;

	volatile UInt8 *pUartStsCom1 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_01);		// BRAM 시작 주소
	volatile UInt8 *pUartStsCom2 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_02);		// BRAM 시작 주소
	volatile UInt8 *pUartStsCom3 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_03);		// BRAM 시작 주소
	volatile UInt8 *pUartStsCom4 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_04);		// BRAM 시작 주소
	volatile UInt8 *pUartStsCom5 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_05);		// BRAM 시작 주소
	volatile UInt8 *pUartStsCom6 = (volatile UInt8 *)(BRAM_ADDR_STS_UART_06);		// BRAM 시작 주소

	while(1)
	{
		/* RS422 CH1 Write 처리 */
		if( pUartStsCom1[2] == 0 )
		{
			/* RS422 CH1 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[0] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_01, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}

		/* RS422 CH2 Write 처리 */
		if( pUartStsCom2[2] == 0 )
		{
			/* RS422 CH2 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[1] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_02, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}

		/* RS422 CH3 Write 처리 */
		if( pUartStsCom3[2] == 0 )
		{
			/* RS422 CH3 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[2] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_03, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}

		/* RS422 CH4 Write 처리 */
		if( pUartStsCom4[2] == 0 )
		{
			/* RS422 CH4 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[3] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_04, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}

		/* RS422 CH5 Write 처리 */
		if( pUartStsCom5[2] == 0 )
		{
			/* RS422 CH5 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[4] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_05, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}

		/* RS422 CH6 Write 처리 */
		if( pUartStsCom6[2] == 0 )
		{
			/* RS422 CH6 Dequeue */
			if( SerialDequeue( &stRbData, &stRbInfoUart[5] ) < 0 )
			{
				/* Empty */
			}
			else
			{
				/* RS422 Write */
				UartWrite( BRAM_ADDR_WR_UART_06, &stRbData.usSize, (stRbData.usSize+4) );
			}
		}


		vTaskDelay( x5ms );
	}
}


/**
 * @fn gps_thread
 * @brief GPS Thread
 * @param void *p
 * @return void
 * @date 2025-07-21
 */
void gps_thread(void *p)
{
	const TickType_t x1ms = pdMS_TO_TICKS( DELAY_1_MSECOND );

	while(1)
	{
		/* GPS QUEUE확인 후 처리 */
		while(1)
		{
			if( DdrDequeue( &stGpsRbData, &stGpsRbRx ) < 0 )
			{
				/* Empty */
				break;
			}
			else
			{
				/* 사용자 함수 */
				if( usGpsFlag == 1 )
				{
					xil_printf( "GPS recv(%d) : ", stGpsRbData.usSize );

					for( int i=0; i<stGpsRbData.usSize; i++ )
					{
						xil_printf( "%02X ", stGpsRbData.ucData[i] );
					}
					xil_printf( "Counter: %d\r\n", stGpsRbData.ucData[stGpsRbData.usSize-1] );
				}

				/* Send Data to IGNU Task (Queue) */
				if( xGpsDataQueue != NULL )
				{
					/* Send data to queue (Wait time 0 = Non-blocking) */
					xQueueSend( xGpsDataQueue, &stGpsRbData, 0 );
				}
			}
		}

		vTaskDelay( x1ms );
	}
}


/**
 * @fn imu_thread
 * @brief IMU Thread
 * @param void *p
 * @return void
 * @date 2025-07-21
 */
void imu_thread(void *p)
{
	const TickType_t x1ms = pdMS_TO_TICKS( DELAY_1_MSECOND );
	static UInt32 uiImuSendCnt = 0;

	while(1)
	{
		/* STIM300 QUEUE확인 후 처리 */
		while(1)
		{
			if( DdrDequeue( &stImuRbData, &stRbStim ) < 0 )
			{
				/* Empty */
				break;
			}
			else
			{
				/* 사용자 함수 */
				if( usImuFlag == 1 )
				{

					printf( "IMU recv(%d) : ", stImuRbData.usSize );

					for( int i=0; i<stImuRbData.usSize; i++ )
					{
						printf( "%02X ", stImuRbData.ucData[i] );
					}
					printf( "\n" );
				}

				/* IGNU Task로 데이터 전송 (Queue) */
				if( xImuDataQueue != NULL )
				{
					uiImuSendCnt++;
					if( (uiImuSendCnt % 10) == 0 )
					{
						/* 큐로 데이터 전송 (Full 상태면 즉시 리턴, 0 = Non-blocking) */
						/* 데이터 유실을 막으려면 타임아웃을 0보다 크게 설정 가능 */
						xQueueSend( xImuDataQueue, &stImuRbData, 0 );
					}
				}
			}
		}

		vTaskDelay( x1ms );
	}
}


/**
 * @fn RingBufferInit
 * @brief 링버퍼 초기화 함수
 * @param void
 * @return void
 * @date 2024-07-29
 */
static void RingBufferInit( void )
{
	UInt32 i;

	/* UART 링버퍼 초기화 */
	for( i=0; i<MAX_UART_CH; i++ )
	{
		DdrRingBufferInit( &stRbInfoUart[i] );
		stRbInfoUart[i].uiAddr =  &ucRbUart[i][0][0];
	}

	/* GPS 링버퍼 초기화 */
	DdrRingBufferInit( &stGpsRbRx );
	stGpsRbRx.uiAddr =  ucGpsRbRx;

	/* IMU 링버퍼 초기화 */
	DdrRingBufferInit( &stRbStim );
	stRbStim.uiAddr =  ucImuRbRx;
}


/**
 * @fn OpuTaskCreate
 * @brief Task 생성 함수
 * @param void
 * @return void
 * @date 2024-07-29
 */
static void TaskCreate( void )
{
	/* --- UART Task --- */
	xTaskCreate( uart_thread, "uart_thread", SCDAU_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xUartTask );		// UART RX Task 생성
	xTaskCreate( tx_thread, "tx_thread", SCDAU_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, &xTxTask );				// UART TX Task 생성
	xTaskCreate( gps_thread, "gps_thread", SCDAU_STACK_SIZE*16, NULL, tskIDLE_PRIORITY + 2, &xGpsTask );
	xTaskCreate( imu_thread, "imu_thread", SCDAU_STACK_SIZE*8, NULL, tskIDLE_PRIORITY + 2, &xImuTask );
}

/**
 * @fn OpuSemaphoreCreate
 * @brief Semaphore 생성 함수
 * @param void
 * @return void
 * @date 2024-07-29
 */
static void SemaphoreCreate( void )
{
	/* Semaphore 생성 */
	xSemaphore = xSemaphoreCreateBinary();			// 동기 신호 세마포어
}


/**
 * @fn OpuTask
 * @brief OPU Task 함수
 * @param pvParameters Task 매개변수
 * @return void
 * @date 2022-12-19
 */
void OpuTask( void *pvParameters )
{
	const TickType_t x10ms = pdMS_TO_TICKS( DELAY_10_MSECOND );
#if TASK_STACK_SIZE_CHECK
    unsigned long uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
#endif
    /* 메인 주기 카운트 */
	UInt16 usMainCnt = 0;

    /* 네트워크 설정 구조체 초기화 (Reduced delay from 300ms to 10ms for fast startup) */
    vTaskDelay( x10ms );

	/* 인터럽트 초기화 */
	InitInterrupt();

	/* 링버퍼 초기화 */
	RingBufferInit();

	/* Task 생성 */
	TaskCreate();

	/* 세마포어 생성 */
	SemaphoreCreate();

    while(1)
    {
#if TASK_STACK_SIZE_CHECK
    	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    	xil_printf( "OPU Task : %d\r\n", uxHighWaterMark );
    	vTaskDelay( 100 );
#else
       	if( xSemaphoreTake( xSemaphore, 0xffff ) == pdTRUE )
      	{
			/* Data Read */
			ModuleDataRead();

			/* 메인 주기 관리 */
			usMainCnt++;
       	}
#endif
    }

    vTaskDelete( NULL );
}

/**
 * @fn SendToCom1
 * @brief Send data to Com1 (RS-422 Ch1) via Ring Buffer
 * @param pData Data pointer
 * @param uiLen Data length
 * @return SInt32 0 on success, -1 on failure (Ring Buffer Full)
 */
SInt32 SendToCom1(UInt8 *pData, UInt32 uiLen)
{
    SInt32 scSts;
    UInt32 uiAlignLen = uiLen;
    UInt32 uiBuf[MAX_RB_DATA/4]; // Temporary buffer for 4-byte alignment

    /* DdrEnqueue expects 4-byte aligned buffer (UInt32*)? 
       Checking DdrEnqueue implementation:
       memcpy( &pAddr[...], pBuf, ... );
       It takes UInt32* pBuf. So we should copy byte data to aligned buffer first. 
       Or cast it if we are sure? No, pData is UInt8*.
    */
    
    if(uiLen > MAX_RB_DATA) return -1;

    /* Copy to temp buffer to ensure alignment and match DdrEnqueue signature */
    memcpy(uiBuf, pData, uiLen);

    /* Enqueue to Com1 TX Ring Buffer (stRbInfoUart[0]) */
    scSts = DdrEnqueue(uiBuf, &stRbInfoUart[0], uiLen);
    
    return scSts;
}
