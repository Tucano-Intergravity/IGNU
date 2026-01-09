/**
 * @file siu_task.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0.0
 * @date 2025-11-04
 *
 * @copyright Danam Systems Copyright (c) 2025
 */

/*==============================================================================
 * Include Files
 *============================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "FreeRTOS.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xgpiops.h"
#include "xil_io.h"
#include "xuartps.h"
#include "xil_exception.h"
#include "xscugic.h"

#include "siu_task.h"
#include "../common/common.h"

/*==============================================================================
 * Gloabal Function
 *============================================================================*/

/*==============================================================================
 * Gloabal Variables
 *============================================================================*/

/*==============================================================================
 * Local Function
 *============================================================================*/
static void ModConfigWrite( void );
static void PlConfigWrite( void );


/*==============================================================================
 * Local Variables
 *============================================================================*/

/*==============================================================================
 * Functions
 *============================================================================*/


/**
 * @fn ConfigureSlot
 * @brief SLOT 설정 함수
 * @param ucSlotCmd 설정할 슬롯 명령
 * @param ucSlotIdx 설정할 슬롯 인덱스
 * @param usSlotSize 설정할 슬롯 크기
 * @param uiAddr 설정할 슬롯 주소
 * @return void
 * @date 2025-11-07
 */
static void ConfigureSlot( UInt8 ucSlotCmd, UInt8 ucSlotIdx, UInt16 usSlotSize, UInt32 uiAddr )
{
	plSlotConf_t stSlotConf;

    stSlotConf.stPlSlotConfMsg.ucCmd = ucSlotCmd;
    stSlotConf.stPlSlotConfMsg.ucIdx = (ucSlotIdx-1);
    stSlotConf.stPlSlotConfMsg.usSize = usSlotSize;
    PsToPlCommand( stSlotConf.uiSlotConf, uiAddr );
    usleep(TIME_REQ);               // 1ms wait
}


/**
 * @fn SetUartValue
 * @brief 디지털 모듈의 UART 설정을 구성하는 함수
 * @param uiSetAddr UART 설정 주소
 * @param ucBaudRate 설정할 Baud Rate 값
 * @return void
 * @date 2025-11-07
 */
void SetUartValue( UInt32 uiSetAddr, UInt8 ucBaudRate )
{
	pUartConf_t stUartConfData;
    memset( stUartConfData.ucBuf, 0x00, sizeof(sUartConf) );

    stUartConfData.stUartConf.ucBaudRate = ucBaudRate;
    stUartConfData.stUartConf.ucCrcEnable = 0;		// 0:Enable, 1:Disable
    stUartConfData.stUartConf.ucEofEnable = 0;		// 0:Enable, 1:Disable
    stUartConfData.stUartConf.ucHeader = 0;
    stUartConfData.stUartConf.ucFrom = 0;
    stUartConfData.stUartConf.ucTo = 0;
    stUartConfData.stUartConf.usEof = 0;

    BramWrite32( (UInt32 *)stUartConfData.ucBuf, sizeof(sUartConf), uiSetAddr );
    usleep(TIME_REQ);               // 1ms wait
}

/**
 * @fn PcmConfigWrite
 * @brief 엔코더 설정 함수
 * @param void
 * @return void
 * @date 2025-11-07
 */
static void PlConfigWrite( void )
{
    UInt8 ucWriteBuf[MSG_MAX];
    sPlPcmConfMsg stPlPcmConfMsg;                        // PL 설정 정보

    /* 변수 초기화 */
    memset( ucWriteBuf, 0x00, sizeof(ucWriteBuf) );
    memset( &stPlPcmConfMsg, 0x00, sizeof(sPlPcmConfMsg) );

    /* --- PCM 설정 --- */
    /* 계측 설정 */
    stPlPcmConfMsg.usPcmClkMeas = ENC_SPEED_512K;
    stPlPcmConfMsg.usFrameCntMeas = 0x00A0;			// Minor Frame Count 계측용 (160 Word)
    stPlPcmConfMsg.uiSyncClkCycMeas = 0x00000031;	// Sync Clock Cycle 계측용 (5ms = 50-1 => 0x31)

    /* PCM 출력 설정 */
    stPlPcmConfMsg.usPcmClkOut = ENC_SPEED_512K;     // PCM Output - System Clock 출력용
    stPlPcmConfMsg.usFrameCntOut = 0x0280;			// Minor Frame Count 출력용 (640 * 2 = 1280 bytes)
    stPlPcmConfMsg.uiSyncClkCycOut = 0x00000063;	// Sync Clock Cycle 계측용 (10ms = 100-1 => 0x63)

    /* 모듈 설정 정보 요청 메시지 BRAM Write */
    memcpy( ucWriteBuf, &stPlPcmConfMsg, sizeof(sPlPcmConfMsg) );
    BramWrite32( (UInt32 *)ucWriteBuf, sizeof(sPlPcmConfMsg), BRAM_ADDR_SET_PCM );

    /* --- SLOT 설정 --- */
    ConfigureSlot( CMD_SLOT_CONF_LVDS1, GPS_BRAM_PACKET, GPS_BRAM_SIZE, BRAM_ADDR_SET_RB_SLOT_01 );		// SLOT#1 설정
    ConfigureSlot( CMD_SLOT_CONF_LVDS2, IMU_BRAM_PACKET, IMU_BRAM_SIZE, BRAM_ADDR_SET_RB_SLOT_02 );		// SLOT#2 설정

    ConfigureSlot( CMD_SLOT_CONF_UART1, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_01 );		// RS422 #3 설정
    ConfigureSlot( CMD_SLOT_CONF_UART2, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_02 );		// RS422 #3 설정
    ConfigureSlot( CMD_SLOT_CONF_UART3, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_03 );		// RS422 #3 설정
    ConfigureSlot( CMD_SLOT_CONF_UART4, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_04 );		// RS422 #4 설정
    ConfigureSlot( CMD_SLOT_CONF_UART5, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_05 );		// RS422 #5 설정
    ConfigureSlot( CMD_SLOT_CONF_UART6, UART_BRAM_PACKET, UART_BRAM_SIZE, BRAM_ADDR_SET_RB_UART_06 );		// RS422 #6 설정


    /* 설정 대기 */
    usleep(TIME_REQ);               // 1ms wait

    /* --- UART 설정 --- */
    SetUartValue( BRAM_ADDR_SET_UART_CH1, UART_CONF_BAUDRATE_115200 );
    SetUartValue( BRAM_ADDR_SET_UART_CH2, UART_CONF_BAUDRATE_115200 );
    SetUartValue( BRAM_ADDR_SET_UART_CH3, UART_CONF_BAUDRATE_115200 );
    SetUartValue( BRAM_ADDR_SET_UART_CH4, UART_CONF_BAUDRATE_115200 );
    SetUartValue( BRAM_ADDR_SET_UART_CH5, UART_CONF_BAUDRATE_115200 );
    SetUartValue( BRAM_ADDR_SET_UART_CH6, UART_CONF_BAUDRATE_115200 );

    /* 설정 대기 */
    usleep(TIME_REQ);               // 1ms wait

    /* 모듈 설정 정보 요청 명령 BRAM Write */
    PsToPlCommand( CMD_PL_READY, BRAM_ADDR_CTL_PL );

    /* 수신 대기 */
    usleep(TIME_REQ);               // 1ms wait
}


/**
 * @fn SiuTask
 * @brief SIU Task 함수
 * @param pvParameters Task 매개변수
 * @return void
 * @date 2022-12-19
 */
void SiuTask( void *pvParameters )
{
#if TASK_STACK_SIZE_CHECK
    unsigned long uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
#endif
    const UInt32 x10ms = pdMS_TO_TICKS( DELAY_10_MSECOND );

    /* 초기화 모드 */
    ucPsState = PS_MODE_INIT;

    vTaskDelay( x10ms*100 );

    /* PL 설정 정보 write */
    PlConfigWrite();

#if TASK_STACK_SIZE_CHECK
    while(1)
    {
    	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    	xil_printf( "SIU Task : %d\r\n", uxHighWaterMark );
    	vTaskDelay( 100 );
    }
#endif

    /* 운용모드 전환 */
    ucPsState = PS_MODE_OP;

    vTaskDelete( NULL );
}
