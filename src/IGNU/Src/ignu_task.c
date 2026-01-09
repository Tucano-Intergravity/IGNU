/**
 * @file ignu_task.c
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief IGNU Task Source File (GPS, IMU, TM/TC Processing)
 * @version 1.0.0
 * @date 2026-01-08
 *
 * @copyright Intergravity Technologies Copyright (c) 2026
 */

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "../Inc/ignu_task.h"
#include "../Inc/TMTC.h"
#include "xil_printf.h"

/*==============================================================================
 * Gloabal Variables
 *============================================================================*/
QueueHandle_t xImuDataQueue = NULL;
QueueHandle_t xGpsDataQueue = NULL;
QueueHandle_t xCom1DataQueue = NULL;

/*==============================================================================
 * Local Variables
 *============================================================================*/
static IgnuState_t eCurrentState = IGNU_STATE_IDLE;

/*==============================================================================
 * Local Function Declarations
 *============================================================================*/

/*==============================================================================
 * Functions
 *============================================================================*/

void SetIgnuState(IgnuState_t eState)
{
    eCurrentState = eState;
    xil_printf("[IGNU] State Changed: %s\r\n", (eState == IGNU_STATE_RUN) ? "RUN" : "IDLE");
}

IgnuState_t GetIgnuState(void)
{
    return eCurrentState;
}

/**
 * @fn IgnuTask
 * @brief IGNU Processing Task (GPS, IMU, TM/TC)
 * @param pvParameters Task parameters
 * @return void
 */
void IgnuTask( void *pvParameters )
{
    const TickType_t x10ms = pdMS_TO_TICKS( DELAY_10_MSECOND );
    sRbData stImuData; // Buffer for received IMU data
    sRbData stGpsData; // Buffer for received GPS data
    sRbData stCom1Data; // Buffer for received Com1 data

    /* Static buffer for decoded packet to save stack size */
    static UInt8 ucDecodedPacket[MAX_KISS_BUF]; 

    /* Initialization */
    static int iLoopCnt = 0; // For periodic status log

    /* Create IMU Data Queue (Size: 10, Item Size: size of sRbData structure) */
    xImuDataQueue = xQueueCreate( 10, sizeof(sRbData) );
    if( xImuDataQueue == NULL )
    {
        xil_printf("[IGNU] Failed to create IMU Queue!\r\n");
    }

    /* Create GPS Data Queue (Size: 10, Item Size: size of sRbData structure) */
    xGpsDataQueue = xQueueCreate( 10, sizeof(sRbData) );
    if( xGpsDataQueue == NULL )
    {
        xil_printf("[IGNU] Failed to create GPS Queue!\r\n");
    }

    /* Create COM1 Data Queue (Size: 10, Item Size: size of sRbData structure) */
    xCom1DataQueue = xQueueCreate( 10, sizeof(sRbData) );
    if( xCom1DataQueue == NULL )
    {
        xil_printf("[IGNU] Failed to create COM1 Queue!\r\n");
    }

    xil_printf("[IGNU] Task Started.\r\n");

    while(1)
    {
        /* 1. Receive COM1 Data (Queue) - Always process Commands */
        if( xCom1DataQueue != NULL )
        {
            /* Receive data from queue (Wait time 0 = Non-blocking) */
            if( xQueueReceive( xCom1DataQueue, &stCom1Data, 0 ) == pdTRUE )
            {
                /* Debug: Print Raw Data size */
                xil_printf("[IGNU] Raw Com1 (%d): ", stCom1Data.usSize);
                for(int i=0; i<stCom1Data.usSize; i++)
                {
                    xil_printf("%02X ", stCom1Data.ucData[i]);
                }
                xil_printf("\r\n");
                
                /* Process received Com1 data via KISS Decoder */
                SInt32 siDecodedLen;
                for(int i=0; i<stCom1Data.usSize; i++)
                {
                    /* Feed byte into state machine */
                    siDecodedLen = KissDecode(stCom1Data.ucData[i], ucDecodedPacket);
                    
                    /* If a packet is completed */
                    if( siDecodedLen > 0 )
                    {
                        TickType_t xCurrentTick = xTaskGetTickCount();
                        xil_printf("[%u] [IGNU] KISS Frame Decoded (Len: %d)\r\n", xCurrentTick, siDecodedLen);
                        
                        /* Pass to CSP Layer */
                        CspReceive(ucDecodedPacket, siDecodedLen);
                    }
                }
            }
        }

        /* 2. State Machine */
        switch (eCurrentState)
        {
        case IGNU_STATE_IDLE:
            /* Idle State: Drain Sensor Queues but don't process/send */
            if( xImuDataQueue != NULL ) xQueueReceive( xImuDataQueue, &stImuData, 0 );
            if( xGpsDataQueue != NULL ) xQueueReceive( xGpsDataQueue, &stGpsData, 0 );
            break;

        case IGNU_STATE_RUN:
            /* Run State: Process Sensor Data */
            
            /* Receive IMU Data (Queue) */
            if( xImuDataQueue != NULL )
            {
                /* Receive data from queue (Wait time 0 = Non-blocking) */
                if( xQueueReceive( xImuDataQueue, &stImuData, 0 ) == pdTRUE )
                {
                    /* TODO: Process received IMU data */
                    TickType_t xCurrentTick = xTaskGetTickCount();
                    // xil_printf("[%u] [IGNU] IMU Data Received! Size: %d\r\n", xCurrentTick, stImuData.usSize);
                }
            }

            /* Receive GPS Data (Queue) */
            if( xGpsDataQueue != NULL )
            {
                /* Receive data from queue (Wait time 0 = Non-blocking) */
                if( xQueueReceive( xGpsDataQueue, &stGpsData, 0 ) == pdTRUE )
                {
                    /* TODO: Process received GPS data */
                    TickType_t xCurrentTick = xTaskGetTickCount();
                    // xil_printf("[%u] [IGNU] GPS Data Received! Size: %d, Counter: %d\r\n", xCurrentTick, stGpsData.usSize, stGpsData.ucData[stGpsData.usSize-1]);
                }
            }
            break;
        }

        /* 3. Periodic Status Log (Every 1 sec) */
        iLoopCnt++;
        if (iLoopCnt >= 100)
        {
            iLoopCnt = 0;
            xil_printf("[IGNU] Status: %s\r\n", (eCurrentState == IGNU_STATE_RUN) ? "RUN" : "IDLE");
        }

        vTaskDelay( x10ms );
    }
}
