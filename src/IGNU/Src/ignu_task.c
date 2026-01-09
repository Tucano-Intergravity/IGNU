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

/*==============================================================================
 * Local Function Declarations
 *============================================================================*/

/*==============================================================================
 * Functions
 *============================================================================*/

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

        /* Receive COM1 Data (Queue) */
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

        /* TODO: Implement GPS, IMU Data Processing */
        
        /* TODO: Implement TM/TC Processing */

        vTaskDelay( x10ms );
    }
}
