/**
 * @file ignu_task.c
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief IGNU Task Source File (GPS, IMU, TM/TC Processing)
 * @version 1.2.0 (Split Tx Task for Precise Timing)
 * @date 2026-01-09
 *
 * @copyright Intergravity Technologies Copyright (c) 2026
 */

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "../Inc/ignu_task.h"
#include "../Inc/TMTC.h"
#include "../Inc/ins_gps.h"
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
 * Functions
 *============================================================================*/

/**
 * @brief Initialize IGNU Queues and Resources
 * Should be called from main() before scheduler starts
 */
void IgnuAppInit(void)
{
    /* Create IMU Data Queue (Reduced Size: 2 to save Heap) */
    if (xImuDataQueue == NULL) {
        xImuDataQueue = xQueueCreate( 2, sizeof(sRbData) );
    }

    /* Create GPS Data Queue (Reduced Size: 2) */
    if (xGpsDataQueue == NULL) {
        xGpsDataQueue = xQueueCreate( 2, sizeof(sRbData) );
    }

    /* Create COM1 Data Queue (Reduced Size: 4) */
    if (xCom1DataQueue == NULL) {
        xCom1DataQueue = xQueueCreate( 4, sizeof(sRbData) );
    }
    
    xil_printf("[IGNU] Queues Initialized.\r\n");
}

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
 * @fn TxTask
 * @brief Periodic Telemetry Transmission Task (1Hz)
 * @param pvParameters Task parameters
 * @return void
 */
void TxTask( void *pvParameters )
{
    const TickType_t x1000ms = pdMS_TO_TICKS( 1000 ); // 1Hz
    TickType_t xLastWakeTime;

    xil_printf("[IGNU] TxTask Started.\r\n");

    /* Initialize xLastWakeTime for vTaskDelayUntil */
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        /* Wait for 1 second */
        vTaskDelayUntil( &xLastWakeTime, x1000ms );

        /* Check State */
        if (eCurrentState == IGNU_STATE_RUN) {
            SendTestData();
        }
    }
}

/**
 * @fn IgnuTask
 * @brief IGNU Processing Task (GPS, IMU, TM/TC Reception)
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

    /* Note: Queues are now initialized in IgnuAppInit() called from main */
    /* Safety check in case Init wasn't called */
    if (xImuDataQueue == NULL || xGpsDataQueue == NULL || xCom1DataQueue == NULL) {
        IgnuAppInit();
    }

    xil_printf("[IGNU] RxTask Started.\r\n");

    while(1)
    {
        /* 1. Receive COM1 Data (Queue) - Always process Commands */
        if( xCom1DataQueue != NULL )
        {
            /* Receive data from queue (Wait time 0 = Non-blocking) */
            if( xQueueReceive( xCom1DataQueue, &stCom1Data, 0 ) == pdTRUE )
            {
                /* Debug: Print Raw Data size */
                /*
                xil_printf("[IGNU] Raw Com1 (%d): ", stCom1Data.usSize);
                for(int i=0; i<stCom1Data.usSize; i++)
                {
                    xil_printf("%02X ", stCom1Data.ucData[i]);
                }
                xil_printf("\r\n");
                */
                
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
                    /* Extract 42 bytes (1 IMU Packet) from the received data */
                    if (stImuData.usSize >= 42)
                    {
                        UInt8 ucImuPacket[42];
                        memcpy(ucImuPacket, stImuData.ucData, 42);

                        /* Check Identifier (Fixed: 0xA5) */
                        if (ucImuPacket[0] != 0xA5) {
                            xil_printf("[IGNU] IMU Sync Error! Byte0: 0x%02X (Expected 0xA5)\r\n", ucImuPacket[0]);
                        }
                        else {
                            ImuData_t stDecodedImu;
                            ProcessImuPacket(ucImuPacket, &stDecodedImu);
                            
                            /* Update Global IMU Data */
                            SetImuData(&stDecodedImu);

                            /* Debug Log Reduced */
                            /*
                            TickType_t xCurrentTick = xTaskGetTickCount();
                            xil_printf("[%u] [IMU] Acc(g): X=", xCurrentTick);
                            PrintFloat(stDecodedImu.fAccX);
                            xil_printf(" Y=");
                            PrintFloat(stDecodedImu.fAccY);
                            xil_printf(" Z=");
                            PrintFloat(stDecodedImu.fAccZ);
                            
                            xil_printf(" | Gyro(dps): X=");
                            PrintFloat(stDecodedImu.fGyroX);
                            xil_printf(" Y=");
                            PrintFloat(stDecodedImu.fGyroY);
                            xil_printf(" Z=");
                            PrintFloat(stDecodedImu.fGyroZ);
                            xil_printf("\r\n");
                            */
                        }
                    }
                }
            }

            /* Receive GPS Data (Queue) */
            if( xGpsDataQueue != NULL )
            {
                /* Receive data from queue (Wait time 0 = Non-blocking) */
                if( xQueueReceive( xGpsDataQueue, &stGpsData, 0 ) == pdTRUE )
                {
                    /* Extract 90 bytes (1 GPS Packet) from the received data */
                    /* Note: Assuming stGpsData.usSize holds the packet size */
                    if (stGpsData.usSize >= 90)
                    {
                        UInt8 ucGpsPacket[90];
                        memcpy(ucGpsPacket, stGpsData.ucData, 90);
                        
                        GpsData_t stDecodedGps;
                        memset(&stDecodedGps, 0, sizeof(GpsData_t));
                        if (ParseGpsPacket(ucGpsPacket, &stDecodedGps) == 0)
                        {
                            /* Update Global GPS Data */
                            SetGpsData(&stDecodedGps);


                            TickType_t xCurrentTick = xTaskGetTickCount();
                            
                            /* Debug: Print Raw Hex for Lat/Lon to verify data */
                            /*
                            xil_printf("[%u] [GPS Raw] Lat: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
                                xCurrentTick,
                                ucGpsPacket[10], ucGpsPacket[11], ucGpsPacket[12], ucGpsPacket[13],
                                ucGpsPacket[14], ucGpsPacket[15], ucGpsPacket[16], ucGpsPacket[17]);
                            */

                            /* Debug: Dump Full Packet (Offset 0-89) for thorough check */
                            /*
                            xil_printf("[GPS Dump] Sync:%02X%02X TOW:%02X%02X%02X%02X Mode:%02X Err:%02X\r\n",
                                ucGpsPacket[0], ucGpsPacket[1], 
                                ucGpsPacket[2], ucGpsPacket[3], ucGpsPacket[4], ucGpsPacket[5],
                                ucGpsPacket[8], ucGpsPacket[9]);
                            */

                            xil_printf("[%u] [GPS] TOW: %u Lat:", xCurrentTick, stDecodedGps.tow);
                            PrintDouble(stDecodedGps.latitude);
                            xil_printf(" Lon:");
                            PrintDouble(stDecodedGps.longitude);
                            xil_printf(" NrSV: %u\r\n", stDecodedGps.nrSv);
                        }
                        else
                        {
                            xil_printf("[IGNU] GPS Sync/Parse Error!\r\n");
                        }
                    }
                }
            }
            break;
        }

        vTaskDelay( x10ms );
    }
}
