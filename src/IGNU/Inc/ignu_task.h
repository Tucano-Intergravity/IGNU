/**
 * @file ignu_task.h
 * @author Sebum Chun (sebum.chun@intergravity.tech)
 * @brief IGNU Task Header File
 * @version 1.0.0
 * @date 2026-01-08
 *
 * @copyright Intergravity Technologies Copyright (c) 2026
 */

#ifndef __IGNU_TASK_H__
#define __IGNU_TASK_H__

/*==============================================================================
 * Include Files
 *============================================================================*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../../common/common.h"
#include "../../OPU/opu_task.h" // For sRbData

/*==============================================================================
 * Gloabal Variables Extern
 *============================================================================*/
extern QueueHandle_t xImuDataQueue;
extern QueueHandle_t xGpsDataQueue;
extern QueueHandle_t xCom1DataQueue;

/*==============================================================================
 * Type Definition
 *============================================================================*/
typedef enum {
    IGNU_STATE_IDLE,
    IGNU_STATE_RUN
} IgnuState_t;

/*==============================================================================
 * Gloabal Function Declarations
 *============================================================================*/
void IgnuAppInit(void);
void IgnuTask( void *pvParameters );
void TxTask( void *pvParameters );
void SetIgnuState(IgnuState_t eState);
IgnuState_t GetIgnuState(void);

#endif /* __IGNU_TASK_H__ */
