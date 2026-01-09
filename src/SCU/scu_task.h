
/**
 * @file rcu_task.h
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2022-11-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */
#ifndef __RCUTASK_H__
#define __RCUTASK_H__


/***********************************************************
					Include Files
***********************************************************/

#include "errno.h"				// 에러 코드 및 에러 처리 관련 헤더 파일
#include "../common/common.h"	// 사용자 정의 공통 함수 및 매크로 관련 헤더 파일


/***********************************************************
					Define
***********************************************************/

#define DEFAULT_IP_ADDRESS	"192.168.1.10"			// 기본 IP 주소
#define DEFAULT_IP_MASK		"255.255.255.0"			// 기본 Netmask 주소
#define DEFAULT_GW_ADDRESS	"192.168.1.1"			// 기본 Gateway 주소

/* platform_config.h */
#define PLATFORM_EMAC_BASEADDR XPAR_XEMACPS_0_BASEADDR
#define PLATFORM_EMAC_BASEADDR2 XPAR_XEMACPS_1_BASEADDR


/***********************************************************
					Global Function
***********************************************************/

/* SCU Task */
extern void ScuTask( void *pvParameters );


extern int sock_send;				// send sock
extern int sock_recv;				// Recv sock
extern struct netif server_netif;	// netif

#endif //__RCUTASK_H__


