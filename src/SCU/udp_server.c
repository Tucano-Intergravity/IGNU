/**
 * @file udp_perf_server.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0.0
 * @date 2023-02-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */

/***********************************************************
					Include Files
***********************************************************/

/* Driver includes */
#include "xqspips.h"			// QSPI device driver
#include "xscugic.h"			// Interrupt controller device driver

#include "xparameters.h"		// Xilinx 디바이스 및 매크로 관련 헤더 파일
#include "scu_task.h"			// SCU 태스크 관련 헤더 파일
#include "udp_server.h"	// UDP 성능 서버 관련 헤더 파일

#include "../opu/opu_task.h"	// 사용자 정의 OPU 태스크 관련 헤더 파일
#include "../common/common.h"	// 사용자 정의 공통 함수 및 매크로 관련 헤더 파일

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/igmp.h"
#include "lwip/ip_addr.h"


/***********************************************************
					Gloabal Variables
***********************************************************/

struct udp_pcb *pcb;				// pcb 포인터
struct perf_stats server;			// 서버 상태

/***********************************************************
					Local Variables
***********************************************************/

/***********************************************************
					Gloabal Function
***********************************************************/

void udp_recv_perf_traffic( void );
int transfer_data( unsigned char *pSendMsg, unsigned int uiLen );


/***********************************************************
					Function Declarations
***********************************************************/

/***********************************************************
					Functions
***********************************************************/


/**
 * @fn udp_recv_perf_traffic
 * @brief  Receive data on a udp session
 */
void udp_recv_perf_traffic( void )
{
	SInt32 count;
	UInt8 recv_buf[UDP_RECV_BUFSIZE];		// 수신 패킷 저장

	struct sockaddr_in from;			// 수신 정보 저장 구조체
	socklen_t fromlen = sizeof(from);	// 구조체 길이 저장


	while (1)
	{
		/* 패킷 수신 */
		count = lwip_recvfrom(sock_recv, recv_buf, UDP_RECV_BUFSIZE, 0,
						(struct sockaddr_in *)&from, &fromlen);
		if( count <= 0) {
			continue;
		}

		/* User 영역 - UDP Packet 수신 처리 */
		//printf( "recv %d\n", count );
	}
}


/**
 * @fn transfer_data
 * @brief  Transmit data on a udp session
 * @param *pSendMsg - UDP 송신 패킷
 * @param uiLen - UDP 송신 패킷 길이
 */
int transfer_data( unsigned char *pSendMsg, unsigned int uiLen )
{
	struct sockaddr_in serverAddress;		// 서버 주소

	/* 서버 주소 설정 */
    serverAddress.sin_family = AF_INET;									// 주소 패밀리 IPv4로 설정
	serverAddress.sin_addr.s_addr = inet_addr(UDP_SERVER_IP_ADDRESS);	// 서버 IP 주소 설정
	serverAddress.sin_port = htons(UDP_CONN_PORT_SEND);					// 서버 포트 번호 설정

	/* UDP 패킷 송신 */
	sendto(sock_send, pSendMsg, uiLen, 0, (struct sockaddr_in *)&serverAddress, sizeof(serverAddress));
}

