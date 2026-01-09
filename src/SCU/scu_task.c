/**
 * @file scu_task.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2023-01-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */


/***********************************************************
					Include Files
***********************************************************/

/* FreeRTOS includes. */
#include "FreeRTOS.h"			// FreeRTOS 운영 체제 관련 헤더 파일
#include "task.h"				// FreeRTOS 태스크 관련 헤더 파일
#include "queue.h"				// FreeRTOS 큐 관련 헤더 파일
#include "timers.h"				// FreeRTOS 타이머 관련 헤더 파일
#include "semphr.h"				// FreeRTOS 세마포어 관련 헤더 파일

/* Xilinx includes. */
#include <sleep.h>				// sleep 함수 관련 헤더 파일
#include "xil_printf.h"			// Xilinx printf 함수 관련 헤더 파일
#include "xparameters.h"		// Xilinx 디바이스 및 매크로 관련 헤더 파일
#include "netif/xadapter.h"		// 네트워크 인터페이스 어댑터 관련 헤더 파일
#include "scu_task.h"			// SCU 태스크 관련 헤더 파일

/* Lwip includes */
#include "lwip/init.h"			// LwIP 초기화 관련 헤더 파일
#include "lwip/inet.h"			// LwIP 인터넷 관련 함수 및 매크로 헤더 파일

/* User includes */
#include "udp_server.h"	// LwIP UDP 성능 서버 관련 헤더 파일
#include "../common/common.h"	// 사용자 정의 공통 함수 및 매크로 관련 헤더 파일



/***********************************************************
					Gloabal Variables
***********************************************************/

int sock_send;					// send sock
int sock_recv;					// recv sock
struct netif server_netif;		// netif


/***********************************************************
					Local Variables
***********************************************************/

static int complete_nw_thread;	// Thread


/***********************************************************
					Function Declarations
***********************************************************/

/***********************************************************
					Functions
***********************************************************/

/**
 * @fn print_ip
 * @brief 출력 함수
 * @date 2023-01-19
 */
static void print_ip( const char *msg, const ip_addr_t *ip)
{
	xil_printf(msg);												// 설정 메시지 출력
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
				ip4_addr3(ip), ip4_addr4(ip));						// 설정 주소 출력
}


/**
 * @fn print_ip_settings
 * @brief IP 설정
 * @date 2023-01-19
 */
static void print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	print_ip("Board IP:       ", ip);				// Board IP 출력
	print_ip("Netmask :       ", mask);				// netmask 출력
	print_ip("Gateway :       ", gw);				// gateway 출력
}


/**
 * @fn NullCmd
 * @brief NULL 명령 입력 시 출력 함수
 * @date 2023-01-19
 */
static void assign_default_ip(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{
	/* IP 주소 출력 */
	xil_printf("Configuring default IP %s \r\n", DEFAULT_IP_ADDRESS);

	/* IP 주소 변환 */
	inet_aton(DEFAULT_IP_ADDRESS, ip);

	/* Netmask 주소 변환 */
	inet_aton(DEFAULT_IP_MASK, mask);

	/* Gateway 주소 변환 */
	inet_aton(DEFAULT_GW_ADDRESS, gw);
}


/**
 * @fn NullCmd
 * @brief NULL 명령 입력 시 출력 함수
 * @date 2023-01-19
 */
void network_thread(void *p)
{
	/* the mac address of the board. this should be unique per board */
	u8_t mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };


	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(&server_netif, NULL, NULL, NULL, mac_ethernet_address,
		PLATFORM_EMAC_BASEADDR)) {

		/* Error */
		xil_printf("Error adding N/W interface\r\n");
		return;
	}

	/* Set a network interface as the default network interface */
	netif_set_default(&server_netif);

	/* specify that the network if is up */
	netif_set_up(&server_netif);

	/* start packet receive thread - required for lwIP operation */
	sys_thread_new("xemacif_input_thread",
			(void(*)(void*))xemacif_input_thread, &server_netif, 1024, 2);

	complete_nw_thread = 1;

	vTaskDelete(NULL);
}


/**
 * @fn NullCmd
 * @brief NULL 명령 입력 시 출력 함수
 * @date 2023-01-19
 */
int main_thread()
{
	err_t err;
	struct sockaddr_in addr_recv;

	/* initialize lwIP before calling sys_thread_new */
	lwip_init();

	/* any thread using lwIP should be created using sys_thread_new */
	sys_thread_new("nw_thread", network_thread, NULL, SCDAU_STACK_SIZE, DEFAULT_THREAD_PRIO);

	/* 대기 */
	while(!complete_nw_thread)
	{
		usleep(50);
	}

	/* 기본 주소 할당 */
	assign_default_ip(&(server_netif.ip_addr), &(server_netif.netmask),
				&(server_netif.gw));

	/* 설정 IP 정보 출력 */
	print_ip_settings(&(server_netif.ip_addr), &(server_netif.netmask),
				&(server_netif.gw));
	xil_printf("\r\n");

	sock_send = socket(AF_INET, SOCK_DGRAM, 0);

	/* SBC 수신 소켓 생성 */
	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
	if ( sock_recv < 0)
	{
		xil_printf("UDP server: Error creating Socket\r\n");
	}

	/* 수신 sock 정보 설정 */
	memset(&addr_recv, 0, sizeof(struct sockaddr_in));		// 초기화
	addr_recv.sin_family = AF_INET;							// AF_INET 설정
	addr_recv.sin_port = htons(UDP_CONN_PORT_RECV);			// 수신 PORT 설정
	addr_recv.sin_addr.s_addr = htonl(INADDR_ANY);			// 수신 주소 설정

	/* Binding */
	err = bind(sock_recv, (struct sockaddr_in *)&addr_recv, sizeof(addr_recv));
	if (err != ERR_OK) {
		xil_printf("UDP server: Error on bind: %d\r\n", err);
		close(sock_recv);
	}

	/* 수신 처리 */
	while(1)
	{
		/* 수신 함수 호출 */
		udp_recv_perf_traffic();

		/* 지연시간 */
		vTaskDelay( 10 );
	}
	vTaskDelete(NULL);
	return 0;
}


/**
 * @fn ScuTask
 * @brief SCU Task
 * @param pvParameters
 */
void ScuTask( void *pvParameters )
{
	/* 메인 Thread 생성 */
	sys_thread_new("main_thread", (void(*)(void*))main_thread, 0,
			SCDAU_STACK_SIZE*5, DEFAULT_THREAD_PRIO);

	while(1)
	{
		/* 지연시간 */
		vTaskDelay( 10 );
	}
	vTaskDelete(NULL);

}
