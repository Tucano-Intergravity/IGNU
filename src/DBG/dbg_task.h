/**
 * @file dbg_task.h
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2022-11-04
 *
 * @copyright Danam Systems Copyright (c) 2022
 */
#ifndef __DBG_TASK_H__
#define __DBG_TASK_H__

#include "../common/common.h"

extern void DbgTask( void *pvParameters );			// DbgTask 함수 선언


/*==============================================================================
 * 매크로 및 상수 정의
 *============================================================================*/
#define MH                          0
#define pl_ready_cmd                0x00000001
#define pl_stop_cmd                 0x00000020
#define pcm_data_ram_cmddata_0      0x20028002
#define pcm_data_ram_cmddata_1      0x0200004e
#define pcm_data_ram_cmddata_2      0x4e200280
#define pcm_data_ram_cmddata_3      0x00000000
#define SHMAXTOK                    16				// 한 라인당 최대 토큰 수
#define SHARGLEN                    170				// 인수 저장 영역 길이
#define SHBUFLEN                    168				// 일반 버퍼 길이
#define DEL                         0x7f
#define BS                          '\b'
#define BELL                        7
#define HIS_CNT                     64			// 2의 거듭제곱이어야 함
#define HIS_MSK                     (HIS_CNT-1)
#define USRCMDS                     100			// 사용자 명령 수

#define UART_DEVICE_ID              XPAR_XUARTPS_0_DEVICE_ID
#define UART_BASEADDR               XPAR_XUARTPS_0_BASEADDR

#define NULLCH                      '\0'
#define LF                          0x0a		// 줄 바꿈 문자
#define CR                          0x0d		// 리턴 문자
#define CAN                         0x18		// 취소 문자
#define DEL                         0x7f		// 삭제 문자

/*==============================================================================
 * 구조체
 *============================================================================*/
typedef struct shvars {
    SInt8 *shtok[SHMAXTOK];						// 입력 토큰을 가리키는 포인터 배열
    SInt8 shargst[SHARGLEN];					// 실제 인수 문자열
} SHVARS;

typedef struct cmdent {							// 명령 테이블의 항목
    SInt32 flag;								// 반복 실행 플래그
    SInt8 *cmdnam;								// 명령 이름
    SInt32 (*cproc)(int argc, char *argv[]);	// 실행 프로시저
    SInt8 *cmdhelp;								// 도움말 명령
} CMDENT;

typedef struct usrcmd {
    SInt8 name[9];								// 명령 이름
    SInt32 (*cproc)(int argc, char *argv[]);	// 실행 프로시저
    SInt8 help[80];								// 도움말
} USRCMD;

#endif 			//__DBG_TASK_H__
