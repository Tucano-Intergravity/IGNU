

/**
 * @file dbg_task.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 1.0
 * @date 2022-11-04
 *
 * @copyright Danam Systems Copyright (c) 2022
 */

/*==============================================================================
 * Include Files
 *============================================================================*/

#include <stdio.h>			// 표준 입력/출력 라이브러리
#include <stdlib.h>			// 표준 라이브러리
#include <stddef.h>			// 포인터와 배열 관련 라이브러리
#include <string.h>			// 문자열 처리 관련 라이브러리
#include <unistd.h>			// 유닉스 시스템 관련 라이브러리
#include <time.h>			// 시간과 관련된 라이브러리
#include <sys/time.h>		// 시간 관련 라이브러리

#include "xparameters.h"	// Xilinx 지정 파라미터 관련 라이브러리
#include "xil_printf.h"		// Xilinx printf 함수 라이브러리
#include "xgpiops.h"		// Xilinx GPIO 라이브러리
#include "xil_io.h"			// Xilinx 입출력 라이브러리
#include "xuartps.h"		// Xilinx UART 라이브러리
#include "xil_exception.h"	// Xilinx 예외 처리 라이브러리
#include "xscugic.h"		// Xilinx 중앙 인터럽트 컨트롤러 라이브러리

#include "xtime_l.h"		// Xilinx 시간 라이브러리

#include "dbg_task.h"			// 디버그 태스크 관련 헤더 파일
#include "../siu/siu_task.h"	// SIU 태스크 관련 헤더 파일
#include "../common/common.h"	// 공통 유틸리티 함수 헤더 파일
#include "../opu/opu_task.h"	// OPU 태스크 관련 헤더 파일

/*==============================================================================
 * Gloabal Function
 *============================================================================*/

/*==============================================================================
 * Gloabal Variables
 *============================================================================*/
void DbgTask( void *pvParameters );										// DBG Task

/*==============================================================================
 * Local Variables
 *============================================================================*/


static SInt8 *his_ptrs[HIS_CNT];	// 히스토리 포인터 배열
static SInt8 *for_his;				// 포워드 히스토리 포인터
static SInt8 *promptp;				// 프롬프트 포인터

static SInt32 his_loc;				// 히스토리 위치
static SInt32 HIS_NEW;				// 새로운 히스토리 위치
static SInt32 bufcnt;				// 버퍼 카운트
static SInt32 usr_cmds;				// 사용자 명령 식별
static SInt32 cmdflag;				// 명령 플래그

static CMDENT *cmds;				// 명령 포인터
static USRCMD user_cmd[USRCMDS];	// 사용자 명령 배열

/*==============================================================================
 * Local Function
 *============================================================================*/
static void his_save(void);														// 수신 함수
static void init_cmd(void);														// 명령 초기화 함수
static void tohigh(char *line);													// 대소문자 변환 함수
static void cmdint(char c);														// 명령 초기화 함수
static void cmd_anal(int argc,char *argv[]);									// 명령 확인 함수
static void UsrCmdInit(void);													// 사용자 명령 초기화
static void puts_scc2(char c);													// char 출력 함수
static void UsrCmdList(void);													// 사용자 함수 등록 함수
static void uart_rx_check(void);												// UART 수신 함수
static void uart_driver_Init();													// UART 장치 초기화 함수												// 수신 함수
static void lexan(char *line);													// 수신 함수
static int lexanal(char *line);													// 수신 함수
static int NullCmd(int argc, char *argv[]);										// NULL 명령 입력 시 출력 함수
static int UartPsPolledFunc(XUartPs *UartInstPtr, u16 DeviceId);				// UART 장치 초기화
static int UsrCmdSet( const char *name, int (*cproc)(int argc, char *argv[]),
		const char *help, char flag, const char *name2);						// 사용자 명령 설정
static int help_Usrcmd(int argc, char *argv[]);									// HELP 명령 입력 시 출력 함수

/*==============================================================================
 * Functions
 *============================================================================*/
/**
 * @fn UartPsPolledFunc
 * @brief UART Polled Mode Initialization Function
 * @param UartInstPtr - Pointer to XUartPs instance
 * @param DeviceId - Device ID of the UART
 * @return XST_SUCCESS if successful, XST_FAILURE otherwise
 * @date 2023-01-19
 */
static int UartPsPolledFunc(XUartPs *UartInstPtr, u16 DeviceId)
{
	int Status;								// 상태 변수
	static XUartPs_Config *UartPs_Config;	// UART 설정 인스턴스

	/* 장치 Lookup: 주어진 장치 ID에 대한 UART 설정 검색 */
	UartPs_Config = XUartPs_LookupConfig(DeviceId);
	if (NULL == UartPs_Config) {			// 상태 확인
		return XST_FAILURE; // 설정을 찾을 수 없으면 실패 반환
	}

	/* 장치 초기화: 설정을 사용하여 UART 인스턴스 초기화 */
	Status = XUartPs_CfgInitialize(UartInstPtr, UartPs_Config, UartPs_Config->BaseAddress);
	if (Status != XST_SUCCESS) {			// 상태 확인
		return XST_FAILURE; // 초기화 실패 시 실패 반환
	}

	/* 장치 Self Test: UART의 자체 테스트 실행 */
	Status = XUartPs_SelfTest(UartInstPtr);
	if (Status != XST_SUCCESS) {			// 상태 확인
		return XST_FAILURE; // 자체 테스트 실패 시 실패 반환
	}

	/* UART를 정상 모드로 설정 */
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);

	return XST_SUCCESS; // 함수가 성공적으로 완료될 경우 성공 반환
}


/**
 * @fn UartPsPolledFunc
 * @brief UART Polled Mode Initialization Function
 * @param UartInstPtr - Pointer to XUartPs instance
 * @param DeviceId - Device ID of the UART
 * @return XST_SUCCESS if successful, XST_FAILURE otherwise
 * @date 2023-01-19
 */
static void uart_driver_Init()
{
	int status;							// 상태 저장
	static XUartPs UartPs;				// UART 장치 인스턴스

	/* 장치 초기화 함수 호출 */
	status = UartPsPolledFunc(&UartPs, UART_DEVICE_ID);
	if (status != XST_SUCCESS) {				// 상태 확인
		xil_printf("UART Polled Failed\r\n");	// UART 초기화 실패 시 메시지 출력
	}
}


/**
 * @fn uart_rx_check
 * @brief Check and process received UART data
 * @return None
 * @date 2023-01-19
 */
static void uart_rx_check(void)
{
	unsigned char temp_data;					// 데이터 임시 저장

	/* 수신 데이터 확인 */
    if(XUartPs_IsReceiveData(UART_BASEADDR))
	{
    	/* 데이터 수신 */
		temp_data = (unsigned char)XUartPs_RecvByte(UART_BASEADDR);
		cmdint((char)temp_data);
    }
}


/**
 * @fn NullCmd
 * @brief Null Command Function
 * @param argc - Argument count
 * @param argv - Argument vector
 * @return Always returns 0
 * @date 2023-01-19
 */
static int NullCmd(int argc, char *argv[])
{
	/* 터미널 출력 */
	xil_printf("%s",promptp);
	return 0;				// '0' 값 반환
}


/**
 * @fn help_Usrcmd
 * @brief Help Command Function
 * @param argc - Argument count
 * @param argv - Argument vector
 * @return Always returns 0
 * @date 2023-01-19
 */
static int help_Usrcmd(int argc, char *argv[])
{
	int	i;

	/* HELP 명령 출력: 사용 가능한 사용자 명령 목록 출력 */
	for(i=0;i<usr_cmds;i++)
	{
		/* null인 경우 */
		if(strcmp(user_cmd[i].name,"NULL"))
		{
			/* 출력 */
			xil_printf("%02d : %s 	%s\r\n",i,user_cmd[i].name,user_cmd[i].help);
		}
	}
	return 0; // 함수 완료 후 0 반환
}


/**
 * @fn UsrCmdSet
 * @brief Set User Command
 * @param name - Command name
 * @param cproc - Command processing function
 * @param help - Command help message
 * @param flag - Flag to indicate whether to use 'name' or 'name2'
 * @param name2 - Alternative command name (used when flag is 'C')
 * @return Index of the added command, or -1 if maximum command limit reached
 * @date 2023-01-19
 */
static int UsrCmdSet(const char *name, int (*cproc)(int argc, char *argv[]), const char *help, char flag, const char *name2)
{
    int i, ii, jj;     // 변수 선언
    char *cp;          // 포인터 변수 선언

    tohigh(name);       // 문자열을 대문자로 변환

    ii = usr_cmds;      // 명령 저장
    usr_cmds++;		// 사용자 명령 증가

    // 사용자 명령 구조체에 정보 설정
    user_cmd[ii].cproc = cproc;

    // 명령 이름 설정 (최대 8자까지 지원)
    memset( user_cmd[ii].name, 0x00, 8 );
    cp = user_cmd[ii].name;
    for (jj = 0; jj < 8 && name[jj] != '\0'; jj++)
    {
        *cp++ = (char)name[jj];		// name 저장
    }

    // 명령 도움말 설정
    for (jj = 0; help[jj] != '\0'; jj++)
    {
        user_cmd[ii].help[jj] = help[jj];	// 명령 저장
    }
    user_cmd[ii].help[jj] = '\0';			// 문자열 끝

    // 추가된 명령의 인덱스를 반환
    return (usr_cmds - 1);
}


/**
 * @fn UsrCmdInit
 * @brief Initialize User Commands
 * @return None
 * @date 2023-01-19
 */
static void UsrCmdInit(void)
{
	int	i;

	/* 사용자 명령 초기화 */
	for(i=0;i<USRCMDS;i++)
	{
	    // 사용자 명령 배열의 요소 초기화
	    strcpy(user_cmd[i].name, "NULL"); // 명령 이름 초기화
	    strcpy(user_cmd[i].help, "NULL"); // 명령 도움말 초기화

	    user_cmd[i].cproc = NullCmd;      // 명령 처리 함수 초기화
	}
	usr_cmds=0;

	/* HELP 명령 등록 */
	UsrCmdSet("HE",help_Usrcmd,"Help!!",'N',"\0");
}


/**
 * @fn puts_scc2
 * @brief Print a character to the console
 * @param c - Character to print
 * @return None
 * @date 2023-01-19
 */
static void puts_scc2(char c)
{
	/* 출력 */
	xil_printf("%c", c);		// char 출력
}


/**
 * @fn cmdint
 * @brief Process characters received from the console
 * @param c - Character to process
 * @return None
 * @date 2023-01-19
 */
static void cmdint(char c)
{
    static SInt8 shbuf[SHBUFLEN];    // 명령 입력을 저장하는 버퍼
    static UInt8 escape_char_flag;   // 이스케이프 문자 플래그
    static UInt8 escape_char_count;  // 이스케이프 문자 카운트

    // 새로운 문자를 처리
    if (c != '\r' && c != '\n' && bufcnt < SHBUFLEN - 2)
    {
        // 백스페이스 및 삭제 문자 처리
        if (c == DEL || c == BS)
        {
            if (bufcnt)
            {
                puts_scc2(BS);		// BS 처리
                puts_scc2(' ');		// blank
                puts_scc2(BS);		// BS 처리
                bufcnt--;			// 버퍼 카운트 감소
                return;				// 반환 (종료)
            }
        }

        // 이스케이프 문자 처리
        if (c != 0x1b)
        {
            if (escape_char_flag == 0)
            {
                puts_scc2(c);           // 문자를 화면에 출력
                shbuf[bufcnt++] = c;    // 입력된 문자를 버퍼에 저장
            }
        }
    }

    // Enter 키를 누를 때 명령 실행
    if (c == '\r' || c == '\n')
    {
        escape_char_flag = 0;      // 이스케이프 문자 관련 플래그 초기화
        escape_char_count = 0;     // 이스케이프 문자 카운트 초기화
        puts_scc2('\r');           // '\r'을 화면에 출력
        puts_scc2('\n');           // '\n'을 화면에 출력

        shbuf[bufcnt] = 0;         // 입력된 명령 문자열 종료
        bufcnt = 0;                // 버퍼 카운트 초기화
        lexan(shbuf);              // 입력된 명령 실행
    }
}


/**
 * @fn cmd_anal
 * @brief Analyze and execute user commands
 * @param argc - Argument count
 * @param argv - Argument vector
 * @return None
 * @date 2023-01-19
 */
static void cmd_anal(int argc, char *argv[])
{
    int i;

    // 인수 없을 경우 프롬프트 출력 후 반환
    if (argc == 0)
    {
        xil_printf("%s", promptp);
        return;
    }

    tohigh(argv[0]); // 대문자로 변환

    // 사용자 명령 확인 및 실행
    for (i = 0; i < usr_cmds; i++)
    {
        if (!(strcmp(argv[0], user_cmd[i].name)))
        {
            his_save(); // 히스토리 저장
			user_cmd[i].cproc(argc, argv);
            xil_printf("%s", promptp); // 프롬프트 출력
            cmdflag = 0;
            return;
        }
    }

    cmdflag = 0;
    xil_printf("COMMAND ERROR\r\n"); // 명령 오류 메시지 출력
    xil_printf("%s", promptp);      // 프롬프트 출력
}



/**
 * @fn lexan
 * @brief Lexical analysis and execution of commands
 * @param line - Command string to analyze and execute
 * @return None
 * @date 2023-01-19
 */
static void lexan(char *line)
{
    /*  command history */
    for_his = line; // 히스토리 저장

    lexanal(line); // 명령 해석 및 실행
}

/**
 * @fn lexanal
 * @brief Lexical analysis of a command line and execution
 * @param line - Command string to analyze
 * @return Number of tokens found
 * @date 2023-01-19
 */
static int lexanal(char *line)
{
    static SHVARS Shl;
    char **tokptr;      // 토큰 포인터 배열
    int ntok;           // 토큰 개수
    char *p;            // 문자열 포인터
    char ch;            // 현재 문자
    char *to;           // 토큰 저장 버퍼
    char quote;         // 따옴표 종류

    to = Shl.shargst;        // 토큰 저장 버퍼 초기화
    ntok = 0;
    tokptr = &Shl.shtok[ntok];    // 토큰 포인터 배열 초기화

    for (p = line; *p != '\0' && ntok < SHMAXTOK; )
    {
        while ((ch = *p++) == ' ')
        {
        	// 공백 문자 스킵
        }

        *tokptr++ = to;    // 토큰 시작 포인터 저장
        *to++ = ch;        // 토큰 버퍼에 문자 추가
        ntok++;            // 토큰 개수 증가

        while ((ch = *p) != '\0' &&
               ch != ';' && ch != '=' && ch != ' ' &&
               ch != '"' && ch != '\'' && ch != ':') // 토큰 문자 처리
        {
            *to++ = *p++;
        }
        *to++ = NULLCH; // 문자열 종료
    }

    cmd_anal(ntok, Shl.shtok); // 명령 해석 및 실행
    return (ntok);
}


/**
 * @fn his_save
 * @brief Save a command line to the command history buffer
 * @date 2023-01-19
 */
static void his_save(void)
{
    static SInt8 his_buf[HIS_CNT * 40]; // 히스토리 버퍼
    char *ptr = for_his; // 히스토리 저장 대상 문자열 포인터

    his_ptrs[HIS_NEW & HIS_MSK] = &his_buf[his_loc]; // 히스토리 포인터 설정
    HIS_NEW++; // 히스토리 인덱스 증가

    do
    {
        his_buf[his_loc++] = *ptr; // 히스토리 버퍼에 문자 저장

    } while (*ptr++);
}


/**
 * @fn init_cmd
 * @brief Initialize command-related variables
 * @date 2023-01-19
 */
static void init_cmd(void)
{
    HIS_NEW = 0;   // 히스토리 인덱스 초기화
    his_loc = 0;   // 히스토리 위치 초기화
    cmdflag = 0;   // 명령 플래그 초기화
    bufcnt = 0;    // 버퍼 카운트 초기화
}


/**
 * @fn tohigh
 * @brief Convert a string to uppercase
 * @param line - String to convert
 * @date 2023-01-19
 */
static void tohigh(char *line)
{
    /* 라인 검색 */
    while (*line)
    {
    	/* 소문자 검색 */
        if (*line >= 'a' && *line <= 'z')
        {
            *line -= ' '; 	// 소문자를 대문자로 변환
        }
        line++;				// 라인 증가
    }
}

static int htoi(char *s)
{
  	int	n = 0;
	char	ch;

	if(*s == ':') s++;
	ch=*s;
	while(ch) {
  		if((ch >= '0') && (ch <= '9'))
  			n = (n << 4) + ch - '0';
		else {
 			ch |= ' ';
			if((ch >= 'a') && (ch <= 'f'))
			{
 				n = (n << 4) + ch - 'a' + 10;
			} else {
				return(n);
			}
		}
		s++;ch=*s;
	}
	return(n);
}


static int testUartLogFunc(int argc, char *argv[])
{
	UInt16 usDbgCmd;

	if(argc<2)
	{
		printf("cmd error\n");
	}
	else
	{
		usDbgCmd = htoi(argv[1]);
		usUartFlag = usDbgCmd;
		printf( "UART Packet log cmd : %04X\n", usUartFlag );

	}

	return(0);					// '0' 리턴
}


static int testGpsLogFunc(int argc, char *argv[])
{
	UInt16 usDbgCmd;

	if(argc<2)
	{
		printf("cmd error\n");
	}
	else
	{
		usDbgCmd = htoi(argv[1]);
		usGpsFlag = usDbgCmd;
		printf( "GPS Packet log cmd : %04X\n", usGpsFlag );

	}

	return(0);					// '0' 리턴
}


static int testImuLogFunc(int argc, char *argv[])
{
	UInt16 usDbgCmd;

	if(argc<2)
	{
		printf("cmd error\n");
	}
	else
	{
		usDbgCmd = htoi(argv[1]);
		usImuFlag = usDbgCmd;
		printf( "IMU Packet log cmd : %04X\n", usImuFlag );

	}

	return(0);					// '0' 리턴
}

/**
 * @fn UsrCmdList
 * @brief Initialize and list user commands
 * @date 2023-01-19
 */
static void UsrCmdList(void)
{
	// test 함수
	UsrCmdSet( "uart", testUartLogFunc,"UART Log Function Command",'N',"\0");
	UsrCmdSet( "gps", testGpsLogFunc,"GPS Log Function Command",'N',"\0");
	UsrCmdSet( "imu", testImuLogFunc,"IMU Log Function Command",'N',"\0");
}


/**
 * @fn DbgTask
 * @brief Debug task function
 * @param pvParameters - Task parameters (not used in this function)
 * @date 2023-01-19
 */
void DbgTask( void *pvParameters )
{
#if TASK_STACK_SIZE_CHECK
    unsigned long uxHighWaterMark;
    /* Inspect our own high water mark on entering the task. */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
#endif

    /* RS232 장치 초기화 */
    uart_driver_Init();

    /* 명령창 커서문구 */
	promptp="GINU>";

	/* 초기화 */
	init_cmd();			// 명령 초기화
	UsrCmdInit();		// 사용자 명령 초기화
	UsrCmdList();		// 사용자 명령 등록

	/* 명령창 출력 */
	xil_printf("%s",promptp);

    while(1)
    {
#if TASK_STACK_SIZE_CHECK
    	uart_rx_check();
    	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    	xil_printf( "DBG Task : %d\r\n", uxHighWaterMark );
    	vTaskDelay( 100 );
#else
    	/* 명령 수신 */
    	uart_rx_check();

    	/* 동작 지연시간 */
    	vTaskDelay( 10 );
#endif
    }
}
