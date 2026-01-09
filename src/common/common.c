/**
 * @file common.c
 * @author Heesung Shin (shs777@danam.co.kr)
 * @brief
 * @version 0.1.0
 * @date 2022-11-04
 *
 * @copyright Danam Systems Copyright (c) 2024
 */
#include "xgpiops.h"
#include "common.h"

/*==============================================================================
 * Gloabal Variables
 *============================================================================*/
UInt8 ucPsState = 0;            // PS SW 모드 상태
UInt16 usGpsFlag = 0;			// [TEST] GPS 로그
UInt16 usImuFlag = 0;			// [TEST] IMU 로그
UInt16 usUartFlag = 0;			// [TEST] UART 로그

/*==============================================================================
 * Local Variables
 *============================================================================*/
static XGpioPs gGpio;									// GPIO 제어 변수
static XGpioPs_Config *pConfigPtr;						// GPIO 설정 구조체

/*==============================================================================
 * Function Declarations
 *============================================================================*/
void gpioDriverInit(void);															// GPIO 디바이스 초기화
void gpioSetFunc(void);																// GPIO 설정 (네트워크 리셋 포트)
void ByteSwap_2( SInt8 *cSource );													// 2bytes SWAP 함수
void ByteSwap_4( SInt8 *cSource );													// 4bytes SWAP 함수
void PsToPlCommand( UInt32 uiCmd, UInt32 uiAddr );									// PS->PL 명령 함수
void BramRead( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );					// BRAM 1byte Read
void BramWrite( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );				// BRAM 1byte Write
void BramWrite16( UInt16 *pMsg, UInt16 usMsgLen, UInt32 uiIndexStart );				// BRAM 2bytes Write
void BramWrite32( UInt32 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart );				// BRAM 4bytes Write
UInt16 CalcCKS16(UInt32 uiSumOffset, const UInt16 *uspData, UInt32 uiLength);		// 16bits CRC
UInt32 CalcCRC32(const UInt8 *ucpData, UInt32 uiLength);							// 32bits CRC

/*==============================================================================
 * Functions
 *============================================================================*/
/**
 * @fn		gpioDriverInit
 * @brief	GPIO 장치 초기화
 * @param	void
 * @return	void
 * @date	2023/03/13
 */
void gpioDriverInit(void)
{
	pConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
	XGetPlatform_Info();

	XGpioPs_CfgInitialize(&gGpio, pConfigPtr, pConfigPtr->BaseAddr);

}

/**
 * @fn		gpioSetFunc
 * @brief	Ethernet PHY GPIO 장치 Reset
 * @param	void
 * @return	void
 * @date	2023/03/13
 */
void gpioSetFunc(void)
{
	gpioDriverInit();

	/* GPIO 출력 설정 */
	XGpioPs_SetDirectionPin(&gGpio, 46, 1);
	XGpioPs_SetDirectionPin(&gGpio, 47, 1);

	/* GPIO 활성화 */
	XGpioPs_SetOutputEnablePin(&gGpio, 46, 1);
	XGpioPs_SetOutputEnablePin(&gGpio, 47, 1);

	/* GPIO GND 설정 */
	XGpioPs_WritePin(&gGpio, 46, 1);
	XGpioPs_WritePin(&gGpio, 47, 1);

	usleep(100000);

	/* GPIO GND 설정 */
	XGpioPs_WritePin(&gGpio, 46, 0);
	XGpioPs_WritePin(&gGpio, 47, 0);

	usleep(100000);

	/* GPIO GND 설정 */
	XGpioPs_WritePin(&gGpio, 46, 1);
	XGpioPs_WritePin(&gGpio, 47, 1);
}


/**
 * @fn ByteSwap_2
 * @brief 2 byte Swap 함수
 * @param cSource 2 byte 데이터의 포인터
 * @return void
 */
void ByteSwap_2( SInt8 *cSource )
{
	SInt8 cSrcTmp[2];

	memcpy( cSrcTmp, cSource, sizeof(cSrcTmp) );

	cSource[0] = cSrcTmp[1];
	cSource[1] = cSrcTmp[0];
}

/**
 * @fn ByteSwap_4
 * @brief 4 byte Swap 함수
 * @param cSource 4 byte 데이터의 포인터
 * @return void
 */
void ByteSwap_4( SInt8 *cSource )
{
	SInt8 cSrcTmp[4];

	memcpy( cSrcTmp, cSource, sizeof(cSrcTmp) );

	cSource[0] = cSrcTmp[3];
	cSource[1] = cSrcTmp[2];
	cSource[2] = cSrcTmp[1];
	cSource[3] = cSrcTmp[0];
}

/**
 * @fn BramRead
 * @brief BRAM Read 함수
 * @param pBuf Read Data Buffer
 * @param usMsgLen Read Data Length
 * @param uiIndexStart BRAM Read Address
 * @return void
 */
void BramRead( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart )
{
	UInt16 usLen;
	UInt32 uiIndex;
    UInt32 uiRecvBuf[(MSG_MAX/4)];

	volatile UInt32 *uipPtr = (volatile UInt32 *)(uiIndexStart);

	usLen = (usMsgLen%4 > 0)?(usMsgLen + 4 - (usMsgLen%4)):usMsgLen;

	for( uiIndex = 0; uiIndex < (usLen/4); uiIndex++ )
	{
		uiRecvBuf[uiIndex] = uipPtr[uiIndex];
	}

    memcpy( pBuf, uiRecvBuf, sizeof(uiRecvBuf) );
}

/**
 * @fn BramWrite
 * @brief BRAM Write 함수
 * @param pBuf Read Write Buffer (1 byte 단위 저장)
 * @param usMsgLen Read Write Length
 * @param uiIndexStart BRAM Write Address
 * @return void
 */
void BramWrite( UInt8 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart )
{
	UInt32 uiIndex;
	volatile UInt8 *uipPtr = (volatile UInt8 *)(uiIndexStart);

	for( uiIndex = 0; uiIndex < usMsgLen; uiIndex++ )
	{
		uipPtr[uiIndex] = pBuf[uiIndex];
	}
}

/**
 * @fn BramWrite16
 * @brief BRAM Write 함수
 * @param pBuf Read Write Buffer (2 byte 단위 저장)
 * @param usMsgLen Read Write Length
 * @param uiIndexStart BRAM Write Address
 * @return void
 */
void BramWrite16( UInt16 *pMsg, UInt16 usMsgLen, UInt32 uiIndexStart )
{
	UInt32 uiIndex;
	volatile UInt16 *uipPtr = (volatile UInt16 *)(uiIndexStart);

	usMsgLen = (usMsgLen%2 > 0)?(usMsgLen+1):usMsgLen;

	for( uiIndex = 0; uiIndex < usMsgLen/2; uiIndex++ )
	{
		uipPtr[uiIndex] = pMsg[uiIndex];
	}
}

/**
 * @fn BramWrite
 * @brief BRAM Write 함수
 * @param pBuf Read Write Buffer (1 byte 단위 저장)
 * @param usMsgLen Read Write Length
 * @param uiIndexStart BRAM Write Address
 * @return void
 */
void BramWrite32( UInt32 *pBuf, UInt16 usMsgLen, UInt32 uiIndexStart )
{
	UInt32 uiIndex;
	volatile UInt32 *uipPtr = (volatile UInt32 *)(uiIndexStart);

	for( uiIndex = 0; uiIndex < (usMsgLen/4); uiIndex++ )
	{
		uipPtr[uiIndex] = pBuf[uiIndex];
	}
}

/**
 * @fn PsToPlCommand
 * @brief PS에서 PL 명령 함수
 * @param uiCmd 명령 입력
 * @return void
 */
void PsToPlCommand( UInt32 uiCmd, UInt32 uiAddr )
{
	volatile UInt32 *uipPtr = (volatile UInt32 *)uiAddr;
	*uipPtr = uiCmd;
}

/**
 * @fn CalcCRC32
 * @brief 32bits CRC 생성 함수
 * @param pData CRC 계산 데이터 포인터
 * @param uiLength CRC 계산 데이터 길이
 * @return UInt32 CRC 계산 값
 */
UInt32 CalcCRC32(const UInt8 *pData, UInt32 uiLength)
{
    /* Variable */
	UInt32 ui = 0, uj = 0, uk = 0; // Simple Loop
    //------------------------------------------------------------------------------------------------
	UInt32 uiResult = 0; // Saving Buffer for CRC result
	UInt32 uiTempBuffer = 0; // Help buffer for calculate the CRC
	UInt32 uiReadData = 0; // CRC32 Calculate Data
	UInt32 uiCRCOffset = 0xffffffffL;// CRC32 Old Data
	UInt32 uiTransData = 0; // Transfer the 32bit Data
    //------------------------------------------------------------------------------------------------
	UInt32 uiCRCTable = 0; // Help buffer for make the CRC Table
    //------------------------------------------------------------------------------------------------

    /* Array */
	UInt32 uiaCRCTable[256];
    //------------------------------------------------------------------------------------------------

    memset(uiaCRCTable, 0x00, sizeof(uiaCRCTable));

    if((pData != NULL) && (uiLength != 0x00)) {
        // Set 'ui' Value for Process the Useless Assignment
        uj = TRUE;

        // Make a CRC Calculate Table
        for (uj = 0; uj < 256; uj++) {

            uiCRCTable = uj;

            for (uk = 0; uk < 8; uk++) {

                if (uiCRCTable & 0x00000001L) {
                    uiCRCTable = (uiCRCTable >> 1) ^ CRC_POLY_32;
                } else {
                    uiCRCTable = uiCRCTable >> 1;
                }
            }

            uiaCRCTable[uj] = uiCRCTable;
        }

        // Set 'ui' Value for Process the Useless Assignment
        ui = TRUE;

        // Calculate the CRC 32
        for (ui = 0; ui < uiLength; ui++) {
            // Set the Read Data
            uiTransData = pData[ui];

            // Change the Read Data
            uiReadData = (0x000000ffL & uiTransData);

            // Calculate CRC 32
            uiTempBuffer = uiCRCOffset ^ uiReadData;

            // Make a CRC 32 Result
            uiCRCOffset = (uiCRCOffset >> 8) ^ uiaCRCTable[uiTempBuffer & 0xff];

            // Last change for CRC 32
            uiResult = uiCRCOffset & 0xffffffffL;
        }

        // Generate a CRC 32 Value
        uiResult ^= 0xffffffffL;
    }

    // Return the CRC 32 result
    return uiResult;
}

/**
 * @fn CalcCRC16
 * @brief 16bits CRC 생성 함수
 * @param uiSumOffset  Sum Process Buffer
 * @param pData CRC 계산 데이터 포인터
 * @param uiLength CRC 계산 데이터 길이
 * @return UInt16 CRC 계산 값
 */
UInt16 CalcCKS16(UInt32 uiSumOffset, const UInt16 *pData, UInt32 uiLength)
{
    /* Variable */
	UInt32 ui = 0; // Simple Loop
	UInt32 uiSum = uiSumOffset; // Sum Process Buffer
    //------------------------------------------------------------------------------------------------

    // Checking to the Null pointer and zero length
    if ((pData != NULL) && (uiLength != 0x00)) {
        // Set 'ui' Value for Process the Useless Assignment
        ui = TRUE;

        for (ui = 0; ui < (uiLength / 2); ui++) {
            // Sum per two byte
            uiSum += (UInt32) pData[ui];
        }

        // Additional process about odd length
        if ((uiLength % 2) != 0) {
            // If length is odd, one more sum per two byte
            uiSum += (UInt32) (pData[ui] & 0xFF00);
        }

        // Re add the value about over 2 byte
        uiSum = (uiSum > 0xFFFF) ? ((uiSum & 0xFFFF) + (uiSum >> 16)) : uiSum;

        //  Re add the one more time
        uiSum = (uiSum > 0xFFFF) ? ((uiSum & 0xFFFF) + (uiSum >> 16)) : uiSum;
    }

    // Return the Checksum 16 Value
    return ((UInt16)(uiSum & 0xFFFF));
}

