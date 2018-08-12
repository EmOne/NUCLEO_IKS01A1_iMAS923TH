//------------------------------------------------------------------------------
//
//	File:		WiMOD_LoRaWAN_API.cpp
//
//	Abstract:	API Layer of LoRaWAN Host Controller Interface
//
//	Version:	0.1
//
//	Date:		18.05.2016
//
//	Disclaimer:	This example code is provided by IMST GmbH on an "AS IS" basis
//				without any warranties.
//
//	Maintain by : Anol Paisal <anol.paisal@emone.co.th> @ 2018
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
// Include Files
//
//------------------------------------------------------------------------------

#include "WiMOD_LoRaWAN_API.h"
#include "WiMOD_HCI_Layer.h"
#include "SLIP.h"
#include <string.h>
#include <stdio.h>

#define MAKEWORD(lo,hi) ((lo)|((hi) << 8))
#define MAKELONG(lo,hi) ((lo)|((hi) << 16))

//eWimod_msg_state msg_state;
///**
// * @brief  This function converts a text string into a number.
// * @param  str       The pointer to the text string to convert.
// * @param  pnum      The pointer to the numerical variable.
// * @retval FlagStatus  SET or RESET related to the conversion.
// */
FlagStatus str2num(uint8_t* str, uint32_t* pnum) {
	uint8_t TxtStr_digit, digit;
	uint32_t tenpwr;
	uint32_t number;

	digit = 0;

	while (*(str + digit) != '\0') {
		if (((*(str + digit) >= '0') && (*(str + digit) <= '9'))) {
			digit++;
		} else {
			*pnum = 0;
			return RESET;
		}
	}

	tenpwr = 1;
	number = 0;

	do {
		TxtStr_digit = (*(str + (--digit)));
		number += ((TxtStr_digit - '0') * tenpwr);
		tenpwr *= 10;
	} while (digit);

	*pnum = number;
	return SET;
}
//
///**
// * @brief  Convert a number nbr into a string str with 7 characters.
// * @param  nbr The number to be converted.
// * @param  str The container of the converted number into a text in decimal
// *         format.
// * @note   The decimal digits of the number must be maximum 7 so str has to be
// *         able to store at least 7 characters plus '\0'.
// */
void num2str(uint32_t nbr, uint8_t *str) {
	uint8_t k;
	uint8_t *pstrbuff;
	uint32_t divisor;

	pstrbuff = str;

	/* Reset the text string */
	for (k = 0; k < 7; k++)
		*(pstrbuff + k) = '\0';

	divisor = 1000000;

	if (nbr) // if nbr is different from zero then it is processed
	{
		while (!(nbr / divisor)) {
			divisor /= 10;
		}

		while (divisor >= 10) {
			k = nbr / divisor;
			*pstrbuff++ = '0' + k;
			nbr = nbr - (k * divisor);
			divisor /= 10;
		}
	}

	*pstrbuff++ = '0' + nbr;
	*pstrbuff++ = '\0';
}

///**
// * @brief  Convert an integer number into hexadecimal format.
// *
// * @param  num         The integer number to convert.
// * @param  HexFormat   The output format about hexadecimal number.
// *
// * @retval uint8_t*    The address of the string text for the converted hexadecimal number.
// */
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat) {
	static uint8_t HexValue[8 + 1];
	uint8_t i;
	uint8_t dummy;
	uint8_t HexDigits = 0;

	switch (HexFormat) {
	case HALFBYTE_F:
		HexDigits = 1;
		break;
	case BYTE_F:
		HexDigits = 2;
		break;
	case WORD_F:
		HexDigits = 4;
		break;
	case DOUBLEWORD_F:
		HexDigits = 8;
		break;
	default:
		HexDigits = 2;
		break;
	}

	for (i = 0; i < HexDigits; i++) {
		HexValue[i] = '\0';
		dummy = (num & (0x0F << (((HexDigits - 1) - i) * 4)))
				>> (((HexDigits - 1) - i) * 4);
		if (dummy < 0x0A) {
			HexValue[i] = dummy + '0';
		} else {
			HexValue[i] = (dummy - 0x0A) + 'A';
		}
	}
	HexValue[i] = '\0';

	return HexValue;
}

//------------------------------------------------------------------------------
//
//  Forward Declarations
//
//------------------------------------------------------------------------------

// HCI Message Receiver callback
static TWiMOD_HCI_Message*
WiMOD_LoRaWAN_Process_RxMessage(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_Process_DevMgmt_Message(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_Get_OPMODE_Rsp(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_Get_RTC_ALARM_Rsp(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_Get_RTC_Rsp(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_DeviceStatus_Rsp(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_DeviceInfo_Rsp(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_DevMgmt_FirmwareVersion_Rsp(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_Process_LoRaWAN_Message(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_Process_Reactivate_Rsp(TWiMOD_HCI_Message*  rxMessage);

static void
WiMOD_LoRaWAN_Process_U_DATA_TxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_C_DATA_TxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_JoinTxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_JoinNetworkIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_U_DataRxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_C_DataRxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_MAC_CMDRxIndication(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Set_RSTACK_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_RSTACK_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_Support_Band_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_Device_EUI_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_Custom_CFG_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_Network_Status_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_Process_Get_LINK_ADR_RSP(TWiMOD_HCI_Message* rxMessage);

static void
WiMOD_LoRaWAN_ShowResponse(const char* string, const TIDString* statusTable, UINT8 statusID);

//------------------------------------------------------------------------------
//
//  Section RAM
//
//------------------------------------------------------------------------------

// reserve one Tx-Message
TWiMOD_HCI_Message TxMessage;

// reserve one Rx-Message
TWiMOD_HCI_Message RxMessage;

//------------------------------------------------------------------------------
//
//  Section Const
//
//------------------------------------------------------------------------------

// Status Codes for DeviceMgmt Response Messages
static const TIDString WiMOD_DeviceMgmt_StatusStrings[] =
{
    { DEVMGMT_STATUS_OK,                   "ok" },
    { DEVMGMT_STATUS_ERROR,                "error" } ,
    { DEVMGMT_STATUS_CMD_NOT_SUPPORTED,    "command not supported" },
    { DEVMGMT_STATUS_WRONG_PARAMETER,      "wrong parameter" },
    { DEVMGMT_STATUS_WRONG_DEVICE_MODE,    "wrong device mode" },

    // end of table
    { 0, 0 }
};

// Status Codes for LoRaWAN Response Messages
static const TIDString WiMOD_LoRaWAN_StatusStrings[] =
{
    { LORAWAN_STATUS_OK,                    "ok" },
    { LORAWAN_STATUS_ERROR,                 "error" } ,
    { LORAWAN_STATUS_CMD_NOT_SUPPORTED,     "command not supported" },
    { LORAWAN_STATUS_WRONG_PARAMETER,       "wrong parameter" },
    { LORAWAN_STATUS_WRONG_DEVICE_MODE,     "wrong device mode" },
    { LORAWAN_STATUS_DEVICE_NOT_ACTIVATED,         "device not activated" },
    { LORAWAN_STATUS_DEVICE_BUSY,                  "device busy - command rejected" },
    { LORAWAN_STATUS_QUEUE_FULL,            "message queue full - command rejected" },
    { LORAWAN_STATUS_LENGTH_ERROR,          "HCI message length error" },
    { LORAWAN_STATUS_NO_FACTORY_SETTINGS,   "no factory settings available" },
    { LORAWAN_STATUS_CHANNEL_BLOCKED_BY_DC, "error: channel blocked due to duty cycle, please try later again" },
    { LORAWAN_STATUS_CHANNEL_NOT_AVAILABLE, "error: channel not available" },

    // end of table
    { 0, 0 }
};

// Status Codes for DeviceMgmt Response Messages
static const TIDString WiMOD_DeviceMgmt_ModuleTypes[] =
{
    { 0x90, "iM880A (obsolete)" },
    { 0x92, "iM880A-L (128k)" } ,
    { 0x93, "iU880A (128k)" },
    { 0x98, "iM880B-L" },
    { 0x99, "iU880B" },
	{ 0x9A, "iM980A (iMAS923TH for Thailand)" },
	{ 0xA0, "iM881A" },
    // end of table
    { 0, 0 }
};

//------------------------------------------------------------------------------
//
//  Section Code
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
//  Init
//
//  @brief: init complete interface
//
//------------------------------------------------------------------------------

bool
WiMOD_LoRaWAN_Init(
#ifdef Q_OS_WIN
		const char*              comPort        // comPort
#else
		UART_HandleTypeDef * comPort
#endif
		)
{
    // init HCI layer
    return WiMOD_HCI_Init(comPort,                  // comPort
                   WiMOD_LoRaWAN_Process_RxMessage, // receiver callback
                   &RxMessage);                     // rx message
}


int WiMOD_DevMgmt_Msg_Req(uint8_t msg_id, uint8_t* val, uint16_t len)
{
	int ret;
	TxMessage.SapID = DEVMGMT_SAP_ID;
	TxMessage.MsgID = msg_id;

	memset(TxMessage.Payload, 0x00, WIMOD_HCI_MSG_PAYLOAD_SIZE);

	switch (msg_id) {
		case DEVMGMT_MSG_PING_REQ:
		case DEVMGMT_MSG_RESET_REQ:
		case DEVMGMT_MSG_GET_DEVICE_INFO_REQ:
		case DEVMGMT_MSG_GET_FW_VERSION_REQ:
		case DEVMGMT_MSG_GET_DEVICE_STATUS_REQ:
		case DEVMGMT_MSG_GET_RTC_REQ:
		case DEVMGMT_MSG_GET_RTC_ALARM_REQ:
		case DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ:
		case DEVMGMT_MSG_GET_OPMODE_REQ:
			TxMessage.Length = 0;
			break;
		case DEVMGMT_MSG_SET_RTC_REQ:
		case DEVMGMT_MSG_SET_RTC_ALARM_REQ:
			TxMessage.Length = 4;
			break;
		case DEVMGMT_MSG_SET_OPMODE_REQ:
			TxMessage.Length = 1;
			break;
		default:
			return DEVMGMT_STATUS_CMD_NOT_SUPPORTED;
	}

	if(TxMessage.Length > 0)
		memcpy(TxMessage.Payload, val, TxMessage.Length);

	ret = WiMOD_HCI_SendMessage(&TxMessage);

	if(ret)
		WiMOD_HCI_Process();

	return ret;
}

int WiMOD_LoRaWAN_Msg_Req(uint8_t msg_id, uint8_t* val, uint16_t len)
{
	int ret;
	TxMessage.SapID = LORAWAN_SAP_ID;
	TxMessage.MsgID = msg_id;

	memset(TxMessage.Payload, 0x00, WIMOD_HCI_MSG_PAYLOAD_SIZE);

	switch (msg_id) {
		case LORAWAN_MSG_ACTIVATE_DEVICE_REQ:
			TxMessage.Length = 36;
			break;
		case LORAWAN_MSG_REACTIVATE_DEVICE_REQ:
		case LORAWAN_MSG_JOIN_NETWORK_REQ:
		case LORAWAN_MSG_GET_RSTACK_CONFIG_REQ:
		case LORAWAN_MSG_GET_SUPPORTED_BANDS_REQ:
		case LORAWAN_MSG_GET_DEVICE_EUI_REQ:
		case LORAWAN_MSG_GET_CUSTOM_CFG_REQ:
		case LORAWAN_MSG_GET_LINKADRREQ_CONFIG_REQ:
		case LORAWAN_MSG_FACTORY_RESET_REQ:
		case LORAWAN_MSG_DEACTIVATE_DEVICE_REQ:
		case LORAWAN_MSG_GET_NWK_STATUS_REQ:
			TxMessage.Length = 0;
			break;
		case LORAWAN_MSG_SET_CUSTOM_CFG_REQ:
		case LORAWAN_MSG_SET_LINKADRREQ_CONFIG_REQ:
			TxMessage.Length = 1;
			break;
		case LORAWAN_MSG_SET_JOIN_PARAM_REQ:
			TxMessage.Length = 24;
			break;
		case LORAWAN_MSG_SEND_UDATA_REQ:
		case LORAWAN_MSG_SEND_CDATA_REQ:
		case LORAWAN_MSG_SEND_MAC_CMD_REQ:
			TxMessage.Length = len;
			break;
		case LORAWAN_MSG_SET_RSTACK_CONFIG_REQ:
			TxMessage.Length = 7;
			break;
		case LORAWAN_MSG_SET_DEVICE_EUI_REQ:
			TxMessage.Length = 8;
			break;
		default:
			return DEVMGMT_STATUS_CMD_NOT_SUPPORTED;
	}

	if(TxMessage.Length > 0)
		memcpy(TxMessage.Payload, val, TxMessage.Length);

	ret = WiMOD_HCI_SendMessage(&TxMessage);

//	if(ret)
//		WiMOD_HCI_Process();

	return ret;
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_GetCustomConfig
//
//  @brief: Get custom config
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetCustomConfig(void)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_CUSTOM_CFG_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_SetCustomConfig
//
//  @brief: set custom config
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SetCustomConfig(int8_t val)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SET_CUSTOM_CFG_REQ, (uint8_t*) &val, 1);
}

//------------------------------------------------------------------------------
//
//  SetOPMODE
//
//  @brief: set OPMODE
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SetOPMODE(uint8_t val)
{
	if (val > 1) {
		return -1;
	}
	return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_SET_OPMODE_REQ, &val, 1);
}

//------------------------------------------------------------------------------
//
//  GetOPMODE
//
//  @brief: get OPMODE
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetOPMODE()
{
	return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_OPMODE_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  ClearRTCAlarm
//
//  @brief: clear RTC Alarm
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_ClearRTCAlarm()
{
	return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetRTCAlarm
//
//  @brief: get RTC Alarm
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetRTCAlarm()
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_RTC_ALARM_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  SetRTC
//
//  @brief: set RTC
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SetRTCAlarm(uint8_t* alarm)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_SET_RTC_ALARM_REQ, alarm, 4);
}


//------------------------------------------------------------------------------
//
//  GetRTC
//
//  @brief: get RTC
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetRTC()
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_RTC_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  SetRTC
//
//  @brief: set RTC
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SetRTC(uint32_t* time)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_SET_RTC_REQ, (uint8_t *) time, 4);
}

//------------------------------------------------------------------------------
//
//  SendPing
//
//  @brief: ping device
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SendPing()
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_PING_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetDeviceInfo
//
//  @brief: get device information
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetDeviceInfo(void)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_DEVICE_INFO_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetFirmwareVersion
//
//  @brief: get firmware version
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetFirmwareVersion(void)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_FW_VERSION_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_GetNkwStatus
//
//  @brief: get network status
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetNkwStatus(void)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_NWK_STATUS_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_FactoryReset
//
//  @brief: factory reset
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_FactoryReset(void)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_FACTORY_RESET_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_GetLinkADR
//
//  @brief: GetLinkADR
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetLinkADR(void)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_LINKADRREQ_CONFIG_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_SetLinkADR
//
//  @brief: SetLinkADR
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_SetLinkADR(uint8_t * payload)
{
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SET_LINKADRREQ_CONFIG_REQ, payload, 1);
}

// set mac cmd
int
WiMOD_LoRaWAN_SetMAC_CMD(uint8_t * payload){
	return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SEND_MAC_CMD_REQ, payload, 16);
}

//------------------------------------------------------------------------------
//
//  Reset
//
//  @brief: reset device
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_Reset(void)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_RESET_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetDeviceStatus
//
//  @brief: get device status
//
//------------------------------------------------------------------------------
int
WiMOD_LoRaWAN_GetDeviceStatus(void)
{
    return WiMOD_DevMgmt_Msg_Req(DEVMGMT_MSG_GET_DEVICE_STATUS_REQ, NULL, 0);
}


//------------------------------------------------------------------------------
//
//  JoinNetworkRequest
//
//  @brief: send join radio message
//
//------------------------------------------------------------------------------

int
WiMOD_LoRaWAN_JoinNetworkRequest(void)
{
    return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_JOIN_NETWORK_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  SetJoinParamRequest
//
//  @brief: send set join param message
//
//------------------------------------------------------------------------------

int
WiMOD_LoRaWAN_SetJoinParamRequest(UINT8* data, int len)
{
    return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SET_JOIN_PARAM_REQ, data, len);
}

//------------------------------------------------------------------------------
//
//  SendURadioData
//
//  @brief: send unconfirmed radio message
//
//------------------------------------------------------------------------------

int
WiMOD_LoRaWAN_SendURadioData(UINT8  port,       // LoRaWAN Port
                             UINT8* srcData,    // application payload
                             int    srcLength)  // length of application payload
{
	uint8_t payload[WIMOD_HCI_MSG_PAYLOAD_SIZE] = { 0 };

		// 1. check length
	    if (srcLength > (WIMOD_HCI_MSG_PAYLOAD_SIZE - 1))
	    {
	        // overflow error
	        return -1;
	    }

	    // 3.  init payload
	    // 3.1 init port
	    payload[0] = port;

	    // 3.2 init radio message payload
	    memcpy(&payload[1], srcData, srcLength);

	    return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SEND_UDATA_REQ, payload, 1 + srcLength);
}

//------------------------------------------------------------------------------
//
//  SendCRadioData
//
//  @brief: send confirmed radio message
//
//------------------------------------------------------------------------------

int
WiMOD_LoRaWAN_SendCRadioData(UINT8  port,       // LoRaWAN Port
                             UINT8* srcData,    // application data
                             int    srcLength)  // length of application data
{
	uint8_t payload[WIMOD_HCI_MSG_PAYLOAD_SIZE] = { 0 };

	// 1. check length
    if (srcLength > (WIMOD_HCI_MSG_PAYLOAD_SIZE - 1))
    {
        // overflow error
        return -1;
    }

    // 3.  init payload
    // 3.1 init port
    payload[0] = port;

    // 3.2 init radio message payload
    memcpy(&payload[1], srcData, srcLength);

    return WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SEND_CDATA_REQ, payload, 1 + srcLength);

}

//------------------------------------------------------------------------------
//
//  Process
//
//  @brief: handle receiver process
//
//------------------------------------------------------------------------------

void
WiMOD_LoRaWAN_Process(void)
{
    // call HCI process
//	SLIP_DecodeData(rxBuf_WiMOD, 1);
//	msg_state = UNKNOWN;
//	memset(rxBuf_WiMOD, 0x0, 255);
	WiMOD_HCI_Process();
}

//void
//WiMOD_LoRaWAN_ReadPayload(void)
//{
//	HAL_StatusTypeDef ret;
//	ret = SerialDevice_ReadData(rxBuf_WiMOD, 1);
//	msg_state = PAYLOAD;
//}
//------------------------------------------------------------------------------
//
//  Process
//
//  @brief: handle receiver process
//
//------------------------------------------------------------------------------

static TWiMOD_HCI_Message*
WiMOD_LoRaWAN_Process_RxMessage(TWiMOD_HCI_Message*  rxMessage)
{
    switch(rxMessage->SapID)
    {
        case DEVMGMT_SAP_ID:
            WiMOD_LoRaWAN_Process_DevMgmt_Message(rxMessage);
            break;


        case LORAWAN_SAP_ID:
            WiMOD_LoRaWAN_Process_LoRaWAN_Message(rxMessage);
            break;
    }

    return &RxMessage;
}

//------------------------------------------------------------------------------
//
//  Process_DevMgmt_Message
//
//  @brief: handle received Device Management SAP messages
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_Process_DevMgmt_Message(TWiMOD_HCI_Message*  rxMessage)
{
    switch(rxMessage->MsgID)
    {
        case    DEVMGMT_MSG_PING_RSP:
                WiMOD_LoRaWAN_ShowResponse("ping response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);
                break;
        case    DEVMGMT_MSG_GET_DEVICE_INFO_RSP:
        		WiMOD_LoRaWAN_DevMgmt_DeviceInfo_Rsp(rxMessage);
                break;
        case    DEVMGMT_MSG_GET_FW_VERSION_RSP:
				WiMOD_LoRaWAN_DevMgmt_FirmwareVersion_Rsp(rxMessage);
				break;
        case    DEVMGMT_MSG_RESET_RSP:
				WiMOD_LoRaWAN_ShowResponse("reset response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);
				break;
        case    DEVMGMT_MSG_GET_DEVICE_STATUS_RSP:
				WiMOD_LoRaWAN_DevMgmt_DeviceStatus_Rsp(rxMessage);
				break;
        case    DEVMGMT_MSG_GET_RTC_RSP:
        		WiMOD_LoRaWAN_DevMgmt_Get_RTC_Rsp(rxMessage);
				break;
        case    DEVMGMT_MSG_SET_RTC_RSP:
				WiMOD_LoRaWAN_ShowResponse("set RTC response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);
				break;
        case    DEVMGMT_MSG_GET_RTC_ALARM_RSP:
				WiMOD_LoRaWAN_DevMgmt_Get_RTC_ALARM_Rsp(rxMessage);
				break;
		case    DEVMGMT_MSG_SET_RTC_ALARM_RSP:
				WiMOD_LoRaWAN_ShowResponse("set RTC Alarm response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);
				break;
		case    DEVMGMT_MSG_CLEAR_RTC_ALARM_RSP:
				WiMOD_LoRaWAN_ShowResponse("clear RTC Alarm response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);
				break;
		case    DEVMGMT_MSG_GET_OPMODE_RSP:
				WiMOD_LoRaWAN_DevMgmt_Get_OPMODE_Rsp(rxMessage);
				break;

		default:
                printf("Unhandled DeviceMgmt message received - MsgID : 0x%02X\n\r", (UINT8)rxMessage->MsgID);
                break;
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_Get_OPMODE_Rsp
//
//  @brief: Get OPMODE
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_DevMgmt_Get_OPMODE_Rsp(TWiMOD_HCI_Message* rxMessage)
{
	WiMOD_LoRaWAN_ShowResponse("get OPMODE response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
	{
		printf("OPMODE Set: 0x");
		printf((const char*) num2hex((uint32_t)rxMessage->Payload[1], BYTE_F));
		printf("\n\r");
	}
}
//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_Get_RTC_ALARM_Rsp
//
//  @brief: Get RTC alarm
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_DevMgmt_Get_RTC_ALARM_Rsp(TWiMOD_HCI_Message* rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help;
	WiMOD_LoRaWAN_ShowResponse("get RTC ALARM response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
	{
		printf("RTC ALARM Set: 0x");
		printf((const char*) num2hex((uint32_t)rxMessage->Payload[1], BYTE_F));
		printf("\n\r");

		printf("RTC ALARM Daily: 0x");
		printf((const char*) num2hex((uint32_t)rxMessage->Payload[2], BYTE_F));
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[3], 3);
		printf("RTC ALARM Time: [");
		num2str((help & 0xFF), str);	//Hour
		printf((const char*) str);
		printf(":");
		num2str((help & 0xFF00) >> 8, str);	//Minute
		printf((const char*) str);
		printf(":");
		num2str((help & 0xFF0000) >> 16, str);	//second
		printf((const char*) str);

		printf("]\n\r");
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_Get_RTC_Rsp
//
//  @brief: Get RTC
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_DevMgmt_Get_RTC_Rsp(TWiMOD_HCI_Message* rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help;
	WiMOD_LoRaWAN_ShowResponse("get RTC response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
	{
		memcpy((uint8_t *) &help, &rxMessage->Payload[1], 4);
		printf("RTC Time: ");
		num2str(((help & 0xfc000000) >> 26) + 2000, str); //Year
		printf((const char*) str);
		printf("-");
		num2str(((help & 0xf000) >> 12), str);	//Months
		printf((const char*) str);
		printf("-");
		num2str(((help & 0x3e00000) >> 21), str);	//Day
		printf((const char*) str);
		printf(" [");
		num2str(((help & 0x1f0000) >> 16), str);	//Hour
		printf((const char*) str);
		printf(":");
		num2str(((help & 0xfc0) >> 6), str);	//Minutes
		printf((const char*) str);
		printf(":");
		num2str(((help & 0x3f)), str);	//Seconds
		printf((const char*) str);
		printf("]\n\r");
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_DeviceStatus_Rsp
//
//  @brief: show device status
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_DevMgmt_DeviceStatus_Rsp(TWiMOD_HCI_Message* rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help;
	WiMOD_LoRaWAN_ShowResponse("device status response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
	{
		memcpy((uint8_t *) &help, &rxMessage->Payload[1], 1);
		num2str(help & 0xFF, str);
		printf("System Tick Resolution: ");
		printf((const char* ) str);
		printf(" ms\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[2], 4);
		num2str(help, str);
		printf("System Tick: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[6], 4);
		printf("Target Time: ");
		num2str(((help & 0xfc000000) >> 26) + 2000, str); //Year
		printf((const char* ) str);
		printf("-");
		num2str(((help & 0xf000) >> 12), str);	//Months
		printf((const char* ) str);
		printf("-");
		num2str(((help & 0x3e00000) >> 21), str);	//Day
		printf((const char* ) str);
		printf(" [");
		num2str(((help & 0x1f0000) >> 16), str);	//Hour
		printf((const char* ) str);
		printf(":");
		num2str(((help & 0xfc0) >> 6), str);	//Minutes
		printf((const char* ) str);
		printf(":");
		num2str(((help & 0x3f)), str);	//Seconds
		printf((const char* ) str);
		printf("]\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[10], 2);
		num2str(help & 0xFFFF, str);
		printf("NVM Status: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[12], 2);
		num2str(help & 0xFFFF, str);
		printf("Battery level: ");
		printf((const char* ) str);
		printf(" mV\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[16], 4);
		num2str(help, str);
		printf("TX U-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[20], 4);
		num2str(help, str);
		printf("TX C-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[24], 4);
		num2str(help, str);
		printf("TX Error: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[28], 4);
		num2str(help, str);
		printf("RX1 U-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[32], 4);
		num2str(help, str);
		printf("RX1 C-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[36], 4);
		num2str(help, str);
		printf("RX1 MIC-Error: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[40], 4);
		num2str(help, str);
		printf("RX2 U-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[44], 4);
		num2str(help, str);
		printf("RX2 C-Data: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[48], 4);
		num2str(help, str);
		printf("RX2 MIC-Error: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[52], 4);
		num2str(help, str);
		printf("TX Join: ");
		printf((const char* ) str);
		printf("\n\r");

		memcpy((uint8_t *) &help, &rxMessage->Payload[56], 4);
		num2str(help, str);
		printf("RX Accept: ");
		printf((const char* ) str);
		printf("\n\r");

	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_DeviceInfo_Rsp
//
//  @brief: show device information
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_DevMgmt_DeviceInfo_Rsp(TWiMOD_HCI_Message*  rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help;
    WiMOD_LoRaWAN_ShowResponse("device information response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

    if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
    {
    	WiMOD_LoRaWAN_ShowResponse("Module type", WiMOD_DeviceMgmt_ModuleTypes, rxMessage->Payload[1]);

        memcpy((uint8_t *) &help, &rxMessage->Payload[2], 4);

        printf("Device address: 0x");
        printf((const char* ) num2hex((uint32_t)help, DOUBLEWORD_F));
        printf("\n\r");

        memcpy((uint8_t *) &help, &rxMessage->Payload[6], 4);
        printf("Device ID: 0x");
		printf((const char* ) num2hex((uint32_t)help, DOUBLEWORD_F));
		printf("(");
		num2str(help, str);
		printf((const char* ) str);
		printf(")\n\r");
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_DevMgmt_FirmwareVersion_Rsp
//
//  @brief: show firmware version
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_DevMgmt_FirmwareVersion_Rsp(TWiMOD_HCI_Message*  rxMessage)
{
    char help[80];

    WiMOD_LoRaWAN_ShowResponse("firmware version response", WiMOD_DeviceMgmt_StatusStrings, rxMessage->Payload[0]);

    if (rxMessage->Payload[0] == DEVMGMT_STATUS_OK)
    {

    	printf("version: V%d.%d\n\r", (int)rxMessage->Payload[2], (int)rxMessage->Payload[1]);
		printf("build-count: %d\n\r", (int)MAKEWORD(rxMessage->Payload[3], rxMessage->Payload[4]));

        memcpy(help, &rxMessage->Payload[5], 10);
        help[10] = 0;
        printf("build-date: %s\n\r", help);

        // more information attached ?
        if (rxMessage->Length > 15)
        {
            // add string termination
            rxMessage->Payload[rxMessage->Length] = 0;
            printf("firmware-content: %s\n\r", &rxMessage->Payload[15]);
        }
    }
}

//------------------------------------------------------------------------------
//
//  Process_LoRaWAN_Message
//
//  @brief: handle received LoRaWAN SAP messages
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_Process_LoRaWAN_Message(TWiMOD_HCI_Message*  rxMessage)
{
    switch(rxMessage->MsgID)
    {
    	case 	LORAWAN_MSG_ACTIVATE_DEVICE_RSP:
    			WiMOD_LoRaWAN_ShowResponse("activate device response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
    			break;

    	case 	LORAWAN_MSG_REACTIVATE_DEVICE_RSP:
    			WiMOD_LoRaWAN_Process_Reactivate_Rsp(rxMessage);
    	    	break;

    	case 	LORAWAN_MSG_DEACTIVATE_DEVICE_RSP:
    		WiMOD_LoRaWAN_ShowResponse("deactivate device response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
				break;

        case    LORAWAN_MSG_JOIN_NETWORK_RSP:
                WiMOD_LoRaWAN_ShowResponse("join network response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
                break;

        case    LORAWAN_MSG_SET_JOIN_PARAM_RSP:
                WiMOD_LoRaWAN_ShowResponse("set join param response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
                break;

        case    LORAWAN_MSG_SEND_UDATA_RSP:
                WiMOD_LoRaWAN_ShowResponse("send U-Data response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
                break;

        case    LORAWAN_MSG_SEND_UDATA_IND:
        		WiMOD_LoRaWAN_Process_U_DATA_TxIndication(rxMessage);
        		break;

        case    LORAWAN_MSG_SEND_CDATA_RSP:
                WiMOD_LoRaWAN_ShowResponse("send C-Data response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
                break;

        case	LORAWAN_MSG_SEND_CDATA_IND:
        		WiMOD_LoRaWAN_Process_C_DATA_TxIndication(rxMessage);
				break;

        case    LORAWAN_MSG_JOIN_TRANSMIT_IND:
                WiMOD_LoRaWAN_Process_JoinTxIndication(rxMessage);
                break;

        case    LORAWAN_MSG_JOIN_NETWORK_IND:
                WiMOD_LoRaWAN_Process_JoinNetworkIndication(rxMessage);
                break;

        case    LORAWAN_MSG_RECV_UDATA_IND:
                WiMOD_LoRaWAN_Process_U_DataRxIndication(rxMessage);
                break;

        case    LORAWAN_MSG_RECV_CDATA_IND:
                WiMOD_LoRaWAN_Process_C_DataRxIndication(rxMessage);
                break;

        case	LORAWAN_MSG_SEND_MAC_CMD_RSP:
        		WiMOD_LoRaWAN_ShowResponse("send MAC CMD response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

				break;
        case    LORAWAN_MSG_RECV_MAC_CMD_IND:
        		WiMOD_LoRaWAN_Process_MAC_CMDRxIndication(rxMessage);
        		break;

        case    LORAWAN_MSG_RECV_NODATA_IND:
                printf("no data received indication\n\r");
                break;

        case	LORAWAN_MSG_SET_RSTACK_CONFIG_RSP:
        		WiMOD_LoRaWAN_Process_Set_RSTACK_RSP(rxMessage);
        	break;

        case	LORAWAN_MSG_GET_RSTACK_CONFIG_RSP:
               		WiMOD_LoRaWAN_Process_Get_RSTACK_RSP(rxMessage);
               	break;

        case	LORAWAN_MSG_GET_SUPPORTED_BANDS_RSP:
        	WiMOD_LoRaWAN_Process_Get_Support_Band_RSP(rxMessage);
        	break;

        case 	LORAWAN_MSG_GET_DEVICE_EUI_RSP:
        	WiMOD_LoRaWAN_Process_Get_Device_EUI_RSP(rxMessage);
        	break;

        case LORAWAN_MSG_GET_CUSTOM_CFG_RSP:
        	WiMOD_LoRaWAN_Process_Get_Custom_CFG_RSP(rxMessage);
        	break;

        case LORAWAN_MSG_SET_CUSTOM_CFG_RSP:
        	 WiMOD_LoRaWAN_ShowResponse("set custom cfg response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
        	 break;

        case LORAWAN_MSG_FACTORY_RESET_RSP:
        	 WiMOD_LoRaWAN_ShowResponse("factory reset response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
			 break;

        case LORAWAN_MSG_GET_NWK_STATUS_RSP:
        	WiMOD_LoRaWAN_Process_Get_Network_Status_RSP(rxMessage);
        	break;

        case LORAWAN_MSG_GET_LINKADRREQ_CONFIG_RSP:
        	WiMOD_LoRaWAN_Process_Get_LINK_ADR_RSP(rxMessage);
        	break;

        case LORAWAN_MSG_SET_LINKADRREQ_CONFIG_RSP:
        	WiMOD_LoRaWAN_ShowResponse("set link adr response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
            break;

        default:
        		printf("Unhandled LoRaWAN SAP message received - MsgID : 0x");
        		printf(num2hex(rxMessage->MsgID, BYTE_F));
        		printf("\n\r");
                break;
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Reactivate_Rsp
//
//  @brief: reactivate device
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_Process_Reactivate_Rsp(TWiMOD_HCI_Message*  rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help = 0;
    WiMOD_LoRaWAN_ShowResponse("reactivate device response", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

    if (rxMessage->Payload[0] == LORAWAN_STATUS_OK)
    {
    	memcpy((uint8_t*) &help, &rxMessage->Payload[1], 4);
    	num2str(help, str);	//Second
    	printf("Device address: 0x");
		printf((const char* ) num2hex((uint32_t)help, DOUBLEWORD_F));
		printf("(");
		printf((const char* ) str);
		printf(")\n\r");
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_C_DATA_TxIndication
//
//  @brief: unconfirm data transmit indicaton
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_C_DATA_TxIndication(TWiMOD_HCI_Message* rxMessage)
{
	if (rxMessage->Payload[0] == 0)
    {
        printf("c data tx event : radio packet sent\n\r");
    }
    // channel info attached ?
    else if(rxMessage->Payload[0] == 1)
    {
    	printf("c data tx event : radio packet sent with tx channel info attached\n\r");
    	printf("c data tx event ChIdx %d\n\r", (int)rxMessage->Payload[1]);
    	printf("\tDR: %d\n\r", (int)rxMessage->Payload[2]);
    	printf("\tNum: %d\n\r", (int)rxMessage->Payload[3]);
    	printf("\tPower(dBm): %d\n\r", (int)rxMessage->Payload[4]);
    	printf("\tAirtime (ms): %ld\n\r", (uint32_t *) &rxMessage->Payload[5]);

    }
    else if (rxMessage->Payload[0] & 0x02)
		        printf("error, maximum number of retransmissions reached\n\r");
    else if (rxMessage->Payload[0] & 0x04)
		        printf("maximum payload size exceeded for current data rate\n\r");
    else
    {
        printf("c data tx event - Status : error\n\r");
    }

}
//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_U_DATA_TxIndication
//
//  @brief: unconfirm data transmit indicaton
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_U_DATA_TxIndication(TWiMOD_HCI_Message* rxMessage)
{
	if (rxMessage->Payload[0] == 0)
    {
        printf("u data tx event : radio packet sent\n\r");
    }
    // channel info attached ?
    else if(rxMessage->Payload[0] == 1)
    {
    	printf("u data tx event : radio packet sent with tx channel info attached\n\r");
    	printf("u data tx event ChIdx %d\n\r", (int)rxMessage->Payload[1]);
    	printf("\tDR: %d\n\r", (int)rxMessage->Payload[2]);
    	printf("\tNum: %d\n\r", (int)rxMessage->Payload[3]);
    	printf("\tPower(dBm): %d\n\r", (int)rxMessage->Payload[4]);
    	printf("\tAirtime (ms): %ld\n\r", (uint32_t *) &rxMessage->Payload[5]);

    }
    else
    {
        printf("u data tx event - Status : error\n\r");
    }
}
//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_JoinTxIndication
//
//  @brief: show join transmit indicaton
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_Process_JoinTxIndication(TWiMOD_HCI_Message* rxMessage)
{
    if (rxMessage->Payload[0] == 0)
    {
        printf("join tx event - Status : ok\n\r");
    }
    // channel info attached ?
    else if(rxMessage->Payload[0] == 1)
    {
        printf("join tx event:%d, ChnIdx:%d, DR:%d - Status : ok\n\r", (int)rxMessage->Payload[3], (int)rxMessage->Payload[1], (int)rxMessage->Payload[2]);
    }
    else
    {
        printf("join tx event - Status : error\n\r");
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_JoinNetworkIndication
//
//  @brief: show join network indicaton
//
//------------------------------------------------------------------------------

void
WiMOD_LoRaWAN_Process_JoinNetworkIndication(TWiMOD_HCI_Message* rxMessage)
{
    if (rxMessage->Payload[0] == 0)
    {
        UINT32 address = MAKELONG(MAKEWORD(rxMessage->Payload[1],rxMessage->Payload[2]),
                                  MAKEWORD(rxMessage->Payload[3],rxMessage->Payload[4]));

        printf("join network accept event - DeviceAddress:0x%08X\n\r", address);
    }
    else if (rxMessage->Payload[0] == 1)
    {
        UINT32 address = MAKELONG(MAKEWORD(rxMessage->Payload[1],rxMessage->Payload[2]),
                                  MAKEWORD(rxMessage->Payload[3],rxMessage->Payload[4]));

        printf("join network accept event - DeviceAddress:0x%08X, ChnIdx:%d, DR:%d, RSSI:%d, SNR:%d, RxSlot:%d\n\r", address,
               (int)rxMessage->Payload[5], (int)rxMessage->Payload[6], (int)rxMessage->Payload[7],
               (int)rxMessage->Payload[8], (int)rxMessage->Payload[9]);
    }
    else
    {
        printf("join network timeout event\n\r");
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_U_DataRxIndication
//
//  @brief: show received U-Data
//
//------------------------------------------------------------------------------

void
WiMOD_LoRaWAN_Process_U_DataRxIndication(TWiMOD_HCI_Message* rxMessage)
{
    int payloadSize = rxMessage->Length - 1;

    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
        payloadSize -= 5;

    if (payloadSize >= 1)
    {
    	printf("U-Data rx event: port:0x");
        printf(num2hex(rxMessage->Payload[1], BYTE_F));
        printf("\n\rapp-payload: 0x");
        for(int i = 1; i < payloadSize; i++)
        	printf(num2hex(rxMessage->Payload[1+i], BYTE_F));
        printf("\n\r");
    }

    if (rxMessage->Payload[0] & 0x02)
        printf("ack for uplink packet:yes\n\r");
    else
        printf("ack for uplink packet:no\n\r");

    if (rxMessage->Payload[0] & 0x04)
        printf("frame pending:yes\n\r");
    else
        printf("frame pending:no\n\r");

    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
    {
        UINT8* rxInfo = &rxMessage->Payload[1 + payloadSize];
        printf("ChnIdx:%d, DR:%d, RSSI:%d, SNR:%d, RxSlot:%d\n\r",
              (int)rxInfo[0], (int)rxInfo[1], (int)rxInfo[2],
              (int)rxInfo[3], (int)rxInfo[4]);
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_LINK_ADR_RSP
//
//  @brief: Get LINK ADR ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_LINK_ADR_RSP(TWiMOD_HCI_Message* rxMessage)
{
	WiMOD_LoRaWAN_ShowResponse(" Get Network Status ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

		if (rxMessage->Payload[0] == LORAWAN_STATUS_OK) {
			printf("LinkADRReq Option: ");
			switch (rxMessage->Payload[1]) {
				case 0:
					printf("LoRaWAN v1.0.2");
					break;
				case 1:
					printf("Semtech proposal");
									break;
				case 2:
					printf("KPN/Actility proposal");
									break;
				default:
					printf("Unknown proposal");
					break;
			}
			printf("\n\r");
		}


}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_Network_Status_RSP
//
//  @brief: Get Network Status ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_Network_Status_RSP(TWiMOD_HCI_Message* rxMessage)
{
	uint8_t str[8] = {0};
		uint32_t help;
	WiMOD_LoRaWAN_ShowResponse(" Get Network Status ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == LORAWAN_STATUS_OK) {

		printf("Network Status: 0x%02X\n\r", rxMessage->Payload[1]);
		switch (rxMessage->Payload[1]) {
		case	0x01:
			printf("active (ABP)\n\r");
			break;
		case	0x02:
			printf("active (OTAA)\n\r");
			break;
		case	0x03:
			printf("joining (OTAA)\n\r");
			break;
		default:
			printf("device inactive\n\r");
			return;
		}
		memcpy((uint8_t *) &help, &rxMessage->Payload[2], 4);

		printf("\tDevice address: 0x");
		printf((const char* ) num2hex((uint32_t)help, DOUBLEWORD_F));
		printf("\n\r");

		num2str(rxMessage->Payload[6],str);
		printf("\tDR: ");
		printf(str);
		printf("\n\r");

		num2str(rxMessage->Payload[7],str);
		printf("\tPower: ");
		printf(str);
		printf("dBm\n\r");

		num2str(rxMessage->Payload[8],str);
		printf("\tMax. payload size: ");
		printf(str);
		printf("\n\r");

	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_Custom_CFG_RSP
//
//  @brief: Get custom config ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_Custom_CFG_RSP(TWiMOD_HCI_Message* rxMessage)
{
	WiMOD_LoRaWAN_ShowResponse(" Get custom cfg ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
	if (rxMessage->Payload[0] == LORAWAN_STATUS_OK)
	{
		printf("RF Gain: %d dBd\n\r", (int) rxMessage->Payload[1]);
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_Device_EUI_RSP
//
//  @brief: Get device eui ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_Device_EUI_RSP(TWiMOD_HCI_Message* rxMessage)
{
	WiMOD_LoRaWAN_ShowResponse(" Get device eui ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);
	if (rxMessage->Payload[0] == LORAWAN_STATUS_OK)
	{
		printf("Device EUI: \n\r");
		for (int var = 1; var < rxMessage->Length; ++var) {
			printf(" %02X", rxMessage->Payload[var]);
		}
		printf("\n\r");
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_Support_Band_RSP
//
//  @brief: Get support band ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_Support_Band_RSP(TWiMOD_HCI_Message* rxMessage)
{

	WiMOD_LoRaWAN_ShowResponse(" Get support band ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == LORAWAN_STATUS_OK) {
		for (int var = 1; var < rxMessage->Length; var += 2) {
			printf("Band Idx: %d\n\r", rxMessage->Payload[var]);
			printf("Max. EIRP: %d dBm\n\r", rxMessage->Payload[1+var]);
		}
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Set_RSTACK_RSP
//
//  @brief: Set Radio STACK ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Set_RSTACK_RSP(TWiMOD_HCI_Message* rxMessage)
{
	WiMOD_LoRaWAN_ShowResponse("Set Radio STACK ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

	if (rxMessage->Payload[0] == LORAWAN_STATUS_WRONG_PARAMETER) {

		if (rxMessage->Payload[1] & 0x01) {
			printf("Wrong Data Rate\n\r");
		}
		if (rxMessage->Payload[1] & 0x02) {
			printf("Wrong TX Power Level\n\r");
		}
		if (rxMessage->Payload[1] & 0x20) {
			printf("Wrong Band Index\n\r");
		}
	}
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_Get_RSTACK_RSP
//
//  @brief: Get Radio STACK ReSPonse
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_Get_RSTACK_RSP(TWiMOD_HCI_Message* rxMessage)
{
	uint8_t str[8] = {0};
	uint32_t help;
	WiMOD_LoRaWAN_ShowResponse("Get Radio STACK ReSPonse", WiMOD_LoRaWAN_StatusStrings, rxMessage->Payload[0]);

	printf("Data Rate: ");
	num2str(rxMessage->Payload[1], str);
	printf(str);
	printf("\n\r");

	printf("TX Power Level (EIRP): ");
	num2str(rxMessage->Payload[2], str);
	printf(str);
	printf("\n\r");

	printf("Adaptive DR: ");
	if (rxMessage->Payload[3] & 0x1) {
		printf("enable\n\r");
	} else {
		printf("disable\n\r");
	}
	printf("Duty Cycle: ");
	if (rxMessage->Payload[3] & 0x2) {
		printf("enable\n\r");
	} else {
		printf("disable\n\r");
	}
	printf("Class : ");
	if (rxMessage->Payload[3] & 0x4) {
		printf("C\n\r");
	} else {
		printf("A\n\r");
	}
	printf("RF format : ");
	if (rxMessage->Payload[3] & 0x40) {
		printf("extended\n\r");
	} else {
		printf("standard\n\r");
	}
	printf("MAC forwarding : ");
	if (rxMessage->Payload[3] & 0x80) {
		printf("enable\n\r");
	} else {
		printf("disable\n\r");
	}


	printf("Power Saving Mode: ");
	if (rxMessage->Payload[4] & 0x1) {
		printf("automatic\n\r");
	} else {
		printf("off\n\r");
	}

	printf("Retransmissions: ");
	num2str(rxMessage->Payload[5], str);
	printf(str);
	printf("\n\r");

	printf("BandIdx: ");
	num2str(rxMessage->Payload[6], str);
	printf(str);
	printf("\n\r");

	printf("MAC length: ");
	num2str(rxMessage->Payload[7], str);
	printf(str);
	printf("\n\r");

}
//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_MAC_CMDRxIndication
//
//  @brief: show received MAC cmd
//
//------------------------------------------------------------------------------
static void
WiMOD_LoRaWAN_Process_MAC_CMDRxIndication(TWiMOD_HCI_Message* rxMessage)
{
    int payloadSize = rxMessage->Length - 1;
//
    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
        payloadSize -= 5;

    if (payloadSize >= 1)
    {
        printf("MAC rx event:\n\rmac-payload: 0x");
		for(int i = 0; i < payloadSize; i++)
			printf(num2hex(rxMessage->Payload[1+i], BYTE_F));
		printf("\n\r");
    }

//    if (rxMessage->Payload[0] & 0x02)
//        printf("ack for uplink packet:yes\n\r");
//    else
//        printf("ack for uplink packet:no\n\r");
//
//    if (rxMessage->Payload[0] & 0x04)
//        printf("frame pending:yes\n\r");
//    else
//        printf("frame pending:no\n\r");

    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
    {
        UINT8* rxInfo = &rxMessage->Payload[1 + payloadSize];
        printf("ChnIdx:%d, DR:%d, RSSI:%d, SNR:%d, RxSlot:%d\n\r",
              (int)rxInfo[0], (int)rxInfo[1], (int)rxInfo[2],
              (int)rxInfo[3], (int)rxInfo[4]);
    }
}
//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_Process_C_DataRxIndication
//
//  @brief: show received C-Data
//
//------------------------------------------------------------------------------

void
WiMOD_LoRaWAN_Process_C_DataRxIndication(TWiMOD_HCI_Message* rxMessage)
{
    int payloadSize = rxMessage->Length - 1;

    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
        payloadSize -= 5;

    if (payloadSize >= 1)
    {
        printf("C-Data rx event: port:0x%02X\n\rapp-payload:", rxMessage->Payload[1]);
        for(int i = 1; i < payloadSize;)
        	printf(num2hex(rxMessage->Payload[1+i], BYTE_F));
        printf("\n\r");
    }

    if (rxMessage->Payload[0] & 0x02)
        printf("ack for uplink packet:yes\n\r");
    else
        printf("ack for uplink packet:no\n\r");

    if (rxMessage->Payload[0] & 0x04)
        printf("frame pending:yes\n\r");
    else
        printf("frame pending:no\n\r");

    // rx channel info attached ?
    if (rxMessage->Payload[0] & 0x01)
    {
        UINT8* rxInfo = &rxMessage->Payload[1 + payloadSize];
        printf("ChnIdx:%d, DR:%d, RSSI:%d, SNR:%d, RxSlot:%d\n\r",
              (int)rxInfo[0], (int)rxInfo[1], (int)rxInfo[2],
              (int)rxInfo[3], (int)rxInfo[4]);
    }
}

//------------------------------------------------------------------------------
//
//  WiMOD_LoRaWAN_ShowResponse
//
//  @brief: show response status as human readable string
//
//------------------------------------------------------------------------------

static void
WiMOD_LoRaWAN_ShowResponse(const char* string, const TIDString* statusTable, UINT8 statusID)
{
    while(statusTable->String)
    {
        if (statusTable->ID == statusID)
        {
            printf(string);
            printf(" - Status(0x");
			printf((const char* ) num2hex(statusID, BYTE_F));
			printf(") ");
            printf(statusTable->String);
            printf("\n\r");
            return;
        }

        statusTable++;
    }
}
//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
