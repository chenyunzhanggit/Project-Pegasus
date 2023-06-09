#ifndef SABER_PROTOCOL_H
#define SABER_PROTOCOL_H


/* - MACRO definition start here - */
/* Data Packet Length*/
#define LINEARACC_LEN         23

#define ORIENTION_LEN         27

#define ACC_RAW_LEN           17
#define ACC_CAL_LEN           23
#define ACC_KAL_LEN           23

#define GYRO_RAW_LEN          17
#define GYRO_CAL_LEN  		  23
#define GYRO_KAL_LEN  		  23

#define GNSS_PVT_LEN          92

#define SABER_EMPTY_LEN       8
#define SABER_HEAD_LEN        6
#define SABER_TAIL_LEN        2
/*
 ************************************************************************
 * Definition for Protocol
 ************************************************************************
 */



#define MADDR_OUT                               0xFF

#define RING_BUFFER_SIZE                        (1024 * 2)
#define FILE_BUFFER_SIZE                        (1024 * 2)


#define MAX_RECV_BYTES                          (2048*2)
#define PRINT_SIZE                              (2*MAX_RECV_BYTES)

#define PACKET_PL_INDEX                         2
#define PACKET_DATA_INDEX                       3

enum mode
{
	NOT_CONNNECT_MODE,
	CONNECTED_MODE,
	WAKEUP_MODE,
	CONFIG_MODE,
	MEASURE_MODE,
	DEBUG_MODE,
	MAG_CALIBRATE_MODE,
};

enum parserCode
{
	ERROR_NOT_ENOUGH_LENGTH = 10,
	ERROR_CRC_FAIL,
	ERROR_DECODE_FAIL,
	ERROR_DUPLICATED_FRAME,
};

enum frameType
{
	FRAME_COMPLETE = 1,
	FRAME_ERROR,
	NOT_FRAME,
};
extern void Saber_SwitchModeReq(signed char  nFD, char mode);
extern void sendPacket(signed char  nFD, u8 MADDR, u8 classID, u8 msgID, u8 res, u8* payloadData, u16 payloadLen);
extern void Saber_SetDataPacketConfigReq(signed char  nFD, u8* pData, u8 dataLen);
extern void Saber_setPacktUpdateRateReq(signed char  nFD, u8 *setRate, u8 dataLen_2);
extern void Saber_GetGNSSReceiverParaReq(signed char  nFD,u8* pData, u8 dataLen);

#endif // ATOMPROTOCOL_H
