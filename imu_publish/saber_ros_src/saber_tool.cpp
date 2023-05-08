/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Atom-Robotics Corporation. All rights reserved.
 *  Author: niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Saber Measure Mode get data stream tools  on ROS/Linux
 *--------------------------------------------------------------------------------------------*/
#include "../saber_ros_inc/saber_tool.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "../main.h"
#include "../saber_ros_inc/saber_macro.h"
#include "../saber_ros_inc/saber_serial.h"
#include "../saber_ros_inc/saber_protocol.h"

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberAlign
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Saber measure data packet align on the serial stream
 *  Parameter:   nFD = serialport file desriptor, used by system call ,such as read,write
 *  Return:      32bit int = data packet length(bytes)           
 *--------------------------------------------------------------------------------------------*/
void Printstr(unsigned int len,unsigned int tmp2)
{

    while(len>0)
    {
      len--;
      printf("%02X ",tmp2);
    }

}

int SaberAlign(unsigned char nFD, unsigned char * buffer)
{
    unsigned char tmp = 0;
    unsigned int uLen = 0;
    int packLength = 0;
    unsigned int tmp_i = 0;

    while(1)
    {        
        uLen=read(nFD,&tmp,1);   
        //Printstr(uLen,tmp);   
        if(tmp == 0x41)  //Judge flag byte
        {
              unsigned int keylen=4;
              unsigned int StrFlag=1;
              while(keylen && StrFlag)  
              {
                 uLen=read(nFD,&tmp_i,1);  
                 switch(keylen)
                 {
                    case 4:if(tmp_i!=0x78) StrFlag=0;break;
                    case 3:if(tmp_i!=0xff) StrFlag=0;break;
                    case 2:if(tmp_i!=0x06) StrFlag=0;break;
                    case 1:if(tmp_i!=0x81) StrFlag=0;break;
                 }
                 //printf(";head1:");
                 //Printstr(uLen,tmp_i);
                 keylen--;
              }

            if(StrFlag)
            {
                uLen=read(nFD,&tmp_i,1);//atom20221102 tmp->tmp_i
                packLength = tmp_i;
                buffer[0] = 0x41;
                buffer[1] = 0x78;
                buffer[2] = 0xff;
                buffer[3] = 0x06;
                buffer[4] = 0x81;
                buffer[5] = packLength;

                //Read packet

               // uLen = read(nFD, &buffer[6], packLength + SABER_TAIL_LEN);
                unsigned int StrLen=packLength+SABER_TAIL_LEN; 
                unsigned int StartPoint=6;
                while(StrLen)  //Byte by byte reading to reduce loss rate
                {
                     uLen=read(nFD,&buffer[StartPoint],1);//atom20221102 tmp->tmp_i
                     StrLen--;
					 StartPoint++;
                     
                }
                if(buffer[packLength + SABER_EMPTY_LEN - 1] != 0x6D)
                    packLength = 0;

                break;
            }
        }
        
    }

    uLen = 0;
    return  packLength;
}

u8 SaberGetGnssmod(unsigned char nFD)
{
    unsigned char tmp = 0;
    //unsigned char tmp1 = 0;
    unsigned int uLen = 0;
    u8 Gnssmod = 0;
    unsigned int tmp_i = 0;

    while(1)
    {

        uLen=read(nFD,&tmp,1);
        if(tmp == 0x41)
        {

            uLen=read(nFD,&tmp_i,4);
			if(tmp_i == 0xB206FF78)
			{
				uLen=read(nFD,&tmp,1);
                if(tmp == 2)
				{
					unsigned char temp_buff[4];
					read(nFD,temp_buff,4);
					Gnssmod = temp_buff[0];
			        printf("GNSS Mod:%d \n",temp_buff[0]);
					break;
				}
			}
        }
    }
    uLen = 0;
    return  Gnssmod;
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberGetFrame
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.24
 *  Description: Get a frame of Saber measure data packet
 *  Parameter:   nFD = serialport file desriptor, used by system call ,such as read,write
 *               tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
int SaberGetFrame(unsigned char nFD, unsigned char * tmpBuf, int frameLen){
    int pkgLen = frameLen;
    int uLen = 0;
    int tLen = 0;
    memset(tmpBuf,0,pkgLen);
    while(uLen < pkgLen) {
        tLen = read(nFD, tmpBuf+uLen, pkgLen-uLen);
        uLen += tLen;
    }
    return uLen;
}
/*---------------------------------------------------------------------------------------------
 *  Function:    SaberValidFrame
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Valid a frame of Saber measure data packet
 *  Parameter:   tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
bool SaberValidFrame(unsigned char * tmpBuf,int frameLen)
{
    return  (tmpBuf[0] == 0x41) &
            (tmpBuf[1] == 0x78) &
            (tmpBuf[2] == 0xFF) &
            (tmpBuf[3] == 0x06) &
            (tmpBuf[4] == 0x81) &
            (tmpBuf[frameLen-1] == 0x6D);
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberFillFrameHead
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Fill the frame head of Saber measure data packet losted by SaberAlign, using couple of  
 *               SaberAlign(unsigned char nFD),such as:
 *               
 *               SaberAlign(nFD);
 *               SaberFillFrameHead(dataBuf);
 *                               
 *  Parameter:   tmpBuf = buffer store the frame of Saber measure data packet 
 *               frameLen = data packet length(bytes)
 *  Return:      32bit int =  data packet length(bytes) equal frameLen usually          
 *--------------------------------------------------------------------------------------------*/
int  SaberFillFrameHead(unsigned char * tmpBuf, unsigned char len)
{
    tmpBuf[0] = 0x41;
    tmpBuf[1] = 0x78;
    tmpBuf[2] = 0xFF;
    tmpBuf[3] = 0x06;
    tmpBuf[4] = 0x81;
    tmpBuf[5] = len;
    return 0;
}

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberParserDataPacket
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.23
 *  Description: Parser a frame of Saber measure data packet
 *  Parameter:   saberDataHandle = data struct of all Saber measure data type,such as, linear_acc... 
 *               pBuffer = data buffer
 *               dataLen = data payload length(bytes)
 *               fpLog = a log file ,optional
 *  Return:      nothing        
 *--------------------------------------------------------------------------------------------*/

void SaberParserDataPacket(SaberData *saberDataHandle,u8 *pBuffer, u16 dataLen, FILE *fpLog)
{
    u16 PID = 0;
    u8 *pData = pBuffer;
    u8 index = 0;
    u8 pl = 0;

    //reset saberDataHandle
    memset(saberDataHandle, 0, sizeof(saberDataHandle));
    //printf("\n");
    while (index < dataLen)
    {
        
        PID = ((*((u16*)(pData + index))) & 0x7fff);
        pl = *(pData + index + 2);
        
        //printf(" pData: 0x%04x, index:%d, pid: 0x%04x, pl: %d\n", pData, index, PID, pl );

        //distance PID 0x3403
        if (PID == (SESSION_NAME_TEMPERATURE))
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->temperature.data, pData + index, PL_TEMPERTURE);
            saberDataHandle->temperature.dataID = PID;
            saberDataHandle->temperature.dataLen = pl;
            //printf(" *** temperature:\t%11.4f *** \n", saberDataHandle->temperature.data);

            index += pl;

        }
        else if (PID == (SESSION_NAME_RAW_ACC))
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accRawData.accX, pData + index, PL_RAW_DATA);
            saberDataHandle->accRawData.dataID = PID;
            saberDataHandle->accRawData.dataLen = pl;

            index += pl;

        }
        else if (PID == SESSION_NAME_RAW_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroRawData.gyroX, pData + index, PL_RAW_DATA);
            saberDataHandle->gyroRawData.dataID = PID;
            saberDataHandle->gyroRawData.dataLen = pl;
            index += pl;
        }
        else if (PID == SESSION_NAME_RAW_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magRawData.magX, pData + index, PL_RAW_DATA);
            saberDataHandle->magRawData.dataID = PID;
            saberDataHandle->magRawData.dataLen = pl;
            index += pl;
        }
        else if (PID == SESSION_NAME_CAL_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accCal.accX, pData + index, PL_CAL_DATA);
            saberDataHandle->accCal.dataID = PID;
            saberDataHandle->accCal.dataLen = pl;
            index += pl;
            if(fpLog!=NULL)
                fprintf(fpLog," *** accCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accCal.accX, saberDataHandle->accCal.accY, saberDataHandle->accCal.accZ);
        }
        else if (PID == SESSION_NAME_CAL_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroCal.gyroX, pData + index, PL_CAL_DATA);

            saberDataHandle->gyroCal.dataID = PID;
            saberDataHandle->gyroCal.dataLen = pl;
            index += pl;
            if(fpLog!=NULL)
                fprintf(fpLog," *** gyroCal:    \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->gyroCal.gyroX, saberDataHandle->gyroCal.gyroY, saberDataHandle->gyroCal.gyroZ);
        }
        else if (PID == SESSION_NAME_CAL_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magCal.magX, pData + index, PL_CAL_DATA);
            saberDataHandle->magCal.dataID = PID;
            saberDataHandle->magCal.dataLen = pl;
            index += pl;

            //printf(" *** magCal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->magCal.magX, saberDataHandle->magCal.magY, saberDataHandle->magCal.magZ);
        }
        else if (PID == SESSION_NAME_KAL_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accKal.accX, pData + index, PL_KAL_DATA);
            saberDataHandle->accKal.dataID = PID;
            saberDataHandle->accKal.dataLen = pl;
            index += pl;
            if(fpLog!=NULL)
                fprintf(fpLog," *** accKal:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accKal.accX, saberDataHandle->accKal.accY, saberDataHandle->accKal.accZ);
        }
        else if (PID == SESSION_NAME_KAL_GYRO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->gyroKal.gyroX, pData + index, PL_KAL_DATA);
            saberDataHandle->gyroKal.dataID = PID;
            saberDataHandle->gyroKal.dataLen = pl;
            index += pl;
        }
        else if (PID == SESSION_NAME_KAL_MAG)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->magKal.magX, pData + index, PL_KAL_DATA);
            saberDataHandle->magKal.dataID = PID;
            saberDataHandle->magKal.dataLen = pl;
            index += pl;
        }
            //////////////////////////
        else if (PID == SESSION_NAME_QUAT)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->quat.Q0.uint_x, pData + index, PL_QUAT_EULER);
            saberDataHandle->quat.dataID = PID;
            saberDataHandle->quat.dataLen = pl;
            index += pl;
            if(fpLog!=NULL)
                fprintf(fpLog," *** quat :      \t%11.4f, %11.4f, %11.4f, %11.4f *** \n", saberDataHandle->quat.Q0.float_x, saberDataHandle->quat.Q1.float_x, saberDataHandle->quat.Q2.float_x, saberDataHandle->quat.Q3.float_x);

        }
        else if (PID == SESSION_NAME_EULER)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->euler.roll, pData + index, PL_QUAT_EULER);
            saberDataHandle->euler.dataID = PID;
            saberDataHandle->euler.dataLen = pl;
            index += pl;

            //printf(" *** euler:      \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->euler.roll, saberDataHandle->euler.pitch, saberDataHandle->euler.yaw);
        }

        else if (PID == SESSION_NAME_ROTATION_M)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->romatix.a, pData + index, PL_MATERIX);
            saberDataHandle->romatix.dataID = PID;
            saberDataHandle->romatix.dataLen = pl;
            index += pl;

        }

        else if (PID == SESSION_NAME_LINEAR_ACC)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->accLinear.accX, pData + index, PL_LINEAR_ACC_DATA);
            saberDataHandle->accLinear.dataID = PID;
            saberDataHandle->accLinear.dataLen = pl;
            index += pl;
            if(fpLog!=NULL)
                fprintf(fpLog," *** lin_acc:     \t%11.4f, %11.4f, %11.4f *** \n", saberDataHandle->accLinear.accX, saberDataHandle->accLinear.accY, saberDataHandle->accLinear.accZ);
        }

        else if (PID == SESSION_NAME_DELTA_T)
        {
            //Ignore pid and pl
            index += 3;
            memcpy(&saberDataHandle->dt.DT, pData + index, PL_DT_DATA);

            saberDataHandle->dt.dataID = PID;
            saberDataHandle->dt.dataLen = pl;
            index += pl;
        }

        else if (PID == SESSION_NAME_OS_TIME)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->tick.OS_Time_ms, pData+index, PL_OS_REFERENCE_TIME-2); //first 4 bytes are miliseconds
            saberDataHandle->tick.OS_Time_ms = *((u32*)(pData + index));
            saberDataHandle->tick.OS_Time_us = *((u16*)(pData + index + 4));

            saberDataHandle->tick.dataID = PID;
            saberDataHandle->tick.dataLen = pl;
            index += pl;
        }
        else if (PID == SESSION_NAME_STATUS_WORD)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->status.status, pData + index, PL_STATUS);
            saberDataHandle->status.dataID = PID;
            saberDataHandle->status.dataLen = pl;
            index += pl;
        }
        else if (PID == SESSION_NAME_PACKET_COUNTER)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->packetCounter.packerCounter, pData + index, PL_PACKET_NUMBER);
            saberDataHandle->packetCounter.dataID = PID;
            saberDataHandle->packetCounter.dataLen = pl;
            index += pl;

            if(fpLog!=NULL)
                fprintf(fpLog," *** packet_count:  %d, *** \n", saberDataHandle->packetCounter.packerCounter);
        }
		else if (PID == SESSION_NAME_ALTITUDE)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->altitude.longlatZ, pData + index, PL_ALTITUDE_NUMBER);
            saberDataHandle->altitude.dataID = PID;
            saberDataHandle->altitude.dataLen = pl;
            index += pl;

            if(fpLog!=NULL)
                fprintf(fpLog," *** altitude:     \t%11.4f  *** \n", saberDataHandle->altitude.longlatZ);
        }
		else if (PID == SESSION_NAME_LONLAT)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->position.longlatX, pData + index, PL_LONLAT_NUMBER);
            saberDataHandle->position.dataID = PID;
            saberDataHandle->position.dataLen = pl;
            index += pl;

            if(fpLog!=NULL)
                fprintf(fpLog," *** LonLat:     \t%11.4f, %11.4f *** \n", saberDataHandle->position.longlatX,saberDataHandle->position.longlatY);
        }
		else if (PID == SESSION_NAME_GNSS)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->GnssData.GnssNavPvtData, pData + index, PL_GNSS);
            saberDataHandle->GnssData.dataID = PID;
            saberDataHandle->GnssData.dataLen = pl;
			saberDataHandle->GnssData.GnssNavPvtData.fixType = (u8)pData[index+20];
            index += pl;

            if(fpLog!=NULL);
                fprintf(fpLog," *** fixType:     \t%d    *** \n", saberDataHandle->GnssData.GnssNavPvtData.fixType);
        }
		else if (PID == SESSION_NAME_BARO)
        {
            //Ignore pid and pl
            index += 3;

            memcpy(&saberDataHandle->BaroData.baro, pData + index, PL_BARO);
            saberDataHandle->BaroData.dataID = PID;
            saberDataHandle->BaroData.dataLen = pl;
            index += pl;

            if(fpLog!=NULL)
                fprintf(fpLog," *** Baro:     \t%11.4f  *** \n", saberDataHandle->BaroData.baro);
        }

        
		else
		{
            printf(" unknow ID... 0x%04x\n",PID );
            //std::cout << "SaberParserDataPacket"<<std::endl;
            //while(1);
			index += 3;
			index += pl;
		}
        
    }
}
