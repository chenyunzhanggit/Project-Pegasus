/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Atom-Robotics Corporation. All rights reserved.
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.24
 *  Version:     1.0
 *  Description: Config Saber  using a JSON file on ROS/Linux
 *--------------------------------------------------------------------------------------------*/
#include "../saber_ros_inc/saber_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>

#include "../main.h"
#include "../config_tool/cJSON.h"
#include "../saber_ros_inc/saber_macro.h"
#include "../saber_ros_inc/saber_serial.h"
#include "../saber_ros_inc/saber_protocol.h"
#include "../saber_ros_inc/saber_tool.h"

extern u8 GnssMod;

/*---------------------------------------------------------------------------------------------
 *  Function:    SaberInitConfig
 *  Author:      niuyunzhu
 *  Last Modify: 2019.10.20
 *  Description: Get config from a JSON file and download the config to Saber
 *  Parameter:   fileName = JSON config file path
 *  Return:      8bit unsigned char
 *               nFD = serialport file desriptor, used by system call ,such as read,write
 *--------------------------------------------------------------------------------------------*/

signed char  SaberInitConfig(const char *fileName){

    int status = 0;
    const cJSON *name = NULL;
    const char * pJSONData = cJSON_ReadFileToMem(fileName); //"/home/atom/catkin_ws/devel/lib/imu_publish/saber_cfg.json")
    cJSON *monitor_json = cJSON_Parse(pJSONData);
    int cfgIndex = 0;
    int baudRate = 0;
    int dataPktLen = 0;
    unsigned char nFD = 0;
    u16 pid = 0;

    ConfigSingleDataPacket cfgPacket[DATAPACKET_CNT];

    if (monitor_json == NULL)
    {
        printf("Warning:Fail to obtain saber_cfg.json file,please check your path in main.cpp\n");
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
#ifndef MCU_ON
            printf("Error before: %s\n", error_ptr);
#endif
        }
        status = -1;
        return status;
    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "BAUDRATE");
    if (cJSON_IsNumber(name)) {
        baudRate = name->valueint;
    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "COM_NAME");
    if (cJSON_IsString(name))
    {
#ifndef MCU_ON
        printf("Connet Serial Port: %s,with baudrate:%d \n", name->valuestring, baudRate);
#endif
        nFD = Saber_OpenCom(name->valuestring,baudRate);
        if(nFD<0){
            return -1;
        }
    }

    Saber_SwitchModeReq(nFD, CONFIG_MODE);


    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "LINEAR_ACC");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Linear acceleration data enable\n");
#endif
        pid = SESSION_NAME_LINEAR_ACC | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= LINEARACC_LEN;
    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ANGULAR_VELOCITY");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Angular velocity data enable \n");
#endif
        pid = SESSION_NAME_CAL_GYRO | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= GYRO_CAL_LEN;
    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ORIENTATION");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Quaternion data enable \n");
#endif
        pid = SESSION_NAME_QUAT | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= ORIENTION_LEN;
    }
	
	name = cJSON_GetObjectItemCaseSensitive(monitor_json, "EULER");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("EULER data enable \n");
#endif
        pid = SESSION_NAME_EULER | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= ORIENTION_LEN;
    }

    //Add other data packet ,such as raw data, temperature, packet cnt, eulur data...
    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ACC_RAW");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Acceleration RAW data enable \n");
#endif
        pid = SESSION_NAME_RAW_ACC | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= ACC_RAW_LEN;

    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ACC_CAL");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Acceleration CAL data enable \n");
#endif
        pid = SESSION_NAME_CAL_ACC | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= ACC_CAL_LEN;

    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ACC_KAL");
    if (cJSON_IsTrue(name))
    {
#ifndef MCU_ON
        printf("Acceleration KAL data enable \n");
#endif
        pid = SESSION_NAME_KAL_ACC | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= ACC_KAL_LEN;

    }

    name = cJSON_GetObjectItemCaseSensitive(monitor_json, "GYRO_RAW");
    if (cJSON_IsTrue(name))
    {
        printf("Gyroscope rawdata enable \n");
        pid = SESSION_NAME_RAW_GYRO | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
        dataPktLen+= GYRO_RAW_LEN;
    }

	name = cJSON_GetObjectItemCaseSensitive(monitor_json, "ALTITUDE");
    if (cJSON_IsTrue(name))
    {
        printf("ALTITUDE data  enable \n");
        pid = SESSION_NAME_ALTITUDE | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
    }
	
	name = cJSON_GetObjectItemCaseSensitive(monitor_json, "LONLAT");
    if (cJSON_IsTrue(name))
    {
        printf("LONLAT data  enable \n");
        pid = SESSION_NAME_LONLAT | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
    }
	
	name = cJSON_GetObjectItemCaseSensitive(monitor_json, "GNSS_PVT");
    if (cJSON_IsTrue(name))
    {
        printf("GNSS_PVT data enable \n");
        pid = SESSION_NAME_GNSS | 0x8000;
        cfgPacket[cfgIndex].reserve0 = 0xff;
        cfgPacket[cfgIndex].reserve1 = 0xff;
        cfgPacket[cfgIndex].packetID = pid;
        cfgIndex++;
    }
#ifndef MCU_ON
    printf("Packet total length is  %d \n", dataPktLen);
#endif

    Saber_SetDataPacketConfigReq(nFD, (u8 *) &cfgPacket, cfgIndex * 4);
#ifdef GPS_ON
	Saber_GetGNSSReceiverParaReq(nFD,(u8 *) &cfgPacket,0);
	printf("GET GNSS MOD \n");
	GnssMod = SaberGetGnssmod(nFD);
#endif
    Saber_SwitchModeReq(nFD, MEASURE_MODE);
    /*TODO:free pJSONData*/
    if(pJSONData != NULL){
        free((void *)pJSONData);
    }
    return nFD;
}

