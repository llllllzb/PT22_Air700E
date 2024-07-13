#include <app_protocol.h>
#include "app_task.h"
#include "app_mir3da.h"
#include "app_atcmd.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_net.h"
#include "app_net.h"
#include "app_param.h"
#include "app_port.h"
#include "app_sys.h"
#include "app_socket.h"
#include "app_server.h"
#include "app_jt808.h"
#include "app_central.h"
#include "app_hid.h"
#include "math.h"

#define SYS_LED1_ON       LED1_ON
#define SYS_LED1_OFF      LED1_OFF


static SystemLEDInfo sysledinfo;
motionInfo_s motionInfo;
static bleScanTry_s bleTry;
static int8_t wifiTimeOutId = -1;

/**************************************************
@bref		bit0 ��λ������
@param
@return
@note
**************************************************/
void terminalDefense(void)
{
    sysinfo.terminalStatus |= 0x01;
}

/**************************************************
@bref		bit0 ���������
@param
@return
@note
**************************************************/
void terminalDisarm(void)
{
    sysinfo.terminalStatus &= ~0x01;
}
/**************************************************
@bref		��ȡ�˶���ֹ״̬
@param
@return
	>0		�˶�
	0		��ֹ
@note
**************************************************/

uint8_t getTerminalAccState(void)
{
    return (sysinfo.terminalStatus & 0x02);

}

/**************************************************
@bref		bit1 ��λ���˶���accon
@param
@return
@note
**************************************************/

void terminalAccon(void)
{
    sysinfo.terminalStatus |= 0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 1);
}

/**************************************************
@bref		bit1 �������ֹ��accoff
@param
@return
@note
**************************************************/
void terminalAccoff(void)
{
    sysinfo.terminalStatus &= ~0x02;
    jt808UpdateStatus(JT808_STATUS_ACC, 0);
}

/**************************************************
@bref		bit2 ��λ�����
@param
@return
@note
**************************************************/

void terminalCharge(void)
{
    sysinfo.terminalStatus |= 0x04;
}
/**************************************************
@bref		bit2 �����δ���
@param
@return
@note
**************************************************/

void terminalunCharge(void)
{
    sysinfo.terminalStatus &= ~0x04;
}

/**************************************************
@bref		��ȡ���״̬
@param
@return
	>0		���
	0		δ���
@note
**************************************************/

uint8_t getTerminalChargeState(void)
{
    return (sysinfo.terminalStatus & 0x04);
}

/**************************************************
@bref		bit 3~5 ������Ϣ
@param
@return
@note
**************************************************/

void terminalAlarmSet(TERMINAL_WARNNING_TYPE alarm)
{
    sysinfo.terminalStatus &= ~(0x38);
    sysinfo.terminalStatus |= (alarm << 3);
}

/**************************************************
@bref		bit6 ��λ���Ѷ�λ
@param
@return
@note
**************************************************/

void terminalGPSFixed(void)
{
    sysinfo.terminalStatus |= 0x40;
}

/**************************************************
@bref		bit6 �����δ��λ
@param
@return
@note
**************************************************/

void terminalGPSUnFixed(void)
{
    sysinfo.terminalStatus &= ~0x40;
}

/**************************************************
@bref		LED1 ��������
@param
@return
@note
**************************************************/

static void sysLed1Run(void)
{
    static uint8_t tick = 0;


    if (sysledinfo.sys_led1_on_time == 0)
    {
        SYS_LED1_OFF;
        return;
    }
    else if (sysledinfo.sys_led1_off_time == 0)
    {
        SYS_LED1_ON;
        return;
    }

    tick++;
    if (sysledinfo.sys_led1_onoff == 1) //on status
    {
        SYS_LED1_ON;
        if (tick >= sysledinfo.sys_led1_on_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 0;
        }
    }
    else   //off status
    {
        SYS_LED1_OFF;
        if (tick >= sysledinfo.sys_led1_off_time)
        {
            tick = 0;
            sysledinfo.sys_led1_onoff = 1;
        }
    }
}

/**************************************************
@bref		���õƵ���˸Ƶ��
@param
@return
@note
**************************************************/

static void ledSetPeriod(uint8_t ledtype, uint8_t on_time, uint8_t off_time)
{
    if (ledtype == GPSLED1)
    {
        //ϵͳ�źŵ�
        sysledinfo.sys_led1_on_time = on_time;
        sysledinfo.sys_led1_off_time = off_time;
    }
}

/**************************************************
@bref		����ϵͳ��״̬
@param
@return
@note
**************************************************/

void ledStatusUpdate(uint8_t status, uint8_t onoff)
{
    if (onoff == 1)
    {
        sysinfo.sysLedState |= status;
    }
    else
    {
        sysinfo.sysLedState &= ~status;
    }
    if ((sysinfo.sysLedState & SYSTEM_LED_RUN) == SYSTEM_LED_RUN)
    {
        //����
        ledSetPeriod(GPSLED1, 10, 10);
        if ((sysinfo.sysLedState & SYSTEM_LED_NETOK) == SYSTEM_LED_NETOK)
        {
            //����
            ledSetPeriod(GPSLED1, 1, 9);
            if ((sysinfo.sysLedState & SYSTEM_LED_GPSOK) == SYSTEM_LED_GPSOK)
            {
                //��ͨ�Ƴ���
                ledSetPeriod(GPSLED1, 1, 0);
            }
        }
    }
    else
    {
        SYS_LED1_OFF;
        ledSetPeriod(GPSLED1, 0, 1);
    }
}

/**************************************************
@bref		�ƿ�����
@param
@return
@note
**************************************************/

static void ledTask(void)
{
	if (sysinfo.ledTick <= 0)
	{
		SYS_LED1_OFF;
		return;
	}


    sysLed1Run();
}

/**************************************************
@bref		gps��������
@param
@return
@note
**************************************************/
void gpsRequestSet(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestSet==>0x%04X", flag);
    sysinfo.gpsRequest |= flag;
}

/**************************************************
@bref		gps�������
@param
@return
@note
**************************************************/

void gpsRequestClear(uint32_t flag)
{
    LogPrintf(DEBUG_ALL, "gpsRequestClear==>0x%04X", flag);
    sysinfo.gpsRequest &= ~flag;
}

uint32_t gpsRequestGet(uint32_t flag)
{
    return sysinfo.gpsRequest & flag;
}

/**************************************************
@bref		gps����״̬���л�
@param
@return
@note
**************************************************/

static void gpsChangeFsmState(uint8_t state)
{
    sysinfo.gpsFsm = state;
}


/**************************************************
@bref		gps���ݽ���
@param
@return
@note
**************************************************/

void gpsUartRead(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[UART_RECV_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > UART_RECV_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            if (/*sysinfo.nmeaOutPutCtl*/1)
            {
                LogWL(DEBUG_GPS, gpsRestore + begin, i - begin);
                LogWL(DEBUG_GPS, "\r\n", 2);
            }
            nmeaParser(gpsRestore + begin, i - begin);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}

//static void gpsCfg(void)
//{
//    char param[50];
//    //�ر�GSV
//    //sprintf(param, "$CCMSG,GSV,1,0,*1A\r\n");
//    sprintf(param, "$PCAS03,1,0,1,0,1,0,0,0,0,0,0,0,0,0*03\r\n");
//    portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
//    sprintf(param, "$PCAS03,,,,,,,,,,,1*1F\r\n");
//    portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
//    LogMessage(DEBUG_ALL, "gps config nmea output");
//}
///**************************************************
//@bref       �л��п�΢������Ϊ115200
//@param
//@return
//@note
//**************************************************/
//
////$PCAS03,1,0,1,1,1,0,0,0,0,0,0,0,0,0*02
//static void changeGPSBaudRate(void)
//{
//    char param[50];
//    sprintf(param, "$PCAS01,5*19\r\n");
//    portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
//    portUartCfg(APPUSART1, 1, 115200, gpsUartRead);
//    LogMessage(DEBUG_ALL, "gps config baudrate to 115200");
//    startTimer(10, gpsCfg, 0);
//}
///**************************************************
//@bref       �п�΢����������
//@param
//@return
//@note
//**************************************************/
//
//static void gpsWarmStart(void)
//{
//    char param[50];
//    //������
//    sprintf(param, "$PCAS10,0*1C\r\n");
//    portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
//    LogMessage(DEBUG_ALL, "Gps config warm start");
//    startTimer(10, changeGPSBaudRate, 0);
//}

/*
		о����gps
	
Ĭ�ϲ���: 115200,  uart0(gpsоƬ)

����ָ��:
����GSV:$CFGMSG,0,3,1,1*18\r\n
�ر�GSV:$CFGMSG,0,3,0,1*19\r\n

����GLL:$CFGMSG,0,1,1,1*1A\r\n
�ر�GLL:$CFGMSG,0,1,0,1*1B\r\n

����ZDA:$CFGMSG,0,6,1,1*1D\r\n
�ر�ZDA:$CFGMSG,0,6,0,1*1C\r\n

��λ:$RESET,0,h01*0C

������λ:
��ѯagnss����������Чֵ:$AIDINFO
���븨����λʱ��:$AIDTIME
���븨����λλ��:$AIDPOS

*/

/**************************************************
@bref		о����gpsоƬ��ѯ���������Ƿ���Ч
@param
@return
@note
**************************************************/
void icoeGpsAgnssInfo(void)
{
	uint8_t param[20] = { 0 };
	sprintf(param, "$AIDINFO\r\n");
	portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
	LogMessage(DEBUG_ALL, "icoeGpsAgnssInfo==>ok");
}

/**************************************************
@bref		о����gpsоƬ���븨��ʱ��
@param
@return
@note
**************************************************/
void icoeGpsInjectAidTime(uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second)
{
	uint8_t param[100] = { 0 };
	uint8_t crc;
	sprintf(param, "$AIDTIME,%d,%d,%d,%d,%d,%d,200", year, month, date, hour, minute, second);
	crc = param[1];
	XOR_CRC_TOTAL(param + 2, strlen(param + 2), crc);
	sprintf(param + strlen(param), "*%x\r\n", crc);
	portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
	LogPrintf(DEBUG_ALL, "icoeGpsInjectAidTime==>%s", param);
}

/**************************************************
@bref		о����gpsоƬ���븨��λ��
@param
@return
@note
**************************************************/
void icoeGpsInjectLatLon(gpsinfo_s *gpsinfo)
{
	uint8_t param[100] = { 0 };
	uint8_t crc;
	sprintf(param, "$AIDPOS,%lf,%c,%lf,%c,%f",
	        fabsf(dynamicParam.saveLat), dynamicParam.saveLat > 0.0 ? 'N' : 'S', fabsf(dynamicParam.saveLon), dynamicParam.saveLon > 0.0 ? 'E' : 'W', dynamicParam.savealt);
	crc = param[1];
	XOR_CRC_TOTAL(param + 2, strlen(param + 2), crc);
	sprintf(param + strlen(param), "*%x\r\n", crc);
	portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
	LogPrintf(DEBUG_ALL, "icoeGpsInjectLatLon==>%s", param);

}


/**************************************************
@bref		о����gpsоƬ��λ
@param
@return
@note
**************************************************/
void icoeGpsReset(void)
{
	uint8_t param[50] = { 0 };
	sprintf(param, "$RESET,0,h01*0C\r\n");
	portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
	LogMessage(DEBUG_ALL, "icoeGpsReset==>ok");
}

/**************************************************
@bref       о����gps�������
@param
@return
@note
**************************************************/
static void icoeGpsCfg(void)
{
	uint8_t param[128] = { 0 };
	//�ر�GSV �ر�GLL �ر�ZDA
	sprintf(param, "$CFGMSG,0,1,0,1*1B\r\n$CFGMSG,0,6,1,1*1D\r\n");
	portUartSend(&usart1_ctl, (uint8_t *)param, strlen(param));
	LogMessage(DEBUG_ALL, "icoeGpsCfg==>ok");
}


/**************************************************
@bref       ����gps
@param
@return
@note
**************************************************/

void gpsOpen(void)
{
    GPSPWR_ON;
    GPSLNA_ON;
    portUartCfg(APPUSART1, 1, 115200, gpsUartRead);
    startTimer(20, icoeGpsCfg, 0);
    sysinfo.gpsUpdatetick = sysinfo.sysTick;
    sysinfo.gpsOnoff = 1;
    gpsChangeFsmState(GPSWATISTATUS);
    gpsClearCurrentGPSInfo();
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
    moduleSleepCtl(0);
    LogMessage(DEBUG_ALL, "gpsOpen");
    sysinfo.ephemerisFlag = 0;
}
/**************************************************
@bref       �ȴ�gps�ȶ�
@param
@return
@note
**************************************************/

static void gpsWait(void)
{
//    static uint8_t runTick = 0;
//    if (++runTick >= 4)
//    {
//        runTick = 0;
//        gpsChangeFsmState(GPSOPENSTATUS);
//        if (sysinfo.ephemerisFlag == 0 || sysinfo.agpsTick == 0 || sysinfo.sysTick >= sysinfo.agpsTick)
//        {
//          	agpsRequestSet();
//          	sysinfo.agpsTick = sysinfo.sysTick + 120;
//        }
//    }
	agpsRequestSet();
	gpsChangeFsmState(GPSOPENSTATUS);
}

/**************************************************
@bref       �ر�gps
@param
@return
@note
**************************************************/

void gpsClose(void)
{
    GPSPWR_OFF;
    GPSLNA_OFF;
    portUartCfg(APPUSART1, 0, 115200, NULL);
    sysinfo.rtcUpdate = 0;
    sysinfo.gpsOnoff = 0;
    gpsClearCurrentGPSInfo();
    terminalGPSUnFixed();
    gpsChangeFsmState(GPSCLOSESTATUS);
    ledStatusUpdate(SYSTEM_LED_GPSOK, 0);
    if (primaryServerIsReady())
    {
        moduleSleepCtl(1);
    }
    LogMessage(DEBUG_ALL, "gpsClose");
}

/**************************************************
@bref		������һ��gpsλ��
@param
@return
@note
**************************************************/
void saveGpsHistory(void)
{
	gpsinfo_s *gpsinfo;
    double latitude, longtitude;
    gpsinfo = getLastFixedGPSInfo();
    if (gpsinfo->fixstatus != 0)
    {
        latitude   = gpsinfo->src_lat;
        longtitude = gpsinfo->src_lon;
        if (gpsinfo->NS == 'S')
        {
            if (latitude > 0)
            {
                latitude *= -1;
            }
        }
        if (gpsinfo->EW == 'W')
        {
            if (longtitude > 0)
            {
                longtitude *= -1;
            }
        }
        dynamicParam.saveLat = latitude;
        dynamicParam.saveLon = longtitude;
        dynamicParam.savealt = gpsinfo->src_alt;
        LogPrintf(DEBUG_ALL, "Save Latitude:%lf,Longtitude:%lf altitude:%f", 
        		dynamicParam.saveLat, dynamicParam.saveLon, dynamicParam.savealt);
		dynamicParamSaveAll();
    }
}


/**************************************************
@bref		gps��������
@param
@return
@note
**************************************************/

static void gpsRequestTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t gpsInvalidTick = 0;
	static uint8_t gpsInvalidFlag = 0, gpsInvalidFlagTick = 0;
	uint16_t gpsInvalidparam;
	/* ͬ��flag */
	static uint8_t flag = 0;

    switch (sysinfo.gpsFsm)
    {
        case GPSCLOSESTATUS:
            //���豸���󿪹�
            if (sysinfo.canRunFlag != 1)
            {
            	break;
            }
            if (sysinfo.gpsRequest != 0)
            {
                gpsOpen();
            }
            break;
        case GPSWATISTATUS:
            gpsWait();
            break;
        case GPSOPENSTATUS:
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus)
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 1);
               	lbsRequestClear();
				wifiRequestClear();
				//GPS���еĹ����вŸı����ͬ����flag�������λ����һ�� ��gps֮ǰ�ֲ���λ�Ļ�����ô�ֻ����¿�ʼ��ʱ�ϱ�wifi
				if (sysinfo.gpsRequest != 0)
					flag = 0;
            }
            else
            {
                ledStatusUpdate(SYSTEM_LED_GPSOK, 0);    
                if (sysinfo.gpsRequest != 0)
                	flag = 1;
            }
            if (sysinfo.gpsRequest == 0 || (sysinfo.sysTick - sysinfo.gpsUpdatetick) >= 10)
            {
            	if (sysinfo.gpsRequest == 0)
            	{
					saveGpsHistory();
					agpsRequestClear();
            	}
                gpsClose();
            }
            break;
        default:
            gpsChangeFsmState(GPSCLOSESTATUS);
            break;
    }

//    /* ֻҪACCON����Ϊ�ǿ���GPS�� ����Ҫ����GPS�Ŀ������ */
//    if (getTerminalAccState() == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//        return;
//    }
//    if (sysparam.gpsuploadgap == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//        return;
//    }
//    if (flag == 0)
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//		return;
//    }
//    gpsInvalidparam = (sysparam.gpsuploadgap < 60) ? 60 : sysparam.gpsuploadgap;
//    LogPrintf(DEBUG_ALL, "gpsInvalidTick:%d  gpsInvalidparam:%d", gpsInvalidTick, gpsInvalidparam);
//    gpsinfo = getCurrentGPSInfo();
//    if (gpsinfo->fixstatus == 0)
//    {
//        if (++gpsInvalidTick >= gpsInvalidparam)
//        {
//            gpsInvalidTick = 0;
//            gpsInvalidFlag = 1;
//    		wifiRequestSet(DEV_EXTEND_OF_MY);
//        }
//    }
//    else
//    {
//        gpsInvalidTick = 0;
//        gpsInvalidFlag = 0;
//        gpsInvalidFlagTick = 0;
//    }

}



/**************************************************
@bref		����һ��gpsλ��
@param
@return
@note
**************************************************/

static void gpsUplodOnePointTask(void)
{
    gpsinfo_s *gpsinfo;
    static uint16_t runtick = 0;
    static uint8_t uploadtick = 0;
    //�ж��Ƿ���������¼�
    if (sysinfo.gpsOnoff == 0)
        return;
    if (gpsRequestGet(GPS_REQUEST_UPLOAD_ONE) == 0)
    {
        runtick = 0;
        uploadtick = 0;
        return;
    }
    gpsinfo = getCurrentGPSInfo();
    runtick++;
    if (gpsinfo->fixstatus == 0)
    {
        uploadtick = 0;
        if (runtick >= 180)
        {
            runtick = 0;
            uploadtick = 0;
            LogPrintf(DEBUG_ALL, "gps fix time out");
            gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
            if (getTerminalAccState() == 0)
            {
            	wifiRequestSet(DEV_EXTEND_OF_MY);
            }
        }
        return;
    }
    runtick = 0;
    if (++uploadtick >= 10)
    {
        uploadtick = 0;
        if (sysinfo.flag123)
        {
            dorequestSend123();
        }
        protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        if (!(sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX && sysparam.gpsuploadgap != 0 && 
        	(sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)))
        {
			if (primaryServerIsReady())
    		{
        		protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                            dynamicParam.startUpCnt, dynamicParam.runTime);
        		protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
        		privateHbtTickReset();
        	}
        }
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        gpsRequestClear(GPS_REQUEST_UPLOAD_ONE);
    }

}

/**************************************************
@bref		������������
@param
@return
@note
**************************************************/
void alarmRequestSet(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestSet==>0x%04X", request);
    sysinfo.alarmRequest |= request;
}
/**************************************************
@bref		�����������
@param
@return
@note
**************************************************/

void alarmRequestClear(uint16_t request)
{
    LogPrintf(DEBUG_ALL, "alarmRequestClear==>0x%04X", request);
    sysinfo.alarmRequest &= ~request;
}

/**************************************************
@bref		��������
@param
@return
@note
**************************************************/

void alarmRequestTask(void)
{
    uint8_t alarm;
    if (primaryServerIsReady() == 0 || sysinfo.alarmRequest == 0)
    {
        return;
    }
    if (getTcpNack() != 0)
    {
        return;
    }
    //�йⱨ��
    if (sysinfo.alarmRequest & ALARM_LIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_LIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Light Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LIGHT);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�͵籨��
    if (sysinfo.alarmRequest & ALARM_LOWV_REQUEST)
    {
        alarmRequestClear(ALARM_LOWV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>LowVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOWV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�ϵ籨��
    if (sysinfo.alarmRequest & ALARM_LOSTV_REQUEST)
    {
        alarmRequestClear(ALARM_LOSTV_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>lostVoltage Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_LOSTV);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    
    //�𶯱���
    if (sysinfo.alarmRequest & ALARM_SHUTTLE_REQUEST)
    {
        alarmRequestClear(ALARM_SHUTTLE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>shuttle Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SHUTTLE);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //SOS����
    if (sysinfo.alarmRequest & ALARM_SOS_REQUEST)
    {
        alarmRequestClear(ALARM_SOS_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>SOS Alarm");
        terminalAlarmSet(TERMINAL_WARNNING_SOS);
        alarm = 0;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //�����ٱ���
    if (sysinfo.alarmRequest & ALARM_ACCLERATE_REQUEST)
    {
        alarmRequestClear(ALARM_ACCLERATE_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid Accleration Alarm");
        alarm = 9;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //�����ٱ���
    if (sysinfo.alarmRequest & ALARM_DECELERATE_REQUEST)
    {
        alarmRequestClear(ALARM_DECELERATE_REQUEST);
        LogMessage(DEBUG_ALL,
                   "alarmRequestTask==>Rapid Deceleration Alarm");
        alarm = 10;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //����ת����
    if (sysinfo.alarmRequest & ALARM_RAPIDLEFT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDLEFT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid LEFT Alarm");
        alarm = 11;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //����ת����
    if (sysinfo.alarmRequest & ALARM_RAPIDRIGHT_REQUEST)
    {
        alarmRequestClear(ALARM_RAPIDRIGHT_REQUEST);
        LogMessage(DEBUG_ALL, "alarmRequestTask==>Rapid RIGHT Alarm");
        alarm = 12;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

    //��������
    if (sysinfo.alarmRequest & ALARM_GUARD_REQUEST)
    {
        alarmRequestClear(ALARM_GUARD_REQUEST);
        LogMessage(DEBUG_ALL, "alarmUploadRequest==>Guard Alarm\n");
        alarm = 1;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }
    //�ػ�����
    if (sysinfo.alarmRequest & ALARM_SHUTDOWN_REQUEST)
    {
        alarmRequestClear(ALARM_SHUTDOWN_REQUEST);
        LogMessage(DEBUG_ALL, "alarmUploadRequest==>�ػ�����");
        alarm = 0x30;
        protocolSend(NORMAL_LINK, PROTOCOL_16, &alarm);
    }

}



/**************************************************
@bref		�����˶���ֹ״̬
@param
	src 		�����Դ
	newState	��״̬
@note
**************************************************/

static void motionStateUpdate(motion_src_e src, motionState_e newState)
{
    char type[20];


    if (motionInfo.motionState == newState)
    {
        return;
    }
    motionInfo.motionState = newState;
    switch (src)
    {
        case ACC_SRC:
            strcpy(type, "acc");
            break;
        case VOLTAGE_SRC:
            strcpy(type, "voltage");
            break;
        case GSENSOR_SRC:
            strcpy(type, "gsensor");
            break;
       	case SYS_SRC:
       		strcpy(type, "sys");
            break;
        case STEP_SRC:
			strcpy(type, "step");
        	break;
        default:
            strcpy(type, "unknow");
            break;
    }
    LogPrintf(DEBUG_ALL, "Device %s , detected by %s", newState == MOTION_MOVING ? "moving" : "static", type);

    if (newState)
    {
        netResetCsqSearch();
        if (src != SYS_SRC)
        {
	        if (sysparam.gpsuploadgap != 0)
	        {
	            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	            if (sysparam.gpsuploadgap < GPS_UPLOAD_GAP_MAX)
	            {
	                gpsRequestSet(GPS_REQUEST_ACC_CTL);
	            }
	        }
	        //û����ʱ
	        if (isModeDone())
	        {
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	        }    
	        if (sysparam.bf)
	        {
				alarmRequestSet(ALARM_GUARD_REQUEST);
	        }
        }
        terminalAccon();
        hiddenServerCloseClear();
    }
    else
    {
    	if (src != SYS_SRC)
    	{
	        if (sysparam.gpsuploadgap != 0)
	        {
	            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
	            gpsRequestClear(GPS_REQUEST_ACC_CTL);
	        }
        }
        terminalAccoff();
        sysinfo.accStep = sysinfo.step;		//ͬ��acc����
        updateRTCtimeRequest();
    }
    if (primaryServerIsReady() && src != SYS_SRC)
    {
        protocolInfoResiter(getBatteryLevel(), sysinfo.outsidevoltage > 5.0 ? sysinfo.outsidevoltage : sysinfo.insidevoltage,
                            dynamicParam.startUpCnt, dynamicParam.runTime);
        protocolSend(NORMAL_LINK, PROTOCOL_13, NULL);
        jt808SendToServer(TERMINAL_POSITION, getLastFixedGPSInfo());
        privateHbtTickReset();
    }
}


/**************************************************
@bref       ���ж�
@param
@note
**************************************************/

void motionOccur(void)
{
    motionInfo.tapInterrupt++;
}

/**************************************************
@bref       tapCnt ��С
@param
@note
**************************************************/

uint8_t motionGetSize(void)
{
    return sizeof(motionInfo.tapCnt);
}
/**************************************************
@bref		ͳ��ÿһ����жϴ���
@param
@note
**************************************************/

static void motionCalculate(void)
{
    motionInfo.ind = (motionInfo.ind + 1) % sizeof(motionInfo.tapCnt);
    motionInfo.tapCnt[motionInfo.ind] = motionInfo.tapInterrupt;
    motionInfo.tapInterrupt = 0;
}
/**************************************************
@bref		��ȡ�����n����𶯴���
@param
@note
**************************************************/

static uint16_t motionGetTotalCnt(uint8_t n)
{
    uint16_t cnt;
    uint8_t i;
    cnt = 0;
    for (i = 0; i < n; i++)
    {
        cnt += motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)];
    }
    return cnt;
}

/**************************************************
@bref       ��ⵥλʱ������Ƶ�ʣ��ж��Ƿ��˶�
@param
@note
**************************************************/

static uint16_t motionCheckOut(uint8_t sec)
{
    uint8_t i;
    uint16_t validCnt;

    validCnt = 0;
    if (sec == 0 || sec > sizeof(motionInfo.tapCnt))
    {
        return 0;
    }
    for (i = 0; i < sec; i++)
    {
        if (motionInfo.tapCnt[(motionInfo.ind + sizeof(motionInfo.tapCnt) - i) % sizeof(motionInfo.tapCnt)] != 0)
        {
            validCnt++;
        }
    }
    return validCnt;
}

void motionClear(void)
{
	LogMessage(DEBUG_ALL, "motionClear==>OK");
	memset(&motionInfo, 0, sizeof(motionInfo_s));
}

/**************************************************
@bref		��ȡ�˶�״̬
@param
@note
**************************************************/

motionState_e motionGetStatus(void)
{
    return motionInfo.motionState;
}
#define ACC_ON_DETECT_TIME			6

static uint8_t accOnStep[ACC_ON_DETECT_TIME]   = { 0 };
static uint8_t acc_on_step_ind  = 0;
static uint32_t totalStep = 0;

static void accStepInit(void)
{
	totalStep = 0;
	tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
	acc_on_step_ind  = 0;
}

uint8_t accOnOffByGsensorStep(void)
{
	uint32_t stepgap = 0;
	uint8_t i = 0;
	static uint8_t runtick = 0;
//	if (getTerminalAccState())
//	{
//		acc_on_step_ind = 0;
//		runtick = 0;
//		tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
//		//�����˶�ʱ���жϲ���
//		return 0;
//	}
	runtick++;
	if (runtick <= 10)
	{
		if (totalStep >= sysparam.motionstep)
			return 1;
		else
			return 0;
	}
	runtick = 0;
	totalStep = 0;
	portUpdateStep();
	stepgap = abs(sysinfo.step - sysinfo.accStep);
	accOnStep[acc_on_step_ind] = stepgap;
	acc_on_step_ind = (acc_on_step_ind + 1) % ACC_ON_DETECT_TIME;
	for (i = 0; i < ACC_ON_DETECT_TIME; i++)
	{
		totalStep += accOnStep[i];
	}
	sysinfo.accStep = sysinfo.step;
	LogPrintf(DEBUG_ALL, "accOnOffByGsensorStep==>%d step", totalStep);
	if (totalStep >= sysparam.motionstep)
	{
		//tmos_memset(accOnStep, 0, ACC_ON_DETECT_TIME);
		return 1;
	}
	return 0;
}

/**************************************************
@bref		�˶��;�ֹ���ж�
@param
@note
**************************************************/

static void motionCheckTask(void)
{
    static uint16_t gsStaticTick = 0;
    static uint16_t autoTick = 0;
    static uint8_t  accOnTick = 0;
    static uint8_t  accOffTick = 0;
    static uint8_t fixTick = 0;

    static uint8_t  volOnTick = 0;
    static uint8_t  volOffTick = 0;
    static uint8_t bfFlag = 0;
    static uint8_t bfTick = 0;
    static uint8_t lTick = 0, hTick = 0;
    static uint8_t vFlag = 0;
    static uint8_t motionState = 0;
    gpsinfo_s *gpsinfo;

    uint16_t totalCnt, staticTime;

    motionCalculate();

	if (sysinfo.gsensorOnoff == 0)
	{
		motionState = 0;
		return;
	}
    if (sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
    {
        staticTime = 180;
    }
    else
    {
        staticTime = 180;
    }



    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 || sysparam.MODE == MODE4)
    {
        motionStateUpdate(SYS_SRC, MOTION_STATIC);
        gsStaticTick = 0;
        return ;
    }

    //�����˶�״̬ʱ�����gap����Max�����������ϱ�gps
    if (getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    {
        if (++autoTick >= sysparam.gpsuploadgap)
        {
            autoTick = 0;
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
        }
    }
    else
    {
        autoTick = 0;
    }
    totalCnt = motionCheckOut(sysparam.gsdettime);
//    LogPrintf(DEBUG_ALL, "motionCheckOut=%d,%d,%d,%d,%d,%d", totalCnt, sysparam.gsdettime, sysparam.gsValidCnt,
//                  sysparam.gsInvalidCnt, motionState, autoTick);

    if (totalCnt >= sysparam.gsValidCnt && sysparam.gsValidCnt != 0)
    {
        motionState = 1;
    }
    else if (totalCnt <= sysparam.gsInvalidCnt)
    {
        motionState = 0;
    }

    if (ACC_READ == ACC_STATE_ON)
    {
        //����Զ�ǵ�һ���ȼ�
        if (++accOnTick >= 10)
        {
            accOnTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_MOVING);
        }
        accOffTick = 0;
        return;
    }
    accOnTick = 0;
    if (sysparam.accdetmode == ACCDETMODE0)
    {
        //����acc�߿���
        if (++accOffTick >= 10)
        {
            accOffTick = 0;
            motionStateUpdate(ACC_SRC, MOTION_STATIC);
        }
        return;
    }

    if (sysparam.accdetmode == ACCDETMODE1 || sysparam.accdetmode == ACCDETMODE3)
    {
        //��acc��+��ѹ����
        if (sysinfo.outsidevoltage >= sysparam.accOnVoltage)
        {
            if (++volOnTick >= 5)
            {
                vFlag = 1;
                volOnTick = 0;
                motionStateUpdate(VOLTAGE_SRC, MOTION_MOVING);
            }
        }
        else
        {
            volOnTick = 0;
        }

        if (sysinfo.outsidevoltage < sysparam.accOffVoltage)
        {
            if (++volOffTick >= 15)
            {
                vFlag = 0;
                volOffTick = 0;
                if (sysparam.accdetmode == ACCDETMODE1)
                {
                    motionStateUpdate(MOTION_MOVING, MOTION_STATIC);
                }
            }
        }
        else
        {
            volOffTick = 0;
        }
        if (sysparam.accdetmode == ACCDETMODE1 || vFlag != 0)
        {
            return;
        }
    }
    //ʣ�µģ���acc��+gsensor����

    if (motionState)
    {
        motionStateUpdate(GSENSOR_SRC, MOTION_MOVING);
    }
    if (motionState == 0)
    {
        if (sysinfo.gpsOnoff)
        {
            gpsinfo = getCurrentGPSInfo();
            if (gpsinfo->fixstatus && gpsinfo->speed >= 7)
            {
                if (++fixTick >= 5)
                {
                    gsStaticTick = 0;
                }
            }
            else
            {
                fixTick = 0;
            }
        }
        gsStaticTick++;
        if (gsStaticTick >= staticTime)
        {
            motionStateUpdate(GSENSOR_SRC, MOTION_STATIC);
        }
    }
    else
    {
        gsStaticTick = 0;
    }
}





/**************************************************
@bref		��ѹ�������
@param
@return
@note
**************************************************/

static void voltageCheckTask(void)
{
    static uint16_t lowpowertick = 0;
    static uint8_t  lowwflag = 0;
    float x;
    static uint8_t protectTick = 0;
    uint8_t ret = 0;
    if (sysinfo.adcOnoff == 0)
	{
		return;
	}
	/* �豸���еĹ����п��ܻḴλģ�飬Ҳ���Ǹ�ģ��ϵ磬���ʱ������ͣ��ѹ��⣬�������е�ѹ */
	if (sysinfo.moduleSupplyStatus == 0)
	{
		LogPrintf(DEBUG_ALL, "Temporarily stop voltage detection..");
		return;
	}
    x = portGetAdcVol(ADC_CHANNEL);
    sysinfo.outsidevoltage = x * sysparam.adccal;
    sysinfo.insidevoltage = sysinfo.outsidevoltage;

    //LogPrintf(DEBUG_ALL, "x:%.2f, outsidevoltage:%.2f bat:%d%%", x, sysinfo.outsidevoltage ,getBatteryLevel());

	//��ر���
    if (sysinfo.outsidevoltage < BAT_DET_CANNOT_RUN_VOL && sysinfo.canRunFlag == 1)
    {
		protectTick++;
		if (protectTick >= 5)
		{
			protectTick = 0;
			sysinfo.canRunFlag = 0;
			LogPrintf(DEBUG_ALL, "Batvoltage is lowwer than %.2f", sysinfo.outsidevoltage);

		}
    }
	else
	{
		protectTick = 0;
	}
    
    //�͵籨��
    if (sysinfo.outsidevoltage < sysinfo.lowvoltage)
    {
        lowpowertick++;
        if (lowpowertick >= 30)
        {
            if (lowwflag == 0)
            {
                lowwflag = 1;
                LogPrintf(DEBUG_ALL, "power supply too low %.2fV", sysinfo.outsidevoltage);
                //�͵籨��
                jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 1);
                alarmRequestSet(ALARM_LOWV_REQUEST);
                //lbsRequestSet(DEV_EXTEND_OF_MY);
                wifiRequestSet(DEV_EXTEND_OF_MY);
                gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            }

        }
    }
    else
    {
        lowpowertick = 0;
    }


    if (sysinfo.outsidevoltage >= sysinfo.lowvoltage + 0.5)
    {
        lowwflag = 0;
        jt808UpdateAlarm(JT808_LOWVOLTAE_ALARM, 0);
    }


}

/**************************************************
@bref		ģʽ״̬���л�
@param
@return
@note
**************************************************/

static void changeModeFsm(uint8_t fsm)
{
    sysinfo.runFsm = fsm;
    LogPrintf(DEBUG_ALL, "changeModeFsm==>%d", fsm);
}

/**************************************************
@bref		���ٹر�
@param
@return
@note
**************************************************/

static void modeShutDownQuickly(void)
{
    static uint16_t delaytick = 0;
    //����һ�������GPS��һ��ʼ�Ͷ�λ�ˣ������wifi��վ�ı�־λ����4G����������������30��͹ػ��ˣ����Ҫ����primaryServerIsReady�ж�
    if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest == 0 && sysinfo.wifiRequest == 0 && sysinfo.lbsRequest == 0 && primaryServerIsReady())
    {
    	//sysparam.gpsuploadgap>=60ʱ������gpsrequest==0��ر�kernal,����һЩ��1sΪʱ�������񲻼�ʱ���ᵼ��gps�ϱ�����ʱ��acc״̬�޷��л���
    	//�����˶�������£�����û��GPS_REQUEST_ACC_CTL������ᵼ�¹ر�kernal
    	if ((sysparam.MODE == MODE21 || sysparam.MODE == MODE23) && getTerminalAccState() && sysparam.gpsuploadgap >= GPS_UPLOAD_GAP_MAX)
    	{
			delaytick = 0;
    	}
        delaytick++;
        if (delaytick >= 20)
        {
            LogMessage(DEBUG_ALL, "modeShutDownQuickly==>shutdown");
            delaytick = 0;
            changeModeFsm(MODE_STOP); //ִ����ϣ��ػ�
        }
    }
    else
    {
        delaytick = 0;
    }
}

/**************************************************
@bref		mode4�л�����ģʽ
@param
@return
@note
**************************************************/

static void mode4CloseSocketQuickly(void)
{
	static uint16_t tick = 0;
	
	if (isModuleRunNormal())
	{
		if (sysinfo.gpsRequest == 0 && sysinfo.alarmRequest == 0 && sysinfo.wifiExtendEvt == 0 && sysinfo.lbsRequest == 0)
		{
			tick++;
			if (tick >= 15)
			{
				LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>On net");
				changeMode4Callback();
				tick = 0;
			}
		}
		else
		{
			tick = 0;
		}
		sysinfo.nonetTick = 0;
	}
	else if (isModuleRunNormal() == 0 && isModuleOfflineStatus() == 0)
	{
		sysinfo.nonetTick++;
		tick = 0;
		LogPrintf(DEBUG_ALL, "sysinfo.nonetTick:%d", sysinfo.nonetTick);
		if (sysinfo.nonetTick >= 270)
		{
			sysinfo.nonetTick = 0;
			LogMessage(DEBUG_ALL, "mode4CloseSocketQuickly==>Shut down");
			modeTryToStop();
		}
	}
	else
	{
		sysinfo.nonetTick = 0;
		tick = 0;
	}
}

/**************************************************
@bref		����-���ػ�
@param
@return
@note
**************************************************/

void modeTryToStop(void)
{
    sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    sysinfo.wifiExtendEvt = 0;
    sysinfo.lbsExtendEvt = 0;
    volCheckRequestClear();
    changeModeFsm(MODE_STOP);
    LogMessage(DEBUG_ALL, "modeTryToStop");
}

/**************************************************
@bref		����
@param
@return
@note
**************************************************/

void modeTryToDone(void)
{
	sysinfo.gpsRequest = 0;
    sysinfo.alarmRequest = 0;
    sysinfo.wifiRequest = 0;
    sysinfo.lbsRequest = 0;
    sysinfo.wifiExtendEvt = 0;
    sysinfo.lbsExtendEvt = 0;
    volCheckRequestClear();
    changeModeFsm(MODE_DONE);
    LogMessage(DEBUG_ALL, "modeTryToDone");
}


/**************************************************
@bref		����ɨ��
@param
@return
@note
**************************************************/

static void modeScan(void)
{
//    static uint8_t runTick = 0;
//    scanList_s  *list;
//    if (sysparam.leavealm == 0 || (sysparam.MODE != MODE1 && sysparam.MODE != MODE3))
//    {
//        runTick = 0;
//        changeModeFsm(MODE_START);
//        return;
//    }
//    if (runTick == 1)
//    {
//        portFsclkChange(1);
//        bleCentralDiscovery();
//    }
//    else if (runTick >= 20)
//    {
//        runTick = 0;
//        list = scanListGet();
//        if (list->cnt == 0)
//        {
//            alarmRequestSet(ALARM_LEAVE_REQUEST);
//        }
//        changeModeFsm(MODE_START);
//    }
//    runTick++;
}


/**************************************************
@bref		����״̬���л�
@param
@return
@note
**************************************************/

static void bleChangeFsm(modeChoose_e fsm)
{
    bleTry.runFsm = fsm;
    bleTry.runTick = 0;
}

/**************************************************
@bref		ɨ����ɻص�
@param
@return
@note
**************************************************/

void bleScanCallBack(deviceScanList_s *list)
{
    uint8_t i;
    for (i = 0; i < list->cnt; i++)
    {
        if (my_strpach(list->list[i].broadcaseName, "AUTO"))
        {
            LogPrintf(DEBUG_ALL, "Find Ble [%s],rssi:%d", list->list[i].broadcaseName, list->list[i].rssi);
            tmos_memcpy(bleTry.mac, list->list[i].addr, 6);
            bleTry.addrType = list->list[i].addrType;
            bleChangeFsm(BLE_CONN);
            return;
        }
    }
    LogMessage(DEBUG_ALL, "no find my ble");
    bleChangeFsm(BLE_SCAN);
}

/**************************************************
@bref		������ɵ�
@param
@return
@note
**************************************************/

void bleConnCallBack(void)
{
    LogMessage(DEBUG_ALL, "connect success");
    bleChangeFsm(BLE_READY);
    dynamicParam.bleLinkCnt = 0;
    dynamicParamSaveAll();
    tmos_set_event(appCentralTaskId, APP_WRITEDATA_EVENT);
}

/**************************************************
@bref		������ɵ�
@param
@return
@note
**************************************************/

void bleTryInit(void)
{
	tmos_memset(&bleTry, 0, sizeof(bleScanTry_s));
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeStart(void)
{
	if (sysparam.pwrOnoff == 0 ||
		sysinfo.bleConnStatus == 1)
	{
		LogPrintf(DEBUG_ALL, "System power key was off");
		modeTryToDone();
		return;
	}
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
    ledStatusUpdate(SYSTEM_LED_RUN, 1);
    sysinfo.runStartTick = sysinfo.sysTick;
    sysinfo.gpsuploadonepositiontime = 180;
    updateRTCtimeRequest();
    delMsgReqSet();
    portFsclkChange(0);
    portLdrGpioCfg(1);
    portSosKeyCfg(1);
    if (sysinfo.mode4First == 0)
    {
		sysinfo.mode4First = 1;
//		lbsRequestSet(DEV_EXTEND_OF_MY);
		wifiRequestSet(DEV_EXTEND_OF_MY);
    	gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
    }
    sysinfo.nonetTick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            portSetNextAlarmTime();
            break;
        case MODE2:
            portGsensorCtl(1);
            if (sysparam.accctlgnss == 0)
            {
                gpsRequestSet(GPS_REQUEST_GPSKEEPOPEN_CTL);
            }
            break;
        case MODE3:
            portGsensorCtl(0);
            dynamicParam.startUpCnt++;
            dynamicParamSaveAll();
            break;
        case MODE21:
            portGsensorCtl(1);
            portSetNextAlarmTime();
            break;
        case MODE23:
            portGsensorCtl(1);
            break;
        /*����ģʽ*/
        case MODE4:
			portGsensorCtl(0);
			modulePowerOn();
		    netResetCsqSearch();
		    portSetNextAlarmTime();
		    portSetNextMode4AlarmTime();
		    changeModeFsm(MODE_RUNING);
        	return;
        default:
            sysparam.MODE = MODE2;
            paramSaveAll();
            break;
    }
    LogPrintf(DEBUG_ALL, "modeStart==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
    LogPrintf(DEBUG_ALL, "Mode:%d, startup:%d", sysparam.MODE, dynamicParam.startUpCnt);
    //lbsRequestSet(DEV_EXTEND_OF_MY);
    wifiRequestSet(DEV_EXTEND_OF_MY);
    gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
    
    modulePowerOn();
    netResetCsqSearch();
    changeModeFsm(MODE_RUNING);
}

static void sysRunTimeCnt(void)
{
    static uint8_t runTick = 0;
    if (++runTick >= 180)
    {
        runTick = 0;
        dynamicParam.runTime++;
        portSaveStep();
    }
}

static void moduleErrDet(void)
{
	static uint8_t tick = 0;
	if (getModulePwrState() == 0 && getTerminalAccState())
	{
		tick++;
		if (tick >= 60)
		{
		    modulePowerOn();
		}
	}
	else
	{
		tick = 0;
	}
	LogPrintf(DEBUG_ALL, "moduleErrDet==>tick = %d", tick);
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeRun(void)
{
    static uint8_t runtick = 0;
    switch (sysparam.MODE)
    {
        case MODE1:
        case MODE3:
            //��ģʽ�¹���3�ְ���
            if ((sysinfo.sysTick - sysinfo.runStartTick) >= 210)
            {
                gpsRequestClear(GPS_REQUEST_ALL);
                changeModeFsm(MODE_STOP);
            }
            modeShutDownQuickly();
            break;
        case MODE2:
            //��ģʽ��ÿ��3���Ӽ�¼ʱ��
            sysRunTimeCnt();
            gpsUploadPointToServer();
            break;
        case MODE21:
        case MODE23:
            //��ģʽ����gps����ʱ���Զ��ػ�
            sysRunTimeCnt();
            modeShutDownQuickly();
            gpsUploadPointToServer();
            moduleErrDet();
            break;
        case MODE4:
			sysRunTimeCnt();
			mode4CloseSocketQuickly();
        	break;
        default:
            LogMessage(DEBUG_ALL, "mode change unknow");
            sysparam.MODE = MODE2;
            break;
    }
}

/**************************************************
@bref		ģʽ����
@param
@return
@note
**************************************************/

static void modeStop(void)
{
    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3)
    {
        portGsensorCtl(0);
    }
    modulePowerOff();
    changeModeFsm(MODE_DONE);
}


/**************************************************
@bref		�ȴ�����ģʽ
@param
@return
@note
**************************************************/

static void modeDone(void)
{
	static uint8_t motionTick = 0;
	//���뵽���ģʽ�Ͱ�sysinfo.canRunFlag���㣬�����Ļ���Դ��GPSδ����ѹ������������
	sysinfo.canRunFlag = 0;
	static uint8_t tick = 0;
	//��֤�ػ�ʱ������gps����
	if (sysparam.pwrOnoff == 0 || sysinfo.bleConnStatus == 1)
	{
		sysinfo.gpsRequest = 0;
	}
	//��֤��mode done״̬�� ģ�鴦�ڹػ�
	if (getModulePwrState() == 1)
	{
		if (++tick >= 60)
		    modulePowerOff();
	}
	else
	{
		tick = 0;
	}
	bleTryInit();
    if (sysinfo.gpsRequest)
    {
        motionTick = 0;
        volCheckRequestSet();
        ledStatusUpdate(SYSTEM_LED_RUN, 1);
        changeModeFsm(MODE_START);
        LogMessage(DEBUG_ALL, "modeDone==>Change to mode start");
    }
    else if (sysparam.MODE == MODE1 || sysparam.MODE == MODE3 || sysparam.MODE == MODE4)
    {
    	motionTick = 0;
        //�ػ����������Ͽ����µ�����
		//Ҫ�ж�mcu������ģ��ر����
		if (isKeyPressInProgress() == 0)
		{
			if (sysinfo.sleep && getModulePwrState() == 0 && sysinfo.kernalRun)
			{
				tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
				motionTick = 0;
			}
		}
    }
    else if (sysparam.MODE == MODE23 || sysparam.MODE == MODE21 || sysparam.MODE == MODE2)
    {
//		LogPrintf(DEBUG_ALL, "motioncnt:%d, motionTick:%d key:%d hid:%d 4Gmodule:%d", 
//							motionCheckOut(sysparam.gsdettime), motionTick,
//							isKeyPressInProgress(), getHidConnStatus(), getModulePwrState());
		//�ػ����������Ͽ����µ�����
		//Ҫ�ж�mcu������ģ��ر����
		if (isKeyPressInProgress() == 0 && 
			sysinfo.sleep && 
			getModulePwrState() == 0 && 
			motionCheckOut(sysparam.gsdettime) <= 0 &&
			sysinfo.kernalRun)
		{
					LogPrintf(DEBUG_ALL, "motioncnt:%d, motionTick:%d key:%d hid:%d 4Gmodule:%d", 
							motionCheckOut(sysparam.gsdettime), motionTick,
							isKeyPressInProgress(), getHidConnStatus(), getModulePwrState());
			tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
			motionTick = 0;
		}
    }
}

/**************************************************
@bref		��ǰ�Ƿ�Ϊ����ģʽ
@param
@return
	1		��
	0		��
@note
**************************************************/

uint8_t isModeRun(void)
{
    if (sysinfo.runFsm == MODE_RUNING || sysinfo.runFsm == MODE_START)
        return 1;
    return 0;
}
/**************************************************
@bref		��ǰ�Ƿ�Ϊdoneģʽ
@param
@return
	1		��
	0		��
@note
**************************************************/

uint8_t isModeDone(void)
{
    if (sysinfo.runFsm == MODE_DONE || sysinfo.runFsm == MODE_STOP)
        return 1;
    return 0;
}


/**************************************************
@bref		ϵͳ��ʱ�Զ�����
@param
@return
@note
**************************************************/

static void sysAutoReq(void)
{
#define NO_NET_CHECK_TIME		30
    uint16_t year;
    uint8_t month, date, hour, minute, second;
    static uint16_t noNetTick = 0;
    if (sysparam.pwrOnoff == 0 || 
    	sysinfo.bleConnStatus == 1)
    {
		sysinfo.mode4NoNetTick = 0;
		sysinfo.sysMinutes = 0;
		LogPrintf(DEBUG_ALL, "sysAutoReq==>system power %s, ble conn status:%s", 
								sysparam.pwrOnoff ? "on" : "off", sysinfo.bleConnStatus ? "Yes" : "No");
		return;
    }

    if (sysparam.MODE == MODE1 || sysparam.MODE == MODE21)
    {
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (date == sysinfo.alarmDate && hour == sysinfo.alarmHour && minute == sysinfo.alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysinfo.kernalRun == 0)
            {
            	volCheckRequestSet();
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
    }
    else if (sysparam.MODE == MODE4)
    {
    	portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (hour == sysinfo.mode4alarmHour && minute == sysinfo.mode4alarmMinute)
        {
            LogPrintf(DEBUG_ALL, "sysAutoReq==>MODE4:%02d/%02d/%02d %02d:%02d:%02d", year, month, date, hour, minute, second);
            gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
            if (sysinfo.kernalRun == 0)
            {
            	volCheckRequestSet();
                tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
            }
        }
		if (isModeDone())
		{
			sysinfo.mode4NoNetTick++;
			LogPrintf(DEBUG_ALL, "mode4NoNetTick:%d", sysinfo.mode4NoNetTick);
			if (sysinfo.mode4NoNetTick >= NO_NET_CHECK_TIME)
			{
				sysinfo.mode4NoNetTick = 0;
                LogMessage(DEBUG_ALL, "mode 4 restoration network");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                    
                }
                changeModeFsm(MODE_START);
			}
		}
		else
		{
			sysinfo.mode4NoNetTick = 0;
		}
    }
    else
    {
        //ģʽ2û�����߼�
    	if (isModeDone() && sysparam.MODE == MODE2)
        {
			noNetTick++;
			LogPrintf(DEBUG_ALL, "mode2NoNetTick:%d ", noNetTick);
			if (noNetTick >= NO_NET_CHECK_TIME)
			{
				noNetTick = 0;
				LogMessage(DEBUG_ALL, "mode 2 restoration network");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
	            changeModeFsm(MODE_START);
			}
        }
        else
        {
			noNetTick = 0;
        }
        if (sysparam.gapMinutes != 0)
        {
            sysinfo.sysMinutes++;
            if (sysinfo.sysMinutes % sysparam.gapMinutes == 0)
            {
            	sysinfo.sysMinutes = 0;
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
                LogMessage(DEBUG_ALL, "upload period");
                if (sysinfo.kernalRun == 0)
                {
                	volCheckRequestSet();
                    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
                }
            }
        }
    }
}

/**************************************************
@bref		��ص͵�ػ����
@param
@return
@note	0�����ڼ��				1��������
**************************************************/

uint8_t SysBatDetection(void)
{
	static uint8_t waitTick = 0;
	static uint8_t ind = 0;
	uint8_t cnt = 0;
	/*��������ѹ*/
	if (sysinfo.volCheckReq == 0)
    {
		/* ����MCU��ȡ������ѹҲ��ģ�������ѹ�����ÿ�δ�kernal��ʱ�򣬿���ģ����磬����ADC��������ѹ */
		moduleSupplyOn();
		
		sysinfo.bat_det_val[ind] = sysinfo.outsidevoltage;
		ind = ((ind + 1) % BAT_DET_TIME);
		waitTick++;
		LogPrintf(DEBUG_ALL, "SysBatDetection==>%.2f", sysinfo.outsidevoltage);
		if (waitTick == BAT_DET_TIME)
		{
			for (uint8_t i = 0; i < BAT_DET_TIME; i++)
				if (sysinfo.bat_det_val[i] > BAT_DET_CAN_RUN_VOL) cnt++;
			
			if (cnt >= (BAT_DET_TIME - 1))
			{
				sysinfo.canRunFlag = 1;
				waitTick = 0;
				volCheckRequestClear();
				sysinfo.pwrDone = 0;
				LogPrintf(DEBUG_ALL, "[%d]��ص�ѹ��������������", cnt);
				return 0;
			}
			else
			{
				waitTick = 0;
				sysinfo.canRunFlag = 0;
				modeTryToDone();
				volCheckRequestClear();
				sysinfo.pwrDone = 0;
				LogPrintf(DEBUG_ALL, "[%d]��ص�ѹƫ�ͣ��ػ�", cnt);
				return 1;
			}
		}
		else
		{
			return 0;
		}
    }
    waitTick = 0;
    ind = 0;
	/* ����ʱ����ѹ */
	/* ����ģ�鹩���޷���ȡadc */
	/* ���ܹ��� */
	if (sysinfo.canRunFlag == 0)
	{
		/*������ڹ���*/
		if (sysinfo.runFsm == MODE_RUNING)
		{
			modeTryToStop();
			if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
			{
				if (sysinfo.gsensorOnoff == 1)
				{
					portGsensorCtl(0);
				}
			}
		}
		else if (sysinfo.runFsm == MODE_START)
		{
			modeTryToDone();
		}
	}
	/*���Թ���*/
	else
	{
		if (sysparam.MODE == MODE2 || sysparam.MODE == MODE21 || sysparam.MODE == MODE23)
		{
			if (sysinfo.gsensorOnoff == 0 && sysparam.pwrOnoff && sysinfo.bleConnStatus == 0)
			{
				portGsensorCtl(1);
			}
		}
	}
	return 1;
}

/**************************************************
@bref		��ؼ����������
@param
@return
@note
**************************************************/

void volCheckRequestSet(void)
{
	sysinfo.volCheckReq = 0;
	sysinfo.canRunFlag = 0;
	tmos_memset(sysinfo.bat_det_val, 0, BAT_DET_TIME);
	LogMessage(DEBUG_ALL, "volCheckRequestSet==>OK");
}

/**************************************************
@bref		��ؼ���������
@param
@return
@note
**************************************************/

void volCheckRequestClear(void)
{
	sysinfo.volCheckReq = 1;
	LogMessage(DEBUG_ALL, "volCheckRequestClear==>OK");
}

/**************************************************
@bref		ģʽ��������
@param
@return
@note
**************************************************/

static void sysModeRunTask(void)
{
	if (SysBatDetection() != 1)
	{
		return;
	}
    switch (sysinfo.runFsm)
    {
        case MODE_START:
            modeStart();
            break;
        case MODE_RUNING:
            modeRun();
            break;
        case MODE_STOP:
            modeStop();
            break;
        case MODE_DONE:
            modeDone();
            break;
    }
}

/**************************************************
@bref		��վ��������
@param
@return
@note
**************************************************/

void lbsRequestSet(uint8_t ext)
{
	if (!(sysinfo.lbsExtendEvt & ext))	//���˵��ظ�lbs����
    	sysinfo.lbsRequest = 1;
    sysinfo.lbsExtendEvt |= ext;
}

/**************************************************
@bref		�����վ��������
@param
@return
@note
**************************************************/

void lbsRequestClear(void)
{
	sysinfo.lbsRequest = 0;
    sysinfo.lbsExtendEvt = 0;
}

static void sendLbs(void)
{
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_MY)
    {
        protocolSend(NORMAL_LINK, PROTOCOL_19, NULL);
    }
    if (sysinfo.lbsExtendEvt & DEV_EXTEND_OF_BLE)
    {
        protocolSend(BLE_LINK, PROTOCOL_19, NULL);
    }
    sysinfo.lbsExtendEvt = 0;
}

/**************************************************
@bref		��վ��������
@param
@return
@note
**************************************************/

static void lbsRequestTask(void)
{
    if (sysinfo.lbsRequest == 0)
    {
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    sysinfo.lbsRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
        moduleGetLbs();
        startTimer(20, sendLbs, 0);
    }
}

/**************************************************
@bref		wifi��ʱ����
@param
@return
@note
**************************************************/

void wifiTimeout(void)
{
	LogMessage(DEBUG_ALL, "wifiTimeout");
	lbsRequestSet(DEV_EXTEND_OF_MY);
	wifiRequestClear();
	wifiTimeOutId = -1;
}

/**************************************************
@bref		wifiӦ��ɹ�
@param
@return
@note
**************************************************/

void wifiRspSuccess(void)
{
	if (wifiTimeOutId != -1)
	{
		stopTimer(wifiTimeOutId);
		wifiTimeOutId = -1;
		LogMessage(DEBUG_ALL, "wifiRspSuccess");
		sysinfo.lockTick = 125;
	}
}

/**************************************************
@bref		����WIFI��������
@param
@return
@note
**************************************************/

void wifiRequestSet(uint8_t ext)
{
	LogPrintf(DEBUG_ALL, "wifiRequestSet==>ext:0x%02x  0x%02x  %d", sysinfo.wifiExtendEvt, ext, !(sysinfo.wifiExtendEvt & ext));
	if (!(sysinfo.wifiExtendEvt & ext))	//���˵��ظ�wifi����
    	sysinfo.wifiRequest = 1;
    sysinfo.wifiExtendEvt |= ext;
}

/**************************************************
@bref		���WIFI��������
@param
@return
@note
**************************************************/

void wifiRequestClear(void)
{
	sysinfo.wifiRequest = 0;
	sysinfo.wifiExtendEvt = 0;
	wifiRspSuccess();
}


/**************************************************
@bref		WIFI��������
@param
@return
@note
**************************************************/

static void wifiRequestTask(void)
{
	static uint8_t tick = 0;
    if (sysinfo.wifiRequest == 0)
    {
    	tick = 0;
        return;
    }
    if (primaryServerIsReady() == 0)
        return;
    //��AGPS�ȵ�AGPS���ݶ�ȡ�����ٷ�WIFI����
    if (sysinfo.agpsRequest)
    {
    	tick++;
    	if (tick >= 15)
    	{
    		tick = 0;
			goto __WIFI;
    	}
    	return;
    }
__WIFI:
	tick = 0;
    sysinfo.wifiRequest = 0;
    if (sysparam.protocol == ZT_PROTOCOL_TYPE)
    {
		startTimer(30, moduleGetWifiScan, 0);        
        wifiTimeOutId = startTimer(300, wifiTimeout, 0);
    }
}

/**************************************************
@bref		�����豸
@param
@return
@note
**************************************************/
void wakeUpByInt(uint8_t      type, uint8_t sec)
{
    switch (type)
    {
        case 0:
            sysinfo.ringWakeUpTick = sec;
            break;
        case 1:
        	if (sec > sysinfo.cmdTick)
            	sysinfo.cmdTick = sec;
            break;
        case 2:
        	sysinfo.irqTick = sec;
        	break;
    }

    portSleepDn();
}

/**************************************************
@bref		��ѯ�Ƿ���Ҫ����
@param
@return
@note
**************************************************/

static uint8_t getWakeUpState(void)
{
    //��ӡ������Ϣʱ��������
    if (sysinfo.logLevel == DEBUG_FACTORY)
    {
        return 1;
    }
    //δ������������
    if (primaryServerIsReady() == 0 && isModeRun() && sysparam.MODE != MODE4)
    {
        return 2;
    }
    //��gpsʱ��������
    if (sysinfo.gpsRequest != 0)
    {
        return 3;
    }
    if (sysinfo.ringWakeUpTick != 0)
    {
        return 4;
    }
    if (sysinfo.cmdTick != 0)
    {
        return 5;
    }
    if (sysinfo.irqTick != 0)
    {
		return 6;
    }
    if (sysparam.MODE == MODE4 && isModeRun() && isModuleOfflineStatus() == 0)
    {
    	return 7;
    }
    //��0 ʱǿ�Ʋ�����
    return 0;
}

/**************************************************
@bref		�Զ�����
@param
@return
@note
**************************************************/

void autoSleepTask(void)
{
    static uint8_t flag = 0;
    if (sysinfo.ringWakeUpTick != 0)
    {
        sysinfo.ringWakeUpTick--;
    }
    if (sysinfo.cmdTick != 0)
    {
        sysinfo.cmdTick--;
    }
    if (sysinfo.irqTick != 0)
    {
		sysinfo.irqTick--;
    }
    uint8_t a = getWakeUpState();
    //LogPrintf(DEBUG_ALL, "getWakeUpState:%d", a);
    if (a)
    {
        portSleepDn();
        if (flag != 0)
        {
            flag = 0;
            portFsclkChange(0);
            LogMessage(DEBUG_ALL, "disable sleep");
            tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
            sysinfo.sleep = 0;
            //portDebugUartCfg(1);
        }
    }
    else
    {
        portSleepEn();
        if (flag != 1)
        {
            flag = 1;
            portFsclkChange(1);
            LogMessage(DEBUG_ALL, "enable sleep");
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            sysinfo.sleep = 1;
            //portDebugUartCfg(0);
        }
    }
}

/**************************************************
@bref		ÿ������
@param
@note
**************************************************/

static void rebootEveryDay(void)
{
    sysinfo.sysTick++;
    if (sysparam.pwrOnoff)
    {
    	if (sysinfo.ledTick > 0)
			sysinfo.ledTick--;
    }
    //    if (sysinfo.sysTick < 86400)
    //        return ;
    //    if (sysinfo.gpsRequest != 0)
    //        return ;
    //    portSysReset();
}

static void tiltDetectionTask(void)
{
	static uint8_t tick = 0;
    if (sysparam.tiltalm == 0)
    {
        return;
    }
    if (sysinfo.doMotionFlag == 0)
    {
    	tick = 0;
        return;
    }
    /*��ִ����б*/
	if (sysinfo.tiltNormalCnt >= 2)
	{
//		if (ROLL_DET)
//		{
//			tick++;
//			if (tick > sysparam.tiltThreshold)
//			{
//				alarmRequestSet(ALARM_TILT_REQUEST);
//		        LogMessage(DEBUG_ALL, "Tilt alarm");
//		    	jt808UpdateAlarm(JT808_TILT_ALARM, 1);
//				lbsRequestSet(DEV_EXTEND_OF_MY);
//				wifiRequestSet(DEV_EXTEND_OF_MY);
//				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
//				tick = 0;
//				sysinfo.doMotionFlag = 0;
//				sysinfo.tiltNormalCnt = 0;
//			}
//		}
//		else
//		{
//			tick= 0;
//			sysinfo.doMotionFlag = 0;
//			sysinfo.tiltNormalCnt = 0;
//		}
		
	}
	/*����ִ����б*/
	else
	{
		tick = 0;
		LogMessage(DEBUG_ALL, "Tilt alarm cancel");
		sysinfo.doMotionFlag = 0;
		sysinfo.tiltNormalCnt = 0;
	}

}


/**************************************************
@bref		�����豸����״̬ʱ��
@param
@return
@note	
����豸���ڰ�״̬���ߴ���ƽ��״̬��ÿһ���Ӽ���+1
**************************************************/

void calculateNormalTime(void)
{

}

/**************************************************
@bref		�й�����
@param
@return
@note
**************************************************/
static void lightDetectionTask(void)
{
	if (sysparam.ldrEn == 0)
		return;
	//LogPrintf(DEBUG_ALL, "ldr:%d", LDR_READ);
	if (sysinfo.ldrIrqFlag)
	{
		//��
		if (sysinfo.ldrDarkCnt >= 2)
		{
			LogMessage(DEBUG_ALL, "Light alarm");
			alarmRequestSet(ALARM_LIGHT_REQUEST);
			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
			jt808UpdateAlarm(JT808_LIGHT_ALARM, 1);
			//lbsRequestSet(DEV_EXTEND_OF_MY);
			wifiRequestSet(DEV_EXTEND_OF_MY);		
		}
		else
		{
			LogMessage(DEBUG_ALL, "Light alarm cancel");
		}
		sysinfo.ldrIrqFlag = 0;
		sysinfo.ldrDarkCnt = 0;
	}
	else
	{
		//��
		
	}
}

/**************************************************
@bref       gsensor�������
@param
@note
**************************************************/
static void gsensorRepair(void)
{
    portGsensorCtl(1);
    LogMessage(DEBUG_ALL, "repair gsensor");
}

static void gsCheckTask(void)
{
    static uint8_t tick = 0;
    static uint8_t errorcount = 0;
    if (sysinfo.gsensorOnoff == 0)
    {
        tick = 0;
        return;
    }

    tick++;
    if (tick % 60 == 0)
    {
        tick = 0;
        if (readInterruptConfig() != 0)
        {
            LogMessage(DEBUG_ALL, "gsensor error");
            portGsensorCtl(0);
            startTimer(20, gsensorRepair, 0);

        }
        else
        {
            errorcount = 0;
        }
    }
}

/**************************************************
@bref		system shutdown callback
@param
@return
@note
**************************************************/

void systemShutDownCB(void)
{
	tmos_start_task(sysinfo.taskId, APP_TASK_SYSTEM_SHUTDOWN_EVENT, MS1_TO_SYSTEM_TIME(1000));
}

/**************************************************
@bref		��ʾ���ڰ��°���
@param
@return
@note		Ŀ����Ϊ�˲���kernel�ر�
**************************************************/

uint8_t isKeyPressInProgress(void)
{
	return sysinfo.keypress;
}

/**************************************************
@bref		��Դ�����������
@param
@return
@note
 once��ֹ�����������Ų���
**************************************************/
#define KEY_ON		1
#define KEY_OFF		0
#define KEY_RESULT_PRESS	1
#define KEY_RESULT_CLICK	2
#define KEY_LONG_PRESS_TIME		20
void keyScan(void)
{
	/* �жϳ������� */
    static uint32_t cnt = 0;
    /* �ж��������� */
    static uint8_t laststatus = 0;	//�����ǰ����ɿ��� �����500ms֮�����������ȥ�ͻ��laststatus��1
    static int8_t clicktime;		//���֮��ļ��
    static uint8_t clickcnt = 1;	//�������
    /* ��ֹ���Ų����� */
	static uint8_t once = 0;		//0:��⵽�ɿ�������	1:�Ѿ��ж�Ϊһ�ְ�����ʽ�� ������ɿ���������ȥ�ǲ����ж��µİ�����ʽ��  ���ڷ�ֹһֱ���ż�����
	/* ���������жϽ�� */
	uint8_t result = 0;
    if (PWR_KEY_READ == KEY_ON)//���ְ��������»��߱����¹�
    {
    	sysinfo.keypress = 1;
    	//���ڴ����ػ��߼������ټ��㰴�����µ�ʱ��
    	//Ŀ����Ϊ�˷�ִֹ�йػ��Ĺ����У�����ٴΰ��£����豸ִ����ػ��������ȥִ�п���
    	if (sysinfo.pwrDone)
    		return;
    	
    	//ֻҪ���£��ƾ͹���30��
		if (sysparam.ledctrl && sysparam.pwrOnoff)
			sysinfo.ledTick = 30;
        cnt++;
        //�ж�Ϊ����
        if (cnt >= KEY_LONG_PRESS_TIME && once == 0)
        {
            once = 1;
            result = KEY_RESULT_PRESS;
            LogMessage(DEBUG_ALL, "Key Press");
        }

        if (laststatus && once == 0)
        {
            laststatus = 0;
            clickcnt++;
        }
        //��������
        if (clickcnt == 3 && once == 0)
        {
            once = 1;
            clickcnt = 0;
            LogMessage(DEBUG_ALL, "Key Triclick");
            uint8_t Count = appHidGetBondCount();
			uint8_t ret;
			ret = appHidRemoveAllBond();
			LogPrintf(DEBUG_ALL, "Number of existing bond is %d,eraser them %s", Count, ret ? "Fail" : "Success");
        }
        //����n��֮��ļ��
        if (cnt >= 10)
        {
            clickcnt = 0;
        }
        clicktime = 5;
    }
    //�ɿ�����
    if (PWR_KEY_READ == KEY_OFF)
    {
    	sysinfo.keypress = 0;
    	if (cnt < KEY_LONG_PRESS_TIME && cnt > 0)
    	{
			result = KEY_RESULT_CLICK;
			LogMessage(DEBUG_ALL, "Key Click");
    	}
        clicktime--;
        cnt = 0;
        once = 0;
        if (clicktime >= 0 )
        {
            laststatus = 1;
        }
        else
        {
            laststatus = 0;
            clickcnt = 1;
        }
        if (clicktime <= -100)//��ֹ����ͷ��
        {
            clicktime = -1;
        }
    }

    /* ִ�н�� */
    if (result == KEY_RESULT_PRESS)
    {
		if (sysparam.pwrOnoff && sysinfo.pwrDone == 0)
		{
			LogPrintf(DEBUG_ALL, "System power off");
			sysinfo.gpsRequest = 0;
		    sysinfo.alarmRequest = 0;
		    sysinfo.wifiRequest = 0;
		    sysinfo.lbsRequest = 0;
		    sysinfo.wifiExtendEvt = 0;
		    sysinfo.lbsExtendEvt = 0;
		    portLdrGpioCfg(0);
			portGsensorCtl(0);
			portSosKeyCfg(0);
			ledStatusUpdate(SYSTEM_LED_RUN, 0);
			sysinfo.pwrDone = 1;
			//�������������־
			sysinfo.bleConnStatus = 0;
			//�ر�����
			appHidBroadcastCtl(0);
			//����ػ�ǰ��Ҫ���ض��Ĳ�����ʹ��systemShutDownCB,����Ҫ�ض�����ֱ���л���modedone/modestop�ͺ���
			systemShutDownCB();
			sysparam.pwrOnoff = 0;
			paramSaveAll();
		}
		else if (sysparam.pwrOnoff == 0 && sysinfo.pwrDone == 0)
		{
			sysparam.pwrOnoff = 1;
			gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
			LogPrintf(DEBUG_ALL, "System power on");
			sysinfo.pwrDone = 1;
			sysinfo.ledTick = 180;
			ledStatusUpdate(SYSTEM_LED_RUN, 1);
			appHidBroadcastCtl(1);
			paramSaveAll();
		}
		else
		{
			LogPrintf(DEBUG_ALL, "System in progress");
		}
    }
//    else if (result == KEY_RESULT_CLICK)
//    {
//		if (sysparam.ledctrl && sysparam.pwrOnoff)
//			sysinfo.ledTick = 30;
//    }
    //LogPrintf(DEBUG_ALL, "soskeycnt=%d", cnt);
}

/**************************************************
@bref		sos�������
@param
@return
@note
 once��ֹ�����������Ų���
**************************************************/

void keySosScan(void)
{
    static uint32_t cnt = 0;
    static uint8_t laststatus = 0;//�����ǰ����ɿ��� �����500ms֮�����������ȥ�ͻ��laststatus��1
    static uint8_t once = 0;//0����⵽�ɿ�������  1:�Ѿ��ж�Ϊһ�ְ�����ʽ�� ������ɿ���������ȥ�ǲ����ж��µİ�����ʽ��  ���ڷ�ֹһֱ���ż�����
    static int8_t clicktime ;//���֮��ļ��
    static uint8_t clickcnt = 1;//�������
	if (sysparam.pwrOnoff == 0)
	{
		return;
	}
	if (sysinfo.soskeyOnoff == 0)
	{
		cnt = 0;
		return;
	}
    if (SOS_KEY_READ == 0)//���ְ��������»��߱����¹�
    {
        cnt++;
        //�ж�Ϊ����
        if (cnt >= 30 && once == 0)
        {
            sysinfo.doSosFlag = 1;
            once = 1;
            LogMessage(DEBUG_ALL, "SOS Key press");
            alarmRequestSet(ALARM_SOS_REQUEST);
        }

        if (laststatus && once == 0)
        {
            laststatus = 0;
            clickcnt++;
        }
        //��������
        if (clickcnt == 3 && once == 0)
        {
            once = 1;
            //soskey.triclick = 1;
            clickcnt = 0;
            LogMessage(DEBUG_ALL, "SOS Key Triclick");
        }
        if (cnt >= 5)
        {
            clickcnt = 0;
        }
        clicktime = 5;
    }
    //�ɿ�����
    if (SOS_KEY_READ)
    {
        clicktime--;
        cnt = 0;
        once = 0;
        if (clicktime >= 0 )
        {
            laststatus = 1;
        }
        else
        {
            laststatus = 0;
            clickcnt = 1;
        }
        if (clicktime <= -100)//��ֹ����ͷ��
        {
            clicktime = -1;
        }
    }
    //LogPrintf(DEBUG_ALL, "soskeycnt=%d", cnt);
}


/**************************************************
@bref		123�ϱ�����
@param
@return
@note
**************************************************/
static void mode123UploadTask(void)
{
	gpsinfo_s *gpsinfo;
	static uint32_t uploadTick = 0, nofixTick = 0;
	if (gpsRequestGet(GPS_REQUEST_123_CTL) == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	if (sysinfo.mode123Min == 0)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		return;
	}
	/* ����ʱ����� */
	if (sysinfo.mode123RunTick >= sysinfo.mode123Min * 60)
	{
		uploadTick = 0;
		sysinfo.mode123RunTick = 0;
		nofixTick = 0;
		gpsRequestClear(GPS_REQUEST_123_CTL);
		return;
	}
	if (sysinfo.mode123GpsFre < GPS_UPLOAD_GAP_MAX)
		calculateTheGPSCornerPoint();
	LogPrintf(DEBUG_ALL, "123runtick:%d nofixTick:%d uploadTick:%d | mode123Min:%d mode123GpsFre:%d", 
				sysinfo.mode123RunTick, nofixTick, uploadTick, sysinfo.mode123Min * 60, sysinfo.mode123GpsFre);
	/* ֻҪһ�����Ϳ�ʼ��ʱ */
	sysinfo.mode123RunTick++;
	nofixTick++;
	gpsinfo = getCurrentGPSInfo();
	if (gpsinfo->fixstatus == 0)
	{
		//����λ��ȡ�����sportsָ��
		if (nofixTick >= sysinfo.mode123Min * 60)
		{
			sysinfo.mode123RunTick = sysinfo.mode123Min * 60;
			wifiRequestSet(DEV_EXTEND_OF_MY);
		}
		return;
	}
	nofixTick = 0;
	uploadTick++;
	
	if (uploadTick >= sysinfo.mode123GpsFre)
	{
		protocolSend(NORMAL_LINK, PROTOCOL_12, getCurrentGPSInfo());
        jt808SendToServer(TERMINAL_POSITION, getCurrentGPSInfo());
        initLastPoint(getCurrentGPSInfo());
        uploadTick = 0;
	}
}

void bleConnCheckTask(void)
{
	
	if (getHidConnStatus() == 0)
	{
		sysinfo.con_tick = 0;
		if (sysinfo.discon_tick < 200)
			sysinfo.discon_tick++;
		if (sysinfo.discon_tick >= 10)
		{
			if (sysinfo.bleConnStatus)
			{
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
				sysinfo.ledTick = 180;
				sysinfo.bleConnStatus = 0;
				
				LogPrintf(DEBUG_ALL, "Ble lost!!!!!!!!!");
			}
		}
		return;
	}

//	if (getHidConnStatus() == 0)
//		return;
	sysinfo.discon_tick = 0;
	if (sysinfo.con_tick < 200)
		sysinfo.con_tick++;
	if (sysinfo.con_tick >= 3)
	{
		if (sysinfo.bleConnStatus == 0)
		{
			sysinfo.bleConnStatus = 1;
			LogPrintf(DEBUG_ALL, "Ble connected!!!!!!!!!!");
			sysinfo.gpsRequest = 0;
		    sysinfo.alarmRequest = 0;
		    sysinfo.wifiRequest = 0;
		    sysinfo.lbsRequest = 0;
		    sysinfo.wifiExtendEvt = 0;
		    sysinfo.lbsExtendEvt = 0;
		    portLdrGpioCfg(0);
			portGsensorCtl(0);
			
			if (sysinfo.runFsm <= MODE_RUNING)
			{
				modeTryToStop();
			}
			else
			{
				modeTryToDone();
			}
			//�ػ���ʱ�����һЩ��Ҫ�Ĳ���
			sysinfo.mode123RunTick = 0;
			sysinfo.mode123Min = 0;
			sysinfo.mode123GpsFre = 0;
		}
	}
}

void bleDisConnCheckTask(void)
{
	if (getHidConnStatus() == 0)
	{
		sysinfo.con_tick = 0;
		if (sysinfo.discon_tick < 200)
			sysinfo.discon_tick++;
		if (sysinfo.discon_tick >= 3)
		{
			if (sysinfo.bleConnStatus)
			{
				gpsRequestSet(GPS_REQUEST_UPLOAD_ONE);
				sysinfo.ledTick = 180;
				sysinfo.bleConnStatus = 0;
				
				LogPrintf(DEBUG_ALL, "Ble lost!!!!!!!!!");
			}
		}
		return;
	}
}
/**************************************************
@bref		1������
@param
@return
@note
**************************************************/

void taskRunInSecond(void)
{
    rebootEveryDay();
    gpsRequestTask();	//gps״̬����/���
    motionCheckTask();	//gsensor״̬����/���
    bleConnCheckTask();
	if (sysparam.pwrOnoff && sysinfo.bleConnStatus == 0)
	{
	    netConnectTask();
	    gsCheckTask();
	    voltageCheckTask();
	    alarmRequestTask();
	    gpsUplodOnePointTask();
	    lbsRequestTask();
	    wifiRequestTask();
	    mode123UploadTask();
	    serverManageTask();
    }
    autoSleepTask();
    sysModeRunTask();
}


/**************************************************
@bref		���ڵ��Խ���
@param
@return
@note
**************************************************/
void doDebugRecvPoll(uint8_t *msg, uint16_t len)
{
    static uint8_t gpsRestore[DEBUG_BUFF_SIZE + 1];
    static uint16_t size = 0;
    uint16_t i, begin;
    if (len + size > DEBUG_BUFF_SIZE)
    {
        size = 0;
    }
    memcpy(gpsRestore + size, msg, len);
    size += len;
    begin = 0;
    for (i = 0; i < size; i++)
    {
        if (gpsRestore[i] == '\n')
        {
            atCmdParserFunction(gpsRestore + begin, i - begin + 1);
            begin = i + 1;
        }
    }
    if (begin != 0)
    {
        memmove(gpsRestore, gpsRestore + begin, size - begin);
        size -= begin;
    }
}


/*����ר������*/
void debugtask(void)
{
    static uint8_t fsm;
    static uint8_t tick;
    switch(fsm)
    {
    case 0:
        modulePowerOn();
        //portGsensorCtl(1);
        fsm = 1;
        break;

    case 1:
        tick++;
        if (tick>=5)
        {
            tick = 0;
        	fsm = 3;
        	modulePowerOff();
//        	moduleSleepCtl(1);
        }
//        else
//        {
//			netConnectTask();
//			serverManageTask();
//        }   
        break;
    case 2:
    	LogMessage(DEBUG_ALL, "SHUT DOWN");
        tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
        tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
		portUartCfg(APPUSART2, 0, 115200, doDebugRecvPoll);
        portSleepEn();
        fsm = 3;
        break;
    case 3:
        tick++;
        if (tick>=5)
        {
            modulePowerOff();
            tick = 0;
            fsm = 4;
            tmos_stop_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT);
            tmos_set_event(sysinfo.taskId, APP_TASK_STOP_EVENT);
            portSleepEn();
        }
        break;
    }
}

/**************************************************
@bref		ϵͳ����ʱ����
@param
@return
@note
**************************************************/

void myTaskPreInit(void)
{
    tmos_memset(&sysinfo, 0, sizeof(sysinfo));
    paramInit();
    sysinfo.logLevel = 9;
    SetSysClock(CLK_SOURCE_PLL_60MHz);
    portGpioSetDefCfg();
    portUartCfg(APPUSART2, 1, 115200, doDebugRecvPoll);
    portModuleGpioCfg(1);
    portGpsGpioCfg(1);
    portLedGpioCfg(1);
    portAdcCfg(1);
	portLdrGpioCfg(0);
    portWdtCfg();
    portPwrKeyCfg();
    portGsensorCtl(0);
    bleTryInit();
    socketListInit();
	sysinfo.ledTick = 180;//�����ƹ���3����
    createSystemTask(ledTask, 1);
    createSystemTask(outputNode, 2);
	createSystemTask(keyScan, 1);
	createSystemTask(keySosScan, 1);
    sysinfo.sysTaskId = createSystemTask(taskRunInSecond, 10);

    LogMessage(DEBUG_ALL, ">>>>>>>>>>>>>>>>>>>>>");
    LogPrintf(DEBUG_ALL,  "SYS_GetLastResetSta:%x", SYS_GetLastResetSta());
	if (sysparam.pwrOnoff)
	{
		volCheckRequestSet();
		ledStatusUpdate(SYSTEM_LED_RUN, 1);
	}
	else
	{
		volCheckRequestClear();
	}
}

/**************************************************
@bref		tmos ����ص�
@param
@return
@note
**************************************************/

static tmosEvents myTaskEventProcess(tmosTaskID taskID, tmosEvents events)
{
	uint8_t ret;
	uint16_t year;
	uint8_t  month = 0, date = 0, hour = 0, minute = 0, second = 0;
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        if ((pMsg = tmos_msg_receive(sysinfo.taskId)) != NULL)
        {
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & APP_TASK_KERNAL_EVENT)
    {
        kernalRun();
        moduleRequestTask();
        portWdtFeed();
        return events ^ APP_TASK_KERNAL_EVENT;
    }

    if (events & APP_TASK_POLLUART_EVENT)
    {
        pollUartData();
        portWdtFeed();
        return events ^ APP_TASK_POLLUART_EVENT;
    }

	if (events & APP_TASK_SYSTEM_SHUTDOWN_EVENT)
	{
		LogPrintf(DEBUG_ALL, "System power off done");
		
		if (sysinfo.runFsm <= MODE_RUNING)
		{
			modeTryToStop();
		}
		else
		{
			modeTryToDone();
		}
		portLdrGpioCfg(0);
		portGsensorCtl(0);
		sysinfo.pwrDone = 0;
		
		//�ػ���ʱ�����һЩ��Ҫ�Ĳ���
		sysinfo.mode123RunTick = 0;
		sysinfo.mode123Min = 0;
		sysinfo.mode123GpsFre = 0;
		if (sysinfo.logLevel == DEBUG_FACTORY)
			sysinfo.logLevel = 0;
		sysinfo.nmeaOutPutCtl = 0;
		sysinfo.mode4First = 0;
		return events ^ APP_TASK_SYSTEM_SHUTDOWN_EVENT;
	}
	
    if (events & APP_TASK_RUN_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal start");
        sysinfo.kernalRun = 1;
        /*��������IO*/
		portModuleGpioCfg(1);
		portGpsGpioCfg(1);
		//portLedGpioCfg(1);
		portAdcCfg(1);
		portWdtCfg();
        tmos_start_reload_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT, MS1_TO_SYSTEM_TIME(100));
        return events ^ APP_TASK_RUN_EVENT;
    }
    
    if (events & APP_TASK_STOP_EVENT)
    {
        LogMessage(DEBUG_ALL, "Task kernal stop");
        sysinfo.kernalRun = 0;
        motionClear();
        /* �ر�����IO */
		portAdcCfg(0);
		portModuleGpioCfg(0);
		portGpsGpioCfg(0);
		//portLedGpioCfg(0);
		if (sysinfo.pwrDone)
		{
			sysinfo.pwrDone = 0;
		}
		portWdtCancel();
		if (sysparam.pwrOnoff == 0)
       		tmos_stop_task(sysinfo.taskId, APP_TASK_KERNAL_EVENT);
        return events ^ APP_TASK_STOP_EVENT;
    }
    
    if (events & APP_TASK_ONEMINUTE_EVENT)
    {
    	sysinfo.oneMinTick++;
    	sysAutoReq();
    	calculateNormalTime();
        LogMessage(DEBUG_ALL, "***************************Task one minutes**********************");
        LogPrintf(DEBUG_ALL,  "*Mode: %d, rungap: %d, System run: %d min, darktime: %d, oneminute: %d*", sysparam.MODE, sysparam.gapMinutes, sysinfo.sysMinutes, sysinfo.ldrDarkCnt, sysinfo.oneMinTick);
        LogMessage(DEBUG_ALL, "*****************************************************************");
        portGetRtcDateTime(&year, &month, &date, &hour, &minute, &second);
        if (hour == 0 && minute == 0)
        {
            LogPrintf(DEBUG_BLE, "time of zero");
            portClearStep();
        }
        return events ^ APP_TASK_ONEMINUTE_EVENT;
    }

	if (events & APP_TASK_SYS_RESET_EVENT)
	{
		portSysReset();
		return events ^ APP_TASK_SYS_RESET_EVENT;
	}

    return 0;
}

/**************************************************
@bref		�����ʼ��
@param
@return
@note
**************************************************/

void myTaskInit(void)
{
    sysinfo.taskId = TMOS_ProcessEventRegister(myTaskEventProcess);
    tmos_set_event(sysinfo.taskId, APP_TASK_RUN_EVENT);
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_POLLUART_EVENT, MS1_TO_SYSTEM_TIME(50));
    tmos_start_reload_task(sysinfo.taskId, APP_TASK_ONEMINUTE_EVENT, MS1_TO_SYSTEM_TIME(60000));
}

