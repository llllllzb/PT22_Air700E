#include <app_protocol.h>
#include "app_net.h"

#include "app_db.h"
#include "app_gps.h"
#include "app_instructioncmd.h"
#include "app_kernal.h"
#include "app_param.h"
#include "app_sys.h"
#include "app_task.h"
#include "app_server.h"
#include "app_socket.h"
#include "app_jt808.h"

//������ؽṹ��

static moduleState_s  moduleState;
static moduleCtrl_s moduleCtrl;
static cmdNode_s *headNode = NULL;


//ָ���
const atCmd_s cmdtable[] =
{
    {AT_CMD, "ATE0"},
    {CPIN_CMD, "AT+CPIN"},
    {CGDCONT_CMD, "AT+CGDCONT"},
    {CSQ_CMD, "AT+CSQ"},
    {CGREG_CMD, "AT+CGREG"},
    {CEREG_CMD, "AT+CEREG"},
    {CIMI_CMD, "AT+CIMI"},
    {CGSN_CMD, "AT+CGSN"},
    {ICCID_CMD, "AT+ICCID"},
    {CMGF_CMD, "AT+CMGF"},
    {CMGR_CMD, "AT+CMGR"},
    {CMGD_CMD, "AT+CMGD"},
    {CMGS_CMD, "AT+CMGS"},
    {CPMS_CMD, "AT+CPMS"},
    {CNMI_CMD, "AT+CNMI"},
    {QSCLK_CMD, "AT+QSCLK"},
    {CFUN_CMD, "AT+CFUN"},
	{ATA_CMD, "ATA"},
	{CBC_CMD, "AT+CBC"},
	{CIPMUX_CMD, "AT+CIPMUX"},
	{CIPSTART_CMD, "AT+CIPSTART"},
	{CIPCLOSE_CMD, "AT+CIPCLOSE"},
	{CIPSEND_CMD, "AT+CIPSEND"},
	{CIPQSEND_CMD, "AT+CIPQSEND"},
	{CIPRXGET_CMD, "AT+CIPRXGET"},
	{CIPACK_CMD, "AT+CIPACK"},
	{CSCLK_CMD, "AT+CSCLK"},
	{CIPSTATUS_CMD, "AT+CIPSTATUS"},
	{CIPSHUT_CMD, "AT+CIPSHUT"},
	{CFGRI_CMD, "AT+CFGRI"},
	{WIFISCAN_CMD, "AT+WIFISCAN"},
	{CFG_CMD, "AT+CFG"},
    {CIICR_CMD, "AT+CIICR"},
    {CIFSR_CMD, "AT+CIFSR"},
    {CSTT_CMD, "AT+CSTT"},
    {CPNETAPN_CMD, "AT+CPNETAPN"},
    {CGAUTH_CMD, "AT+CGAUTH"},
    {CGATT_CMD, "AT+CGATT"},
};

/**************************************************
@bref		����ָ��������У�����˳�����
@param
@return
@note
**************************************************/

uint8_t createNode(char *data, uint16_t datalen, uint8_t currentcmd)
{
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    //�������ͷδ�������򴴽�����ͷ��
    WAKEMODULE;
    if (currentcmd == WIFISCAN_CMD)
    {
		wakeUpByInt(1, 28);
    }
    else
    {
		wakeUpByInt(1, 8);
    }
    if (headNode == NULL)
    {
        headNode = malloc(sizeof(cmdNode_s));
        if (headNode != NULL)
        {
            headNode->currentcmd = currentcmd;
            headNode->data = NULL;
            headNode->data = malloc(datalen);
            if (headNode->data != NULL)
            {
                memcpy(headNode->data, data, datalen);
                headNode->datalen = datalen;
                headNode->nextnode = NULL;
                return 1;
            }
            else
            {
                free(headNode);
                headNode = NULL;
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
    currentnode = headNode;
    do
    {
        nextnode = currentnode->nextnode;
        if (nextnode == NULL)
        {
            nextnode = malloc(sizeof(cmdNode_s));
            if (nextnode != NULL)
            {

                nextnode->currentcmd = currentcmd;
                nextnode->data = NULL;
                nextnode->data = malloc(datalen);
                if (nextnode->data != NULL)
                {
                    memcpy(nextnode->data, data, datalen);
                    nextnode->datalen = datalen;
                    nextnode->nextnode = NULL;
                    currentnode->nextnode = nextnode;
                    nextnode = nextnode->nextnode;
                }
                else
                {
                    free(nextnode);
                    nextnode = NULL;
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        currentnode = nextnode;
    }
    while (nextnode != NULL);

    return 1;
}

/**************************************************
@bref		���ݶ������
@param
@return
@note
**************************************************/

void outputNode(void)
{
    static uint8_t lockFlag = 0;
    static uint8_t sleepTick = 0;
    static uint8_t tickRange = 50;
    cmdNode_s *nextnode;
    cmdNode_s *currentnode;
    if (lockFlag)
    {
        if (sysinfo.lockTick++ >= tickRange)
        {
            lockFlag = 0;
            sysinfo.lockTick = 0;
            LogMessage(DEBUG_ALL, "outputNode==>Unlock");
        }
        return ;
    }
    if (headNode == NULL)
    {
        if (sleepTick > 0)
        {
            sleepTick--;
        }
        else
        {
        	/* 
        		ģ��ػ�״̬ʱDTR��Ҫ�õ�;
        		����øߵĻ�ģ���һֱ��1.5V���ҵĵ�,�����ᵼ��ģ����ܻ�ز��˻�,Ҳ�����˻�,��Ҫ�����ϵ����;
        		����ģ��ػ���ʱ����ò�Ҫ��ֹ���﷢��ATָ��,��Ȼ��һֱռ�Ŷѿռ�
        	*/
        	if (moduleState.powerState == 0)	//�ػ��������ڿ��ػ� DTR������
        	{
            	WAKEMODULE;
            }
            else 
            {
				SLEEPMODULE;
            }
        }
        return ;
    }
    sleepTick = 2;
    currentnode = headNode;
    if (currentnode != NULL)
    {
        nextnode = currentnode->nextnode;
        moduleState.cmd = currentnode->currentcmd;

        //���ݷ���
        portUartSend(&usart3_ctl, (uint8_t *)currentnode->data, currentnode->datalen);
        if (currentnode->data[0] != 0X78 && currentnode->data[0] != 0x79 && currentnode->data[0] != 0x7E)
        {
            LogMessageWL(DEBUG_ALL, currentnode->data, currentnode->datalen);
        }
        free(currentnode->data);
        free(currentnode);
        if (currentnode->currentcmd == CIPCLOSE_CMD || currentnode->currentcmd == CMGS_CMD || 
        		currentnode->currentcmd == WIFISCAN_CMD)
        {
            lockFlag = 1;
            if (currentnode->currentcmd == CIPCLOSE_CMD)
            {
                tickRange = 20;
            }
            else if (currentnode->currentcmd == WIFISCAN_CMD)
            {
				tickRange = 20;
            }
            else
            {
                tickRange = 10;
            }
            LogMessage(DEBUG_ALL, "outputNode==>Lock");
        }
    }
    headNode = nextnode;

}

/**************************************************
@bref		ģ��ָ���
@param
@return
@note
**************************************************/

uint8_t  sendModuleCmd(uint8_t cmd, char *param)
{
    uint8_t i;
    int16_t cmdtype = -1;
    char sendData[256];
    for (i = 0; i < sizeof(cmdtable) / sizeof(cmdtable[0]); i++)
    {
        if (cmd == cmdtable[i].cmd_type)
        {
            cmdtype = i;
            break;
        }
    }
    if (cmdtype < 0)
    {
        snprintf(sendData, 255, "sendModuleCmd==>No cmd");
        LogMessage(DEBUG_ALL, sendData);
        return 0;
    }
    if (param != NULL && strlen(param) <= 240)
    {
        if (param[0] == '?')
        {
            snprintf(sendData, 255, "%s?\r\n", cmdtable[cmdtype].cmd);

        }
        else
        {
            snprintf(sendData, 255, "%s=%s\r\n", cmdtable[cmdtype].cmd, param);
        }
    }
    else if (param == NULL)
    {
        snprintf(sendData, 255, "%s\r\n", cmdtable[cmdtype].cmd);
    }
    else
    {
        return 0;
    }
    createNode(sendData, strlen(sendData), cmd);
    return 1;
}

void sendModuleDirect(char *param)
{
    createNode(param, strlen(param), 0);
}

/**************************************************
@bref		��ʼ��ģ�����ʹ�ýṹ��
@param
@return
@note
**************************************************/

static void moduleInit(void)
{
    memset(&moduleState, 0, sizeof(moduleState_s));
}

/**************************************************
@bref		ģ�鿪��
@param
@return
@note
**************************************************/

void modulePowerOn(void)
{
	moduleReqSet(MODULE_REQUEST_OPEN);
}


/**************************************************
@bref		ģ��ػ�
@param
@return
@note
**************************************************/

void modulePowerOff(void)
{
	moduleReqSet(MODULE_REQUEST_CLOSE);
}

/**************************************************
@bref		ģ�鸴λ
@param
@return
@note
**************************************************/

void moduleReset(void)
{
	//moduleState.powerState == 0��ʾ�豸���ڹػ����ߴ��ڿ��ػ�״̬
	//����Ѿ����ڿ��ػ�״̬����û��Ҫ�����������
	if (moduleState.powerState != 0)
		moduleReqSet(MODULE_REQUEST_RESET);
}

/**************************************************
@bref		ģ��ϵ�����
@param
@return
@note
**************************************************/
void moduleShutdownStartup(void)
{
	moduleReqSet(MODULE_REQUEST_SHUTDOWN_OPEN);
}

/**************************************************
@bref		ģ�鿪��״̬��ȡ
@param
@return    0���ػ����
		   1���������
		   2�����ڽ��п��ػ�
@note
**************************************************/

uint8_t getModulePwrState(void)
{
	/* �����ǹر���ģ���Դ״̬���ǹر�״̬ */
	if (sysinfo.moduleReq == 0 && sysinfo.moduleFsm == MODULE_FSM_CLOSE_DONE)
		return 0;
	if (sysinfo.moduleReq == 0 && sysinfo.moduleFsm == MODULE_FSM_OPEN_DONE)
	    return 1;
	return 2;
}

/**************************************************
@bref		ģ���Դ��������
@param
@return
@note
**************************************************/

void moduleReqSet(uint8_t req)
{
	sysinfo.moduleReq = req;
	LogPrintf(DEBUG_ALL, "moduleReqSet==>%d", sysinfo.moduleReq);
}

/**************************************************
@bref		ģ���Դ�������
@param
@return
@note
**************************************************/
void moduleReqClear(void)
{
	if (sysinfo.moduleReq != 0)
	{
		sysinfo.moduleReq = 0;
		LogPrintf(DEBUG_ALL, "moduleReqClear");
	}
}

/**************************************************
@bref		ģ���Դ�����ѯ
@param
@return
@note
**************************************************/

uint8_t moduleReqGet(void)
{
	return sysinfo.moduleReq;
}

/**************************************************
@bref		ģ�鿪��Ӳ����ʼ��
@param
@return
@note
**************************************************/

static void moduleHwInit(void)
{
	LogMessage(DEBUG_ALL, "moduleHwInit");
    moduleInit();
    sysinfo.moduleRstFlag = 1;
    portUartCfg(APPUSART3, 1, 115200, moduleRecvParser);
    moduleSupplyOn();
    PWRKEY_HIGH;
    RSTKEY_HIGH;
    moduleState.gpsFileHandle = 1;
    moduleCtrl.scanMode = 0;
    socketDelAll();
}

/**************************************************
@bref		ģ���ϵ����
@param
@return
@note
**************************************************/

void moduleSupplyOn(void)
{
    POWER_ON;
	sysinfo.moduleSupplyStatus = 1;
	LogPrintf(DEBUG_ALL, "moduleSupplyOn");
}

/**************************************************
@bref		ģ���µ����
@param
@return
@note
**************************************************/

void moduleSupplyOff(void)
{
	POWER_OFF;
	sysinfo.moduleSupplyStatus = 0;
	LogPrintf(DEBUG_ALL, "moduleSupplyOff");
}

/**************************************************
@bref		ģ�����������µ�
@param
@return
@note
**************************************************/

static void moduleAllPinPullDown(void)
{
	portUartCfg(APPUSART3, 0, 115200, NULL);
	RSTKEY_HIGH;
	PWRKEY_HIGH;
	WAKEMODULE;
	moduleInit();
	sysinfo.moduleRstFlag = 1;
	socketDelAll();	
}


/**************************************************
@bref		ģ���Դ״̬���л�
@param
@return
@note
**************************************************/

static void moduleFsmChange(uint8_t fsm)
{
	sysinfo.moduleFsm = fsm;
	sysinfo.moduleFsmTick = 0;
}

/**************************************************
@bref		ģ���Դ״̬��
@param
@return
@note
sysinfo.moduleReqΪ�������ػ�����λ���ϵ縴λ������
��4�������Ϊ���⣬״̬�����յ������ʼִ�в�������������
�ڴ�����һ������֮ǰ���ڶ�������ʱ���ᱣ��˴�����ֱ����ɵ�ǰ����
��ǰ���������֮�󣬻��ж���һ�������Ƿ������ѡ���Ƿ�ִ��
���統��ɹػ����������һ����λ���󣬲�����������˴�����

**************************************************/
void moduleRequestTask(void)
{
	
	switch (sysinfo.moduleFsm)
	{
	case MODULE_FSM_CLOSE_DONE:
		if (sysinfo.moduleReq == MODULE_REQUEST_OPEN)
		{
			moduleHwInit();
			moduleFsmChange(MODULE_FSM_OPEN_ING1);
		}
		else
		{
			//��ģ�黽��tick��� ��mcu�����������
		    sysinfo.ringWakeUpTick = 0;
		    sysinfo.cmdTick = 0;
		    sysinfo.irqTick = 0;
		}
		moduleReqClear();
		break;
	case MODULE_FSM_OPEN_ING1:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			PWRKEY_LOW;
			moduleFsmChange(MODULE_FSM_OPEN_ING2);
		}
		break;
	case MODULE_FSM_OPEN_ING2:
		if (++sysinfo.moduleFsmTick > 11)	//1100ms
		{
			PWRKEY_HIGH;
			moduleState.powerState = 1;
			LogMessage(DEBUG_ALL, "modulePowerOnDone");
			moduleFsmChange(MODULE_FSM_OPEN_DONE);
		}
		break;
	case MODULE_FSM_OPEN_ING3:
		moduleFsmChange(MODULE_FSM_OPEN_DONE);
		break;
	case MODULE_FSM_OPEN_DONE:
		if (sysinfo.moduleReq == MODULE_REQUEST_CLOSE)
		{
			LogMessage(DEBUG_ALL, "modulePowerOff");
			moduleAllPinPullDown();
		    moduleFsmChange(MODULE_FSM_CLOSE_ING1);
		}
		else if (sysinfo.moduleReq == MODULE_REQUEST_RESET)
		{
			LogMessage(DEBUG_ALL, "modulePowerReset");
			moduleAllPinPullDown();
			moduleFsmChange(MODULE_FSM_RESET_ING1);
		}
		else if (sysinfo.moduleReq == MODULE_REQUEST_SHUTDOWN_OPEN)
		{
			LogMessage(DEBUG_ALL, "module shut down");
			moduleAllPinPullDown();
			moduleFsmChange(MODULE_FSM_SHUTDOWN_ING);
		}
		moduleReqClear();
		break;
	case MODULE_FSM_CLOSE_ING1:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			LogMessage(DEBUG_ALL, "modulePowerOffPress");
		   	PWRKEY_LOW;
			moduleFsmChange(MODULE_FSM_CLOSE_ING2);
		}
		break;
	case MODULE_FSM_CLOSE_ING2:
		if (++sysinfo.moduleFsmTick > 30)	//2000ms
		{
			PWRKEY_HIGH;
			LogMessage(DEBUG_ALL, "modulePowerOffDone");
			moduleFsmChange(MODULE_FSM_CLOSE_DONE);
		}
		break;
	case MODULE_FSM_RESET_ING1:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			RSTKEY_LOW;
			moduleFsmChange(MODULE_FSM_RESET_ING2);
		}
		break;
	case MODULE_FSM_RESET_ING2:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			LogMessage(DEBUG_ALL, "modulePowerResetDone");
			RSTKEY_HIGH;
			portUartCfg(APPUSART3, 1, 115200, moduleRecvParser);
			moduleState.powerState = 1;
			moduleFsmChange(MODULE_FSM_OPEN_DONE);
		}
		break;
	case MODULE_FSM_SHUTDOWN_ING:
		if (++sysinfo.moduleFsmTick > 5)	//500ms
		{
			LogMessage(DEBUG_ALL, "module shut down done");
			PWRKEY_LOW;
			moduleFsmChange(MODULE_FSM_SHUTDOWN_WAIT);
		}
		break;
	case MODULE_FSM_SHUTDOWN_WAIT:
		if (++sysinfo.moduleFsmTick > 32)
		{
			moduleFsmChange(MODULE_FSM_SHUTDOWN_UP);
		}
		if ((sysinfo.moduleFsmTick % 10) == 0)
			LogPrintf(DEBUG_ALL, "module shut down wait %ds...", sysinfo.moduleFsmTick / 10);
		break;
	case MODULE_FSM_SHUTDOWN_UP:
		LogMessage(DEBUG_ALL, "module shut down up");
		PWRKEY_HIGH;
		portUartCfg(APPUSART3, 1, 115200, moduleRecvParser);
		moduleState.powerState = 1;
		moduleFsmChange(MODULE_FSM_OPEN_ING1);
		break;
	default:
		moduleFsmChange(MODULE_FSM_CLOSE_DONE);
		break;
	}
}


/**************************************************
@bref		�л�����״̬��
@param
@return
@note
**************************************************/
static void changeProcess(uint8_t fsm)
{
    moduleState.fsmState = fsm;
    moduleState.fsmtick = 0;
    if (moduleState.fsmState != NORMAL_STATUS)
    {
        ledStatusUpdate(SYSTEM_LED_NETOK, 0);
    }
}

/**************************************************
@bref		����socket
@param
@return
@note
**************************************************/

void openSocket(uint8_t link, char *server, uint16_t port)
{
    char param[100] = { 0 };
    sprintf(param, "%d,\"TCP\",\"%s\",%d", link, server, port);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPSTART_CMD, param);
}

/**************************************************
@bref		�ر�socket
@param
@return
@note
**************************************************/

void closeSocket(uint8_t link)
{
    char param[10] = { 0 };
    sprintf(param, "%d", link);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPCLOSE_CMD, param);
}

/**************************************************
@bref       cgdcong����
@param
@return
@note
**************************************************/

static void netSetCgdcong(char *apn)
{
    char param[100] = { 0 };
    sprintf(param, "1,\"IP\",\"%s\"", apn);
    sendModuleCmd(CGDCONT_CMD, param);
}

/**************************************************
@bref       apn����
@param
@return
@note
**************************************************/

void netSetApn(char *apn, char *apnname, char *apnpassword, uint8_t apnauthport)
{
    char param[100] = { 0 };
    sprintf(param, "2,\"%s\",\"%s\",\"%s\",%d", apn, apnname, apnpassword, apnauthport);
    sendModuleCmd(CPNETAPN_CMD, param);
}

/**************************************************
@bref       cstt����
@param
@return
@note
**************************************************/

static void netSetCstt(char *apn, char *apnname, char *apnpassword)
{
    char param[100] = { 0 };
    sprintf(param, "\"%s\",\"%s\",\"%s\"", apn, apnname, apnpassword);
    sendModuleCmd(CSTT_CMD, param);
}

/**************************************************
@bref		ģ��������ģʽ
@param
@return
@note
**************************************************/

static void moduleEnterFly(void)
{
    sendModuleCmd(CFUN_CMD, "0");
}

/**************************************************
@bref		ģ���������ģʽ
@param
@return
@note
**************************************************/

static void moduleExitFly(void)
{
    sendModuleCmd(CFUN_CMD, "1");
}

/**************************************************
@bref		����socket��ȡ����ָ��
@param
@return
@note
**************************************************/

static void qirdCmdSend(uint8_t link, uint8_t index)
{
    char param[10];
    sprintf(param, "2,%d,500", link);
    moduleState.curQirdId = link;
    sendModuleCmd(CIPRXGET_CMD, param);
}

/**************************************************
@bref		��ȡ����
@param
@return
@note
**************************************************/

static void queryRecvBuffer(void)
{
    char param[10];
    if (moduleState.normalLinkQird)
    {
        qirdCmdSend(NORMAL_LINK, moduleState.normalLinkQird);

    }
    else if (moduleState.agpsLinkQird)
    {
        qirdCmdSend(AGPS_LINK, moduleState.agpsLinkQird);

    }
    else if (moduleState.bleLinkQird)
    {
        qirdCmdSend(BLE_LINK, moduleState.bleLinkQird);

    }
    else if (moduleState.jt808LinkQird)
    {
        qirdCmdSend(JT808_LINK, moduleState.jt808LinkQird);

    }
    else if (moduleState.hideLinkQird)
    {
        qirdCmdSend(HIDDEN_LINK, moduleState.hideLinkQird);

    }
}

/**************************************************
@bref		����׼������
@param
@return
@note
**************************************************/

void netConnectTask(void)
{
    if (moduleState.powerState == 0)
    {
        return;
    }

    moduleState.powerOnTick++;
    switch (moduleState.fsmState)
    {
        case AT_STATUS:
            if (moduleState.atResponOK)
            {
                moduleCtrl.atCount = 0;
                moduleState.atResponOK = 0;
                moduleState.cpinResponOk = 0;
                changeProcess(CPIN_STATUS);
            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    moduleState.powerOnTick = 0;
                    sendModuleCmd(AT_CMD, NULL);
                }
                if (moduleState.fsmtick >= 15)
                {
                    moduleCtrl.atCount++;
                    if (moduleCtrl.atCount >= 2)
                    {
                        moduleCtrl.atCount = 0;
                        moduleShutdownStartup();
                    }
                    else
                    {
                        moduleReset();
                    }
                }
                break;
            }
        case CPIN_STATUS:
            if (moduleState.cpinResponOk)
            {
                moduleState.cpinResponOk = 0;
                moduleState.csqOk = 0;
                netSetCgdcong((char *)sysparam.apn);
                //�޸�APN��Ȩ��ʽ�����ָ��˻Ῠ��
                //netSetApn((char *)sysparam.apn, (char *)sysparam.apnuser, (char *)sysparam.apnpassword, sysparam.apnAuthport);
                sendModuleCmd(CGSN_CMD, "1");
                changeProcess(CSQ_STATUS);
            }
            else
            {
                if (moduleState.fsmtick % 2 == 0)
                {
                    sendModuleCmd(CPIN_CMD, "?");
                }
                if (moduleState.fsmtick >= 30)
                {
                    moduleReset();
                }
                break;
            }
        case CSQ_STATUS:
            if (moduleState.csqOk)
            {
                moduleState.csqOk = 0;
                moduleState.cgregOK = 0;
                moduleState.ceregOK = 0;
                moduleCtrl.csqCount = 0;
                sendModuleCmd(CGREG_CMD, "2");
                sendModuleCmd(CPMS_CMD, "\"ME\",\"ME\",\"ME\"");	/*�޸Ķ��Ŵ洢λ��*/
	        	sendModuleCmd(CNMI_CMD, "2,2");						/*�ڶ���������ʾ������ME��, �������ϱ�*/
	        	sendModuleCmd(CMGF_CMD, "1");						/*TEXTģʽ*/
                changeProcess(CGREG_STATUS);
                netResetCsqSearch();
            }
            else
            {
                sendModuleCmd(CSQ_CMD, NULL);
                if (moduleCtrl.csqTime == 0)
                {
                    moduleCtrl.csqTime = 90;
                }
                if (moduleState.fsmtick >= 90)
                {
                    moduleCtrl.csqCount++;
                    if (moduleCtrl.csqCount >= 3)
                    {
                        moduleCtrl.csqCount = 0;
                        //3��������������ʱ�����û��gps������ػ�
                        if (sysinfo.gpsRequest != 0)
                        {
                            moduleReset();
                        }
                        else
                        {
                            modeTryToStop();
                        }
                    }
                    else
                    {
                        moduleEnterFly();
                        startTimer(80, moduleExitFly, 0);
                    }
                    changeProcess(AT_STATUS);
                }
                break;
            }
        case CGREG_STATUS:
            if (moduleState.ceregOK || moduleState.cgregOK)
            {
                moduleCtrl.cgregCount = 0;
                moduleState.ceregOK = 0;
                moduleState.cgregOK = 0;
                sendModuleCmd(CIPSHUT_CMD, NULL);
                changeProcess(CONFIG_STATUS);
            }
            else
            {
                sendModuleCmd(CGREG_CMD, "?");
                if (moduleState.fsmtick >= 90)
                {
                    moduleCtrl.cgregCount++;
                    if (moduleCtrl.cgregCount >= 2)
                    {
                        moduleCtrl.cgregCount = 0;
                        //2��ע�᲻�ϻ�վʱ�����û��gps������ػ�
                        if (sysinfo.gpsRequest != 0)
                        {
                            moduleReset();
                        }
                        else
                        {
                            modeTryToStop();
                        }

                        LogMessage(DEBUG_ALL, "Register timeout,try to skip");
                    }
                    else
                    {
                        changeProcess(AT_STATUS);
                    }
                }
                break;
            }
        case CONFIG_STATUS:
            sendModuleCmd(CFGRI_CMD, "1,200,200,3");
            sendModuleCmd(CIPMUX_CMD, "1");
            sendModuleCmd(CIPQSEND_CMD, "1");
            sendModuleCmd(CIPRXGET_CMD, "5");
            sendModuleCmd(CFG_CMD, "\"urcdelay\",100");
            sendModuleCmd(CIMI_CMD, NULL);
            sendModuleCmd(CGSN_CMD, "1");
            sendModuleCmd(ICCID_CMD, NULL);
            queryBatVoltage();
            //netSetCstt((char *)sysparam.apn, (char *)sysparam.apnuser, (char *)sysparam.apnpassword);
			if (sysinfo.delMsgReq)
			{
				deleteAllMessage();
				delMsgReqClear();
			}
            if (sysparam.MODE == MODE4 && sysinfo.gpsRequest == 0)
            {
				moduleSleepCtl(1);
				changeProcess(OFFLINE_STATUS);
            }
            else
            {
				changeProcess(QIACT_STATUS);
            }
            break;
        case QIACT_STATUS:
            if (moduleState.qipactOk)
            {
                moduleState.qipactOk = 0;
                moduleCtrl.qipactCount = 0;
                changeProcess(NORMAL_STATUS);
            }
            else
            {
               	sendModuleCmd(CGATT_CMD, "?");
                if (moduleState.fsmtick >= 45)
                {
                    LogMessage(DEBUG_ALL, "try QIPACT again");
                    moduleState.qipactSet = 0;
                    moduleState.fsmtick = 0;
                    moduleCtrl.qipactCount++;
                    if (moduleCtrl.qipactCount >= 3)
                    {
                        moduleCtrl.qipactCount = 0;
                        moduleReset();
                    }
                    else
                    {
                        changeProcess(CPIN_STATUS);
                        sendModuleCmd(CIPSHUT_CMD, NULL);
                    }
                }
                break;
            }
        case NORMAL_STATUS:
            socketSchedule();
            queryRecvBuffer();
            break;
        case OFFLINE_STATUS:
			if (sysparam.MODE != MODE4 || sysinfo.gpsRequest != 0)
			{
				changeProcess(QIACT_STATUS);
			}
        	break;
        default:
            changeProcess(AT_STATUS);
            break;
    }
    moduleState.fsmtick++;
}


/**************************************************
@bref		AT+CSQ	ָ�����
@param
@return
@note
**************************************************/

static void csqParser(uint8_t *buf, uint16_t len)
{
    int index, indexa, datalen;
    uint8_t *rebuf;
    uint16_t  relen;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CSQ:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, ',');
        if (indexa > 6)
        {
            datalen = indexa - 6;
            if (datalen > 5)
                return;
            memset(restore, 0, 5);
            strncpy(restore, (char *)rebuf + 6, datalen);
            moduleState.rssi = atoi(restore);
            if (moduleState.rssi >= 6 && moduleState.rssi <= 31)
                moduleState.csqOk = 1;
        }
    }
}

/**************************************************
@bref		AT+CREG	ָ�����
@param
@return
@note
**************************************************/

static void cgregParser(uint8_t *buf, uint16_t len)
{
    int index, datalen;
    uint8_t *rebuf;
    uint16_t  relen, i;
    char restore[50];
    uint8_t cnt;
    uint8_t type = 0;
    index = my_getstrindex((char *)buf, "+CGREG:", len);
    if (index < 0)
    {
        type = 1;
        index = my_getstrindex((char *)buf, "+CEREG:", len);
    }
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        datalen = 0;
        cnt = 0;
        restore[0] = 0;
        for (i = 0; i < relen; i++)
        {
            if (rebuf[i] == ',' || rebuf[i] == '\r' || rebuf[i] == '\n')
            {
                if (restore[0] != 0)
                {
                    restore[datalen] = 0;
                    cnt++;
                    datalen = 0;
                    switch (cnt)
                    {
                        case 2:
                            if (restore[0] == '1' || restore[0] == '5')
                            {
                                if (type)
                                {
                                    moduleState.ceregOK = 1;
                                }
                                else
                                {
                                    moduleState.cgregOK = 1;
                                }
                            }
                            else
                            {
                                return ;
                            }
                            break;
                        case 3:
                            moduleState.lac = strtoul(restore + 1, NULL, 16);
                            LogPrintf(DEBUG_ALL, "LAC=%s,0x%X", restore, moduleState.lac);
                            break;
                        case 4:
                            moduleState.cid = strtoul(restore + 1, NULL, 16);
                            LogPrintf(DEBUG_ALL, "CID=%s,0x%X", restore, moduleState.cid);
                            break;
                    }
                    restore[0] = 0;
                }
            }
            else
            {
                restore[datalen] = rebuf[i];
                datalen++;
                if (datalen >= 50)
                {
                    return ;
                }
            }
        }
        if (type == 0)
        {
			sendModuleCmd(CGREG_CMD, "0");
		}
    }
}

/**************************************************
@bref		AT+CIMI	ָ�����
@param
@return
@note
	460064814034016
����"460"���й���MCC��"06"���й���ͨ��MNC����������10λ����"4814034016"�Ǹ��ƶ��û���Ψһ��ʶ�������ƶ������������û���
**************************************************/

static void cimiParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
    index = getCharIndex(rebuf, relen, '\n');
    if (index < 0)
    {
        return;
    }
    rebuf = rebuf + index + 1;
    relen = relen - index - 1;
    index = getCharIndex(rebuf, relen, '\r');
    if (index == 15)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMSI[i] = rebuf[i];
        }
        moduleState.IMSI[index] = 0;
        moduleState.mcc = (moduleState.IMSI[0] - '0') * 100 + (moduleState.IMSI[1] - '0') * 10 + moduleState.IMSI[2] - '0';
        moduleState.mnc = (moduleState.IMSI[3] - '0') * 10 + moduleState.IMSI[4] - '0';
        LogPrintf(DEBUG_ALL, "IMSI:%s,MCC=%d,MNC=%02d", moduleState.IMSI, moduleState.mcc, moduleState.mnc);
    }
}


/**************************************************
@bref		AT+ICCID	ָ�����
@param
@return
@note
**************************************************/

static void iccidParser(uint8_t *buf, uint16_t len)
{
    int16_t index, indexa;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t snlen, i;
    char debug[70];
    index = my_getstrindex((char *)buf, "+ICCID:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        relen = len - index;
        indexa = getCharIndex(rebuf, relen, '\r');
        if (indexa > 8)
        {
            snlen = indexa - 8;
            if (snlen == 20)
            {
                for (i = 0; i < snlen; i++)
                {
                    moduleState.ICCID[i] = rebuf[i + 8];
                }
                moduleState.ICCID[snlen] = 0;
                sprintf(debug, "ICCID:%s", moduleState.ICCID);
                LogMessage(DEBUG_ALL, debug);
            }
        }
    }

}


/**************************************************
@bref		���Ž���
@param
@return
@note
**************************************************/

static void cmtiParser(uint8_t *buf, uint16_t len)
{
    uint8_t i;
    int16_t index;
    uint8_t *rebuf;
    char restore[5];
    index = my_getstrindex((char *)buf, "+CMTI:", len);
    if (index >= 0)
    {
        rebuf = buf + index;
        index = getCharIndex(rebuf, len, ',');
        if (index < 0)
            return;
        rebuf = rebuf + index + 1;
        index = getCharIndex(rebuf, len, '\r');
        if (index > 5 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message index=%d", atoi(restore));
        sendModuleCmd(CMGR_CMD, restore);
    }
}

/**************************************************
@bref		CMGR	ָ�����
@param
@return
@note
**************************************************/

static void cmgrParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf;
    uint8_t *numbuf;
    uint16_t  relen, i, renumlen;
    char restore[100];
    insParam_s insparam;
    //�ҵ��ض��ַ�����buf��λ��
    index = my_getstrindex((char *)buf, "+CMGR:", len);
    if (index >= 0)
    {
        //�õ��ض��ַ����Ŀ�ʼλ�ú�ʣ�೤��
        rebuf = buf + index;
        relen = len - index;
        //ʶ���ֻ�����
        index = getCharIndexWithNum(rebuf, relen, '"', 3);
        if (index < 0)
            return;
        numbuf = rebuf + index + 1;
        renumlen = relen - index - 1;
        index = getCharIndex(numbuf, renumlen, '"');
        if (index > 100 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = numbuf[i];
        }
        restore[index] = 0;

        if (index > sizeof(moduleState.messagePhone))
            return ;
        strcpy((char *)moduleState.messagePhone, restore);
        LogPrintf(DEBUG_ALL, "Tel:%s", moduleState.messagePhone);
        //�õ���һ��\n��λ��
        index = getCharIndex(rebuf, len, '\n');
        if (index < 0)
            return;
        //ƫ�Ƶ����ݴ�
        rebuf = rebuf + index + 1;
        //�õ������ݴ���ʼ�ĵ�һ��\n������index�������ݳ���
        index = getCharIndex(rebuf, len, '\n');
        if (index > 100 || index < 0)
            return ;
        for (i = 0; i < index; i++)
        {
            restore[i] = rebuf[i];
        }
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Message:%s", restore);
        insparam.telNum = moduleState.messagePhone;
        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
    }
}

/**************************************************
@bref       CIPACK ����
@param
@return
@note

+CIPACK: 42,42,0

Air780Eһ�㷢��AT+CIPACK=<link>ģ��������һ���Է��أ�
\r\n
+CIPACK: 42,42,0
\r\n
��Air780ET����AT+CIPACK=<link>���ܻ��ȷ���
\r\n
����ż��ٺ������΢���ٷ���
+CIPACK: 42,42,0
\r\n
�����ͻᵼ�� ���cipackParser��������case CIPACK_CMD:����Ļ�
ʵ�ʻ᲻ִ��cipackParser����Ϊ�ȷ��ص�\r\n�ᵼ������ִ����һ��
cipackParser��Ȼ��ִ��moduleState.cmd=0;���µ�+CIPACK: 42,42,0
\r\n���ٵ�ʱ��Ͳ�ִ�е�cipackParser������

**************************************************/
static uint8_t ack_err_cnt = 0;
void cipackParser(uint8_t *buf, uint16_t len)
{
    int index;
    ITEM item;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "+CIPACK:", relen);
    if (index < 0)
    {
        return;
    }

    rebuf += 8;
    relen -= 8;

    if (relen < 0)
    {
        return;
    }
    ack_err_cnt = 0;
    stringToItem(&item, rebuf, relen);
    moduleState.tcpTotal = atoi(item.item_data[0]);
    moduleState.tcpAck = atoi(item.item_data[1]);
    moduleState.tcpNack = atoi(item.item_data[2]);
    LogPrintf(DEBUG_ALL, "Total:%d,Ack:%d,NAck:%d", moduleState.tcpTotal, moduleState.tcpAck, moduleState.tcpNack);

}

/**************************************************
@bref       CIPACK error ����
@param
@return
@note
��ʱ������·�쳣��û�м�⵽�����ܻ�һֱ����AT+CIPACK=0
��ʱ����������·�Ѿ����ˣ����Ի�һֱ����error
��ʱ��⵽5��error�����²�ѯ��������
**************************************************/

static void cipackErrorParser(uint8_t *buf, uint16_t len)
{
	int index;
    uint8_t *rebuf;
    int16_t relen;

    index = my_getstrindex(rebuf, "ERROR", relen);
    if (index >= 0)
    {
        ack_err_cnt++;
        LogPrintf(DEBUG_ALL, "ack error cnt:%d", ack_err_cnt);
        if (ack_err_cnt >= 5)
        {
            changeProcess(CPIN_STATUS);
            ack_err_cnt = 0;
        }
    }

}


/**************************************************
@bref       QWIFISCAN   ָ�����
@param
@return
@note
+WIFISCAN: "50:0f:f5:51:1a:eb",-31,3
+WIFISCAN: "dc:9f:db:1c:1d:76",-50,11
+WIFISCAN: "ec:41:18:0c:82:09",-63,4
+WIFISCAN: "40:22:30:1a:f8:01",-67,1
+WIFISCAN: "08:6b:d1:0b:50:60",-68,11
+WIFISCAN: "14:09:b4:95:94:6d",-72,6
+WIFISCAN: "f8:8c:21:a2:c6:e9",-74,11
+WIFISCAN: "70:3a:73:05:79:1c",-90,13
+WIFISCAN: "2c:9c:6e:28:f5:0e",-91,1
+WIFISCAN: "7c:a7:b0:61:db:14",-92,11

OK
**************************************************/

static void wifiscanParser(uint8_t *buf, uint16_t len)
{
    int index;
    uint8_t *rebuf, i;
    int16_t relen;
    char restore[20];
    uint8_t numb;
    WIFIINFO wifiList = { 0 };
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+WIFISCAN:", relen);
    wifiList.apcount = 0;
    while (index >= 0)
    {
        rebuf += index + 12;
        relen -= index + 12;

        index = getCharIndex(rebuf, relen, '"');
        if (index == 17 && wifiList.apcount < WIFIINFOMAX)
        {
            memcpy(restore, rebuf, index);
            restore[17] = 0;
            LogPrintf(DEBUG_ALL, "WIFI:[%s]", restore);
            wifiList.ap[wifiList.apcount].signal = 0;
            for (i = 0; i < 6; i++)
            {
                changeHexStringToByteArray(wifiList.ap[wifiList.apcount].ssid + i, restore + (3 * i), 1);
            }
            wifiList.apcount++;
        }
        index = getCharIndex(rebuf, relen, '\n');
        rebuf += index;
        relen -= index;
        index = my_getstrindex((char *)rebuf, "+WIFISCAN:", relen);
    }

	if (wifiList.apcount != 0)
    {
    	if (wifiList.apcount < 3)
    	{
			lbsRequestSet(DEV_EXTEND_OF_MY);
    	}
    	else 
    	{
	        if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_MY)
	        {
	            protocolSend(NORMAL_LINK, PROTOCOL_F3, &wifiList);
	        }
	        if (sysinfo.wifiExtendEvt & DEV_EXTEND_OF_BLE)
	        {
	            protocolSend(BLE_LINK, PROTOCOL_F3, &wifiList);
	        }
	        lbsRequestClear();
		}
        sysinfo.wifiExtendEvt = 0;
        wifiRspSuccess();
    }
}

/*
AT+CGSN=1
+CGSN: "861959062037101"

AT+CGSN
861959062037101

*/
static void cgsnParser(uint8_t *buf, uint16_t len)
{
    int16_t index;
    uint8_t *rebuf;
    uint16_t  relen;
    uint8_t i;
    rebuf = buf;
    relen = len;
    
    index = my_getstrindex((char *)rebuf, "+CGSN:", relen);
    rebuf += index + 8;
    relen -= index + 8;
	
    index = getCharIndex(rebuf, relen, '"');
    if (index >= 0 && index < 20)
    {
        for (i = 0; i < index; i++)
        {
            moduleState.IMEI[i] = rebuf[i];
        }
        moduleState.IMEI[index] = 0;
        LogPrintf(DEBUG_ALL, "module IMEI [%s]", moduleState.IMEI);
        if (tmos_memcmp(moduleState.IMEI, dynamicParam.SN, 15) == FALSE)
        {
            tmos_memset(dynamicParam.SN, 0, sizeof(dynamicParam.SN));
            strncpy(dynamicParam.SN, moduleState.IMEI, 15);
            jt808CreateSn(dynamicParam.jt808sn, dynamicParam.SN + 3, 12);
            dynamicParam.jt808isRegister = 0;
            dynamicParam.jt808AuthLen = 0;
            dynamicParamSaveAll();
        }
    }
}


/**************************************************
@bref       +CGATT  ָ�����
@param
@return
@note
    +CGATT: 1

    OK
**************************************************/

static void cgattParser(uint8_t *buf, uint16_t len)
{
    uint8_t ret;
    int index;
    uint8_t *rebuf;
    int16_t relen;
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+CGATT:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    ret = rebuf[8] - '0';
    if (ret == 1)
    {
        moduleState.qipactOk = 1;
    }
    else
    {
        moduleState.qipactOk = 0;
    }
}


void cipstartRspParser(uint8_t *buf, uint16_t len)
{
    int index;
    int relen;
    uint8_t *rebuf;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "CONNECT", relen);
    if (index < 0)
    {
        return;
    }

    index = my_getstrindex(rebuf, "0, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(0, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "1, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(1, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "2, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(2, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "3, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(3, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "4, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(4, SOCKET_CONN_SUCCESS);
    }
    index = my_getstrindex(rebuf, "5, CONNECT OK", relen);
    if (index >= 0)
    {
        socketSetConnState(5, SOCKET_CONN_SUCCESS);
    }

    index = my_getstrindex(rebuf, "0, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(0, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "1, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(1, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "2, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(2, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "3, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(3, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "4, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(4, SOCKET_CONN_ERR);
    }
    index = my_getstrindex(rebuf, "5, CONNECT FAIL", relen);
    if (index >= 0)
    {
        socketSetConnState(5, SOCKET_CONN_ERR);
    }
}

void cipstartParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+CME ERROR:", len);
    if (index < 0)
    {
        return;
    }
    LogMessage(DEBUG_ALL, "socket open err");
    socketDelAll();
    changeProcess(CPIN_STATUS);
}

/**************************************************
@bref       ģ������ݽ��ս�����
@param
@return
@note
+CIPRXGET: 1,0
+CIPRXGET: 2,0,20,0
xx\0
2
xx\0
2


OK
**************************************************/

uint8_t ciprxgetParser(uint8_t *buf, uint16_t len)
{
    int index;
    char restore[513];
    uint8_t *rebuf, type, link, ret = 0;
    int16_t relen;
    uint16_t readLen, unreadLen, debugLen;
    ITEM item;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "+CIPRXGET:", relen);
    while (index >= 0)
    {
        rebuf += index + 11;
        relen -= index + 11;

        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0 && index < 20)
        {
            tmos_memcpy(restore, rebuf, index);
            restore[index] = 0;
            stringToItem(&item, restore, index);
            type = atoi(item.item_data[0]);
            link = atoi(item.item_data[1]);
            if (type == 1)
            {
                LogPrintf(DEBUG_ALL, "Socket[%d] recv data", link);
                switch (link)
                {
                    case NORMAL_LINK:
                        moduleState.normalLinkQird = 1;
                        break;
                    case BLE_LINK:
                        moduleState.bleLinkQird = 1;
                        break;
                    case JT808_LINK:
                        moduleState.jt808LinkQird = 1;
                        break;
                    case HIDDEN_LINK:
                        moduleState.hideLinkQird = 1;
                        break;
                    case AGPS_LINK:
                        moduleState.agpsLinkQird = 1;
                        break;
                }
            }
            else if (type == 2)
            {
                readLen = atoi(item.item_data[2]);
                unreadLen = atoi(item.item_data[3]);
                rebuf += index + 2;
                relen -= index + 2;
                if (relen >= readLen)
                {
                    if (readLen != 0)
                    {
                        debugLen = readLen > 256 ? 256 : readLen;
                        byteToHexString(rebuf, restore, debugLen);
//                        if (link == 4)
//                        {
//                          LogMessageWL(DEBUG_ALL, rebuf, relen);
//                        }
                        restore[debugLen * 2] = 0;
                        LogPrintf(DEBUG_ALL, "TCP RECV (%d)[%d]:%s", link, readLen, restore);
                        if (link == 4)
                        {
                            socketRecv(link, rebuf, readLen);
                        }
                        else 
                        {
                            socketRecv(link, rebuf, relen);//relen���readLen
                        }
                    }
                }
                else
                {
                    ret = 1;
                    break;
                }

                if (unreadLen == 0)
                {
                    switch (link)
                    {
                        case NORMAL_LINK:
                            moduleState.normalLinkQird = 0;
                            break;
                        case BLE_LINK:
                            moduleState.bleLinkQird = 0;
                            break;
                        case JT808_LINK:
                            moduleState.jt808LinkQird = 0;
                            break;
                        case HIDDEN_LINK:
                            moduleState.hideLinkQird = 0;
                            break;
                        case AGPS_LINK:
                            moduleState.agpsLinkQird = 0;
                            break;
                    }
                    LogPrintf(DEBUG_ALL, "Socket[%d] recv data done", link);
                }
            }
        }
        index = my_getstrindex(rebuf, "+CIPRXGET:", relen);
    }

    return ret;
}



/**************************************************
@bref		PDP	ָ�����
@param
@return
@note
+PDP: DEACT

**************************************************/
void deactParser(uint8_t *buf, uint16_t len)
{
    int index;
    /* ����������Ĺ�����������DEACT��������ᣬ����״̬������ */
    if (moduleState.fsmState < NORMAL_STATUS)
    	return;
    index = my_getstrindex(buf, "+PDP: DEACT", len);
    if (index < 0)
    {
        return;
    }
    socketDelAll();
    changeProcess(CPIN_STATUS);
    sendModuleCmd(CIPSHUT_CMD, NULL);
}


/**************************************************
@bref		CMT	ָ�����
@param
@return
@note

+CMT: "+8613822546264",,"24/06/03,11:48:10 +32"
param

**************************************************/

void cmtParser(uint8_t *buf, uint16_t len)
{
    uint8_t *rebuf;
    char restore[130];
    int relen;
    int index;
    insParam_s insparam;
    rebuf = buf;
    relen = len;
    index = my_getstrindex(rebuf, "+CMT:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index;
    relen -= index;
    while (index >= 0)
    {
        rebuf += 7;
        relen -= 7;
        index = getCharIndex(rebuf, relen, '"');
        LogPrintf(DEBUG_ALL, "index=%d", index);
        if (index < 0 || index >= 20)
        {
            return;
        }
        memcpy(moduleState.messagePhone, rebuf, index);
        moduleState.messagePhone[index] = 0;
        LogPrintf(DEBUG_ALL, "TEL:%s", moduleState.messagePhone);
        index = getCharIndex(rebuf, relen, '\n');
        if (index < 0)
        {
            return;
        }
        rebuf += index + 1;
        relen -= index + 1;
        index = getCharIndex(rebuf, relen, '\r');
        if (index < 0 || index >= 128)
        {
            return ;
        }
        memcpy(restore, rebuf, index);
        restore[index] = 0;
        LogPrintf(DEBUG_ALL, "Content:[%s]", restore);
        insparam.telNum = moduleState.messagePhone;
        instructionParser((uint8_t *)restore, index, SMS_MODE, &insparam);
        
        index = my_getstrindex(rebuf, "+CMT:", relen);
    }
}

void ringParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "RING", len);
    if (index < 0)
    {
        return;
    }
    sendModuleCmd(ATA_CMD, NULL);
}

void cipsendParser(uint8_t *buf, uint16_t len)
{
    int index;
    index = my_getstrindex(buf, "+CME ERROR:", len);
    if (index < 0)
    {
        return;
    }
    LogMessage(DEBUG_ALL, "socket send error");
    sendModuleCmd(CIPSTATUS_CMD, NULL);
}

/**************************************************
@bref       ģ������ݽ��ս�����
@param
@return
@note
C: 0,0,"TCP","47.107.25.39","9998","CONNECTED"
C: 1,,"","","","INITIAL"
C: 2,,"","","","INITIAL"
C: 3,,"","","","INITIAL"
C: 4,0,"TCP","47.106.96.28","10188","CLOSED"
C: 5,,"","","","INITIAL"


STATE: IP INITIAL
**************************************************/
void cipstatusParser(uint8_t *buf, uint16_t len)
{
    int index;
    ITEM item;
    uint8_t *rebuf, link;
    int16_t relen;
    rebuf = buf;
    relen = len;

    index = my_getstrindex(rebuf, "IP INITIAL", relen);
    if (index > 0)
    {
        LogMessage(DEBUG_ALL, "All socket lost");
        socketDelAll();
        return;
    }

    index = my_getstrindex(rebuf, "C:", relen);
    while (index >= 0)
    {
        rebuf += index + 3;
        relen -= index + 3;

        index = getCharIndex(rebuf, relen, '\r');
        if (index >= 0)
        {
            stringToItem(&item, rebuf, index);
            link = atoi(item.item_data[0]);
            if (my_strstr(item.item_data[5], "CONNECTED", strlen(item.item_data[5])))
            {
                socketSetConnState(link, SOCKET_CONN_SUCCESS);
            }
            else// if (my_strstr(item.item_data[5], "CLOSED", strlen(item.item_data[5])))
            {
                socketSetConnState(link, SOCKET_CONN_ERR);
            }
            rebuf += index;
            relen -= index;
        }

        index = my_getstrindex(rebuf, "C:", relen);
    }

}

/**************************************************
@bref		CBC	ָ�����
@param
@return
@note
	+CBC: 3965

	OK
**************************************************/

void cbcParser(uint8_t *buf, uint16_t len)
{
    char *rebuf;
    int index, relen;
	char restore[10] = { 0 };
	
    rebuf = buf;
    relen = len;
    index = my_getstrindex((char *)rebuf, "+CBC:", relen);
    if (index < 0)
    {
        return;
    }
    rebuf += index + 6;
    relen -= index + 6;
    index = getCharIndex(rebuf, relen, '\r');
    tmos_memcpy(restore, rebuf, index);
    restore[index] = 0;
    sysinfo.insidevoltage = atoi(restore) / 1000.0;
    LogPrintf(DEBUG_ALL, "batttery voltage %.2f", sysinfo.insidevoltage);
    
}

/**************************************************
@bref		AGPS�����Ƿ�������
@param
@return
@note
**************************************************/

uint8_t isAgpsDataRecvComplete(void)
{
	return moduleState.agpsLinkQird;
}

/**************************************************
@bref		ģ������ݽ��ս�����
@param
@return
@note
**************************************************/

void moduleRecvParser(uint8_t *buf, uint16_t bufsize)
{
    static uint16_t len = 0;
    static uint8_t dataRestore[MODULE_RX_BUFF_SIZE + 1];
    
    if (bufsize == 0)
        return;
    if (len + bufsize > MODULE_RX_BUFF_SIZE)
    {
        len = 0;
        bufsize %= MODULE_RX_BUFF_SIZE;
        LogMessage(DEBUG_ALL, "UartRecv Full!!!");
    }
    memcpy(dataRestore + len, buf, bufsize);
    len += bufsize;
    dataRestore[len] = 0;
//        uint8_t debug[1000];
//        byteToHexString(dataRestore, debug, len);
//        debug[len * 2]=0;
//        LogMessage(DEBUG_ALL, "<<<<<<<<<");
//        LogMessageWL(DEBUG_ALL, (char *)debug, len * 2);
    if (dataRestore[len - 1] != '\n')
    {
        if (dataRestore[2] != '>')
        {
            return;
        }
    }
    LogPrintf(DEBUG_ALL, "--->>>---0x%X [%d]", dataRestore, len);
    LogMessageWL(DEBUG_ALL, (char *)dataRestore, len);
    LogMessage(DEBUG_ALL, "---<<<---");
//        uint8_t debug[1000];
//        byteToHexString(dataRestore, debug, len);
//        debug[len * 2]=0;
//        LogMessage(DEBUG_ALL, "<<<<<<<<<");
//        LogMessageWL(DEBUG_ALL, (char *)debug, len * 2);

    /*****************************************/
	deactParser(dataRestore, len);
    moduleRspSuccess();
    cmtiParser(dataRestore, len);
    cmtParser(dataRestore, len);
    cmgrParser(dataRestore, len);
    cipstartRspParser(dataRestore, len);
    wifiscanParser(dataRestore, len);
    cipackParser(dataRestore, len);		//���ȥ����ע����˵��Ϊʲôcipack����������������
    if (ciprxgetParser(dataRestore, len))
    {
        if (moduleState.cmd == CIPRXGET_CMD)
        {
            return;
        }
    }

    /*****************************************/
    switch (moduleState.cmd)
    {
        case AT_CMD:
            if (distinguishOK((char *)dataRestore))
            {
                moduleState.atResponOK = 1;
            }
            break;
        case CPIN_CMD:
            if (my_strstr((char *)dataRestore, "+CPIN: READY", len))
            {
                moduleState.cpinResponOk = 1;
            }
            break;
        case CSQ_CMD:
            csqParser(dataRestore, len);
            break;
        case CGREG_CMD:
        case CEREG_CMD:
            cgregParser(dataRestore, len);
            break;
        case CIMI_CMD:
            cimiParser(dataRestore, len);
            break;
        case ICCID_CMD:
            iccidParser(dataRestore, len);
            break;
        case CGSN_CMD:
            cgsnParser(dataRestore, len);
            break;
        case CBC_CMD:
            cbcParser(dataRestore, len);
            break;
        case CGATT_CMD:
            cgattParser(dataRestore, len);
            break;
        case CIPSTART_CMD:
            cipstartParser(dataRestore, len);
            break;
        case CIPSTATUS_CMD:
            cipstatusParser(dataRestore, len);
            break;
        case CIPSEND_CMD:
            cipsendParser(dataRestore, len);
            break;
        case CIPACK_CMD:
        	cipackErrorParser(dataRestore, len);
        	break;
        case CIPRXGET_CMD:
            if (my_strstr((char *)dataRestore, "+CME ERROR: 3", len))
            {
                switch (moduleState.curQirdId)
                {
                    case NORMAL_LINK:
                        moduleState.normalLinkQird = 0;
                        break;
                    case BLE_LINK:
                        moduleState.bleLinkQird = 0;
                        break;
                    case JT808_LINK:
                        moduleState.jt808LinkQird = 0;
                        break;
                    case HIDDEN_LINK:
                        moduleState.hideLinkQird = 0;
                        break;
                    case AGPS_LINK:
                        moduleState.agpsLinkQird = 0;
                        break;
                }
                LogPrintf(DEBUG_ALL, "Link[%d] recv err", moduleState.curQirdId);
            }
            break;
        default:
            break;
    }
    moduleState.cmd = 0;
    len = 0;
}

/*--------------------------------------------------------------*/

/**************************************************
@bref		�����ź�����ʱ��
@param
@return
@note
**************************************************/

void netResetCsqSearch(void)
{
    moduleCtrl.csqTime = 90;
}

/**************************************************
@bref		socket��������
@param
@return
@note
**************************************************/

int socketSendData(uint8_t link, uint8_t *data, uint16_t len)
{
    int ret = 0;
    char param[10];
    if (socketGetConnStatus(link) == 0)
    {
        //��·δ����
        return 0;
    }
    sprintf(param, "%d,%d", link, len);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CIPSEND_CMD, param);
    createNode((char *)data, len, CIPSEND_CMD);
    if (link == NORMAL_LINK || link == JT808_LINK)
    {
        moduleState.tcpNack = len;
    }
    return len;
}
/**************************************************
@bref		ģ��˯�߿���
@param
@return
@note
**************************************************/
void moduleSleepCtl(uint8_t onoff)
{
    char param[5];
    if (onoff == 0)
    {
        return;
    }
    sprintf(param, "%d", onoff);
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(CSCLK_CMD, param);
}

/**************************************************
@bref		��ȡCSQ
@param
@return
@note
**************************************************/

void moduleGetCsq(void)
{
    sendModuleCmd(CSQ_CMD, NULL);
}

/**************************************************
@bref		��ȡ��վ
@param
@return
@note
**************************************************/

void moduleGetLbs(void)
{
    sendModuleCmd(CEREG_CMD, "2");
    sendModuleCmd(CGREG_CMD, "2");
    sendModuleCmd(CGREG_CMD, "?");
    sendModuleCmd(CEREG_CMD, "?");
}
/**************************************************
@bref		��ȡWIFIscan
@param
@return
@note
**************************************************/

void moduleGetWifiScan(void)
{
    sendModuleCmd(AT_CMD, NULL);
    sendModuleCmd(WIFISCAN_CMD, NULL);
}

/**************************************************
@bref		���Ͷ���Ϣ
@param
@return
@note
**************************************************/

void sendMessage(uint8_t *buf, uint16_t len, char *telnum)
{
    char param[60];
    sprintf(param, "\"%s\"", telnum);
    sendModuleCmd(CMGF_CMD, "1");
    sendModuleCmd(CMGS_CMD, param);
    LogPrintf(DEBUG_ALL, "len:%d", len);
    buf[len] = 0x1A;
    createNode((char *)buf, len + 1, CMGS_CMD);
}

/**************************************************
@bref       ɾ�����ж���Ϣ
@param
@return
@note
**************************************************/
void deleteAllMessage(void)
{
    sendModuleCmd(CMGD_CMD, "0,4");
}

/**************************************************
@bref		ɾ��ĳ������Ϣ
@param
@return
@note
**************************************************/


void deleteMessage(uint8_t index)
{
	char param[50];
	sprintf(param, "%d", index);
    sendModuleCmd(CMGD_CMD, param);
}


/**************************************************
@bref		��ѯ�����Ƿ������
@param
@return
@note
**************************************************/

void querySendData(uint8_t link)
{
    char param[5];
    sprintf(param, "%d", link);
    sendModuleCmd(CIPACK_CMD, param);
}


/**************************************************
@bref		��ѯģ���ص�ѹ
@param
@return
@note
**************************************************/

void queryBatVoltage(void)
{
    //sendModuleCmd(CBC_CMD, NULL);
}

/**************************************************
@bref		��ѯģ���¶�
@param
@return
@note
**************************************************/

void queryTemperture(void)
{
	
}

/**************************************************
@bref		��ն�����������
@param
@return
@note
����ģ�����źŲ��õ�����£��������׶ѻ�������
������Ҳ���ղ��������ˣ����Զ������һ�ζ���
**************************************************/

void delMsgReqSet(void)
{
	sysinfo.delMsgReq = 1;
	LogMessage(DEBUG_ALL, "delMsgReqSet==>ok");
}

/**************************************************
@bref		��ն����������
@param
@return
@note
**************************************************/

void delMsgReqClear(void)
{
	sysinfo.delMsgReq = 0;
	LogMessage(DEBUG_ALL, "delMsgReqClear==>ok");
}


/**************************************************
@bref		��ȡ�ź�ֵ
@param
@return
@note
**************************************************/

uint8_t getModuleRssi(void)
{
    return moduleState.rssi;
}

/**************************************************
@bref		��ȡIMSI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMSI(void)
{
    return moduleState.IMSI;
}
/**************************************************
@bref		��ȡIMEI
@param
@return
@note
**************************************************/

uint8_t *getModuleIMEI(void)
{
    return moduleState.IMEI;
}



/**************************************************
@bref		��ȡICCID
@param
@return
@note
**************************************************/

uint8_t *getModuleICCID(void)
{
    return moduleState.ICCID;
}

/**************************************************
@bref		��ȡMCC
@param
@return
@note
**************************************************/

uint16_t getMCC(void)
{
    return moduleState.mcc;
}

/**************************************************
@bref		��ȡMNC
@param
@return
@note
**************************************************/

uint8_t getMNC(void)
{
    return moduleState.mnc;
}

/**************************************************
@bref		��ȡLAC
@param
@return
@note
**************************************************/

uint16_t getLac(void)
{
    return moduleState.lac;
}

/**************************************************
@bref		��ȡCID
@param
@return
@note
**************************************************/

uint32_t getCid(void)
{
    return moduleState.cid;
}

/**************************************************
@bref		��ȡδ�����ֽ������ж��Ƿ��ͳɹ�
@param
@return
@note
**************************************************/

uint32_t getTcpNack(void)
{
    return moduleState.tcpNack;
}

/**************************************************
@bref       ��ѯģ��汾
@param
@return
@note
**************************************************/

char *getQgmr(void)
{
//    return moduleState.qgmr;
}


/**************************************************
@bref		�л�ģʽ4�ص�����
@param
@return
@note
**************************************************/
void changeMode4Callback(void)
{
	changeProcess(CPIN_STATUS);
}

/**************************************************
@bref		ģʽ4ģ���Ƿ��������
@param
@return
@note
**************************************************/
uint8_t isModuleOfflineStatus(void)
{
	if (moduleState.fsmState == OFFLINE_STATUS)
		return 1;
	return 0;
}


/**************************************************
@bref		ģ���Ƿ�ﵽ����״̬
@param
@return
@note
**************************************************/

uint8_t isModuleRunNormal(void)
{
    if (moduleState.fsmState == NORMAL_STATUS)
        return 1;
    return 0;
}

/**************************************************
@bref		ģ��ﵽ��������
@param
@return
@note
**************************************************/

uint8_t isModulePowerOnOk(void)
{
    if (moduleState.powerOnTick > 10)
        return 1;
    return 0;
}

/**************************************************
@bref		�Ҷϵ绰
@param
@return
@note
**************************************************/

void stopCall(void)
{
    sendModuleDirect("ATH\r\n");
}
/**************************************************
@bref		����绰
@param
@return
@note
**************************************************/

void callPhone(char *tel)
{
    char param[50];
    sprintf(param, "ATD%s;\r\n", tel);
    LogPrintf(DEBUG_ALL, "Call %s", tel);
    sendModuleDirect(param);

}


