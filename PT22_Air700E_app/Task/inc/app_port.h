#ifndef APP_PORT
#define APP_PORT

#include <stdint.h>
#include "config.h"

#define APPUSART0_BUFF_SIZE 10
#define APPUSART1_BUFF_SIZE 1024
#define APPUSART2_BUFF_SIZE 128
#define APPUSART3_BUFF_SIZE 768

//��©�������
#define GPIOA_MODE_OD(pin)			{GPIOA_ModeCfg(pin, GPIO_ModeIN_PU);GPIOA_ResetBits(pin);}
#define GPIOB_MODE_OD(pin)			{GPIOB_ModeCfg(pin, GPIO_ModeIN_PU);GPIOB_ResetBits(pin);}
#define GPIOA_OD_SetBit(pin)      	(R32_PA_DIR  &= ~pin)
#define GPIOA_OD_ResetBit(pin)      (R32_PA_DIR  |=  pin)
#define GPIOB_OD_SetBit(pin)      	(R32_PB_DIR  &= ~pin)
#define GPIOB_OD_ResetBit(pin)      (R32_PB_DIR  |=  pin)



//4Gģ���漰IO
#define SUPPLY_PIN      
#define PORT_SUPPLY_ON  
#define PORT_SUPPLY_OFF 

#define POWER_PIN    	GPIO_Pin_7		//PB7
#define PORT_PWRKEY_H   GPIOB_ResetBits(POWER_PIN)
#define PORT_PWRKEY_L   GPIOB_SetBits(POWER_PIN)

#define RST_PIN			GPIO_Pin_15		//PA15
#define PORT_RSTKEY_H	GPIOA_ResetBits(RST_PIN)
#define PORT_RSTKEY_L	GPIOA_SetBits(RST_PIN)


#define RING_PIN		GPIO_Pin_11		//PB11

#define DTR_PIN			GPIO_Pin_4		//PB4
#define DTR_HIGH		GPIOB_SetBits(DTR_PIN)
#define DTR_LOW			GPIOB_ResetBits(DTR_PIN)

//DA217�漰IO
#define GSPWR_PIN       GPIO_Pin_8		//PA8
#define GSPWR_ON        portGsensorPwrCtl(1)
#define GSPWR_OFF       portGsensorPwrCtl(0)
#define GSINT_PIN       GPIO_Pin_9		//PA9
#define GSINT_DET		(GPIOA_ReadPortPin(GSINT_PIN)?1:0)

//IIC �漰IO
#define SCL_PIN         GPIO_Pin_14
#define SDA_PIN         GPIO_Pin_15

//LED �漰IO
#define LED1_PIN        GPIO_Pin_10		//PB10
#define LED1_ON         GPIOB_ResetBits(LED1_PIN)
#define LED1_OFF        GPIOB_SetBits(LED1_PIN)
#define LED2_PIN
#define LED2_ON
#define LED2_OFF

//GPS �漰IO
#define GPSPWR_PIN		GPIO_Pin_14		//PA14
#define GPSPWR_ON		GPIOA_ResetBits(GPSPWR_PIN)
#define GPSPWR_OFF		GPIOA_SetBits(GPSPWR_PIN)

#define GPSLNA_PIN      
#define GPSLNA_ON       
#define GPSLNA_OFF  

//KEY �漰IO
#define PWR_KEY_PIN		GPIO_Pin_11		//PA11
#define PWR_KEY_READ	(GPIOA_ReadPortPin(PWR_KEY_PIN)?1:0)

//SOSKEY �漰IO
#define SOS_KEY_PIN		GPIO_Pin_10		//PA10
#define SOS_KEY_READ	(GPIOA_ReadPortPin(SOS_KEY_PIN)?1:0)


#define VCARD_ADCPIN	GPIO_Pin_13		//PA13
#define ADC_CHANNEL		CH_EXTIN_3




// (MAXCALCTICKS * 5) + (max remainder) must be <= (uint16 max),
// so: (13105 * 5) + 7 <= 65535
#define MAXCALCTICKS  ((uint16_t)(13105)) 
//#define	BEGYEAR	        2000UL     // UTC started at 00:00:00 January 1, 2020
#define	DAY             86400UL  // 24 hours * 60 minutes * 60 seconds


typedef enum
{
    APPUSART0,
    APPUSART1,
    APPUSART2,
    APPUSART3,
} UARTTYPE;

typedef struct
{
    uint8_t *rxbuf;
    uint8_t init;
    uint16_t rxbufsize;
    __IO uint16_t rxbegin;
    __IO uint16_t rxend;

    void (*rxhandlefun)(uint8_t *, uint16_t len);
    void (*txhandlefun)(uint8_t *, uint16_t len);

} UART_RXTX_CTL;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} system_date_s;

extern UART_RXTX_CTL usart0_ctl;
extern UART_RXTX_CTL usart1_ctl;
extern UART_RXTX_CTL usart2_ctl;
extern UART_RXTX_CTL usart3_ctl;

void pollUartData(void);
void portUartCfg(UARTTYPE type, uint8_t onoff, uint32_t baudrate, void (*rxhandlefun)(uint8_t *, uint16_t len));
void portUartSend(UART_RXTX_CTL *uartctl, uint8_t *buf, uint16_t len);

void portModuleGpioCfg(uint8_t state);

void portLedGpioCfg(uint8_t onoff);
void portGpsGpioCfg(uint8_t onoff);
void portMicGpioCfg(void);
void portLdrGpioCfg(uint8_t onoff);
void portPwrKeyCfg(void);
void portSosKeyCfg(uint8_t onoff);



void portGpioSetDefCfg(void);

void portSysReset(void);
void portGpioWakeupIRQHandler(void);


uint8_t iicReadData(uint8_t addr, uint8_t regaddr, uint8_t *data, uint8_t len);
uint8_t iicWriteData(uint8_t addr, uint8_t reg, uint8_t data);
void portGsensorPwrCtl(uint8 onoff);
void portIICCfg(void);

void portGsensorCtl(uint8_t onoff);


void portRtcCfg(void);
void portUpdateRtcOffset(uint8_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second);
void portGetRtcDateTime(uint16_t *year, uint8_t *month, uint8_t *date, uint8_t *hour, uint8_t *minute, uint8_t *second);
void portUpdateRtcDateTime(uint8_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute, uint8_t second);
int portSetNextAlarmTime(void);
void portSetNextWakeUpTime(void);
int portSetNextMode4AlarmTime(void);

void portLowPowerCfg(void);
void portUpdateStep(void);

void portClearStep(void);

void portSaveStep(void);

void portAdcCfg(uint8_t onoff);
float portGetAdcVol(ADC_SingleChannelTypeDef channel);

void portSleepEn(void);
void portSleepDn(void);

void portWdtCfg(void);
void portWdtFeed(void);
void portWdtCancel(void);

void portFsclkChange(uint8_t type);

#endif
