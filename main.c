#include <stdlib.h>
#include <string.h>

#include "stm32f0xx.h"
#include "delay.h"

#include "stm32f0xx_exti.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"

// definition section
#define	TRIP_PORT	GPIOA
#define TRIP_RCC	RCC_AHBPeriph_GPIOA
#define TRIP_CH1	GPIO_Pin_0
#define TRIP_CH2	GPIO_Pin_1
#define TRIP_PS_1	GPIO_PinSource0
#define TRIP_PS_2	GPIO_PinSource1

#define TRIP_1		0
#define TRIP_2		1

#define LED_PORT	GPIOA
#define LED_RCC		RCC_AHBPeriph_GPIOA
#define LED_STATUS	GPIO_Pin_4
#define LED_ARMED	GPIO_Pin_5

#define SIM_PORT	GPIOB
#define SIM_RCC		RCC_AHBPeriph_GPIOB
#define SIM_PIN_RST	GPIO_Pin_1

#define ACTION_PORT	GPIOA
#define ACTION_RCC	RCC_AHBPeriph_GPIOA
#define ACTION_PIN	GPIO_Pin_6

#define KEY_PORT	GPIOA
#define KEY_RCC		RCC_AHBPeriph_GPIOA
#define KEY_PIN		GPIO_Pin_7

#define CONF_PORT	GPIOA
#define CONF_RCC	RCC_AHBPeriph_GPIOA
#define CONF_PIN	GPIO_Pin_14


#define I2C_If			I2C1
#define I2C_IfPort		GPIOA
#define I2C_IfPortRCC 	RCC_AHBPeriph_GPIOA
#define I2C_IfRCC		RCC_APB1Periph_I2C1
#define I2C_Pin_SCL		GPIO_Pin_9
#define I2C_Pin_SDA		GPIO_Pin_10
#define I2C_PS_SCL		GPIO_PinSource9
#define I2C_PS_SDA		GPIO_PinSource10

#define USART_If		USART1
#define USART_IfPort	GPIOA
#define USART_IfPortRCC RCC_AHBPeriph_GPIOA
#define USART_IfRCC		RCC_APB2Periph_USART1
#define USART_Pin_RX	GPIO_Pin_3
#define USART_Pin_TX	GPIO_Pin_2
#define USART_PS_RX		GPIO_PinSource3
#define USART_PS_TX		GPIO_PinSource2
#define USART_GPIO_AF	GPIO_AF_1
#define USART_IRQ		USART1_IRQn

#define RXBUF_LEN		255
#define RXBUF_START		0

#define GSM_MAX_CMD_TIMEOUT	1	// timeouts in seconds
#define TRIPPED_TIMEOUT		10
#define ARMING_TIMEOUT		20
#define ALARM_TIMEOUT		60

#define ARMED_ON()		GPIO_ResetBits(LED_PORT, LED_ARMED)
#define ARMED_OFF()		GPIO_SetBits(LED_PORT, LED_ARMED)
#define STATUS_ON()		GPIO_ResetBits(LED_PORT, LED_STATUS)
#define STATUS_OFF()	GPIO_SetBits(LED_PORT, LED_STATUS)
#define ACTION_ON()		GPIO_SetBits(ACTION_PORT, ACTION_PIN)
#define ACTION_OFF()	GPIO_ResetBits(ACTION_PORT, ACTION_PIN)


enum E_ALARM_STATE {
	AS_IDLE = 0,
	AS_ARMING,
	AS_ARMED,
	AS_TRIPPED
};

enum E_GSMRESULT {
	GR_NONE = 0,
	GR_OK,
	GR_ERROR,
	GR_TIMEOUT
};

enum E_GSMSTATUS {
	GS_NONE = 0,
	GS_AVAILABLE,
	GS_LOWPWR
};

// prot
void Init(void);
void InitRCC(void);
void InitGPIO(void);
void InitIT(void);
void InitEXTI(void);
void InitComm(uint32_t baud);
void InitGSM(void);
void AlarmInit(void);
void AlarmLoop(void);
void GSMReset(void);
void ClrBuffer(void);
void USendStr(char *str);
void USend(uint8_t data);
void delay_s(uint8_t s);
enum E_GSMRESULT GSMSendCmd(const char *cmd);



// globals section
const char GSM_RSP_OK[] = "OK\r\n";
const char GSM_RSP_ERROR[] = "ERROR\r\n";

const char GSM_CMD_AT[] = "AT";
const char GSM_CMD_USSD[] = "AT+CUSD=";
const char GSM_CMD_SMSSend[] = "AT+CMGS=\"";
const char GSM_CMD_MSGF[] = "AT+CMGF=1";
const char GSM_CMD_LISTSMSALL[] = "AT+CMGL=\"ALL\"";
const char GSM_CMD_CLRMSG[] = "AT+CMGDA=\"DEL ALL\"";

const char GSM_TXT_MSGUNREAD[] = "REC UNREAD";
const char GSM_TXT_ARMED[] = "Alarm armed.";
const char GSM_TXT_TRIPPED[] = "Alarm tripped!";
const char GSM_TXT_INIT[] = "Alarm started.";
const char GSM_TXT_SAVCONF[] = " - Config saved.";
const char GSM_TXT_CHANN[] = "CHANNEL";
const char GSM_TXT_CHREED[] = "REED";
const char GSM_TXT_CHPIR[] = "PIR";

// eeprom address of saved data
const uint8_t ADDR_T_TRIPPED = 	0x10;
const uint8_t ADDR_T_ARMING =  	0x12;
const uint8_t ADDR_T_ALARM =	0x14;
const uint8_t ADDR_T_OWNUM =	0x16;

volatile enum E_ALARM_STATE ALARM_STATE = AS_IDLE;
volatile enum E_GSMSTATUS GSM_STATUS = GS_NONE;

volatile uint8_t TRIP_STATE	= 0x00;

volatile uint16_t rxbuf_index = RXBUF_START;
volatile char rxbuf[RXBUF_LEN];
volatile char strval[5];

// owner number and delay values
volatile char ownnum[20];
volatile uint8_t T_TRIPPED=	TRIPPED_TIMEOUT;
volatile uint8_t T_ARMING= 	ARMING_TIMEOUT;
volatile uint8_t T_ALARM=	ALARM_TIMEOUT;

// interrupt handlers

void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART_If,USART_IT_RXNE)!=RESET) {

		USART_ClearITPendingBit(USART_If,USART_IT_RXNE);
		rxbuf[rxbuf_index] = USART_ReceiveData(USART_If);
		rxbuf_index++;

		if (rxbuf_index>=RXBUF_LEN-1) {
			rxbuf_index = RXBUF_START;
		} else {
			rxbuf[rxbuf_index] = '\0';
		}
	}

	if (USART_GetITStatus(USART_If,USART_IT_TC)!=RESET) {
		USART_ClearITPendingBit(USART_If,USART_IT_TC);
	}
}


void EXTI0_1_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		if (GPIO_ReadInputDataBit(TRIP_PORT, TRIP_CH1) == 0) {
			TRIP_STATE |= 1<<TRIP_1;
		} else {
			TRIP_STATE &= (~(1<<TRIP_1));
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		if (GPIO_ReadInputDataBit(TRIP_PORT, TRIP_CH2) == 0) {
			TRIP_STATE |= 1<<TRIP_2;
		} else {
			TRIP_STATE &= (~(1<<TRIP_2));
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}


int main(void)
{
	Init();
    while(1) {
    	AlarmLoop();
    }
}



void Init(void) {
	Delay_Init(48);

	InitRCC();
	InitGPIO();
	InitComm(115200);
	EE_I2C_Init();

	InitIT();
	InitEXTI();
	__enable_irq();

	AlarmInit();
	LoadConfig();
	InitGSM();
}


void InitRCC(void) {
	RCC_APB2PeriphClockCmd((USART_IfRCC| RCC_APB2Periph_SYSCFG), ENABLE);
	RCC_AHBPeriphClockCmd((USART_IfPortRCC | I2C_IfPortRCC | TRIP_RCC | LED_RCC | SIM_RCC | ACTION_RCC | KEY_RCC | CONF_RCC), ENABLE);
}


void InitGPIO(void) {
	GPIO_InitTypeDef gis;

	// set trip inputs
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (TRIP_CH1 | TRIP_CH2);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_IN;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(TRIP_PORT, &gis);

	// set led outputs
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (LED_STATUS | LED_ARMED);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_OUT;
	gis.GPIO_OType = GPIO_OType_OD;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LED_PORT, &gis);

	// set sim module RST pin
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (SIM_PIN_RST);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_OUT;
	gis.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(SIM_PORT, &gis);

	// set action pin
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (ACTION_PIN);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_OUT;
	gis.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(ACTION_PORT, &gis);

	// set key pin
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (KEY_PIN);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_IN;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(KEY_PORT, &gis);


	// set conf pin
	GPIO_StructInit(&gis);
	gis.GPIO_Pin = (CONF_PIN);
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_IN;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(CONF_PORT, &gis);


	// set alternate interface functions
	GPIO_PinAFConfig(USART_IfPort, USART_PS_TX, USART_GPIO_AF);
	GPIO_PinAFConfig(USART_IfPort, USART_PS_RX, USART_GPIO_AF);

	GPIO_StructInit(&gis);
	gis.GPIO_Pin = USART_Pin_TX;
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_AF;
	gis.GPIO_OType = GPIO_OType_PP;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USART_IfPort, &gis);

	gis.GPIO_Pin = USART_Pin_RX;
	gis.GPIO_Mode = GPIO_Mode_AF;
	gis.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(USART_IfPort, &gis);

}

void InitComm(uint32_t baud) {
	USART_InitTypeDef uis;
	uis.USART_BaudRate            = baud;
	uis.USART_WordLength          = USART_WordLength_8b;
	uis.USART_StopBits            = USART_StopBits_1;
	uis.USART_Parity              = USART_Parity_No ;
	uis.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uis.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART_If,&uis);
	USART_Cmd(USART_If,ENABLE);
}

void InitIT(void) {
	NVIC_InitTypeDef nitd;
	nitd.NVIC_IRQChannel = (EXTI0_1_IRQn);
	nitd.NVIC_IRQChannelCmd = ENABLE;
	nitd.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&nitd);

	nitd.NVIC_IRQChannel = (USART_IRQ);
	nitd.NVIC_IRQChannelCmd = ENABLE;
	nitd.NVIC_IRQChannelPriority = 1;
	NVIC_Init(&nitd);

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
}

void InitEXTI(void) {
	EXTI_InitTypeDef extis;

	SYSCFG_EXTILineConfig(TRIP_PORT, (TRIP_PS_1));
	SYSCFG_EXTILineConfig(TRIP_PORT, (TRIP_PS_2));

	extis.EXTI_Line = (EXTI_Line0 | EXTI_Line1);
	extis.EXTI_LineCmd = ENABLE;
	extis.EXTI_Mode = EXTI_Mode_Interrupt;
	extis.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&extis);
}

void InitGSM(void) {
	// reset module
	// check if module is available
	// check if module is registered
	// set sms PUD mode


	uint8_t i = 5;
	while ((GSM_STATUS != GS_AVAILABLE) && (i--)) {
		if (GSMSendCmd("AT") == GR_OK) {
			GSM_STATUS = GS_AVAILABLE;
		} else {
			delay_s(1);
		}
	}

	if (GSM_STATUS == GS_AVAILABLE) {
		GSMSendCmd(GSM_CMD_MSGF);
		delay_s(1);
		GSMSendCmd(GSM_CMD_CLRMSG);
		delay_s(1);
		GSMSendSMS(GSM_TXT_INIT,ownnum);
		delay_s(5);
	}

}

void AlarmInit(void) {
	ALARM_STATE = AS_IDLE;
	ARMED_ON();
	STATUS_ON();
}

void AlarmLoop(void) {
	// turn on TRIP LED if any of channels are tripped
	switch (ALARM_STATE) {
	case AS_IDLE: {
		ARMED_OFF();
		ACTION_OFF();
		if (GPIO_ReadInputDataBit(CONF_PORT, CONF_PIN) == 0) {
			STATUS_OFF();
			CheckConfMessages();
			STATUS_ON();
		} else {
			if (TRIP_STATE != 0) {
				STATUS_OFF();
				delay_ms(500);
				STATUS_ON();
				delay_ms(500);
			} else {
				STATUS_ON();
			}

			if (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == 0) {
				delay_s(1);
				ALARM_STATE = AS_ARMING;
			}

		}
		// do nothing
		// if alarm is armed, switch to AS_ARMING
		break;
	}
	case AS_ARMING:
	{
		STATUS_ON();
		// wait for 30s and then set ALARM_STATE to AS_ARMED
		uint8_t i = T_ARMING;
		while ((GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == 0) && (i--)) {
			delay_ms(500);
			ARMED_ON();
			delay_ms(500);
			ARMED_OFF();
		}

		if (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == 0) {

			ALARM_STATE = AS_ARMED;
		} else {
			ALARM_STATE = AS_IDLE;
		}

		break;
	}
	case AS_ARMED: {
		// wait every second for a channel to trip
		// if after 10s channel is tripped again switch ALARM_STATE to AS_TRIPPED
		// if alarm is disarmed, switch ALARM_STATE to AS_IDLE
		TRIP_STATE = 0x00;
		STATUS_OFF();
		ARMED_ON();
		ACTION_OFF();


		while (ALARM_STATE == AS_ARMED) {
			if ((GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == 1)) {
				ALARM_STATE = AS_IDLE;
			} else {
				if (TRIP_STATE != 0) {
					ALARM_STATE = AS_TRIPPED;
					break;
				}
			}
		}
		break;
	}
	case AS_TRIPPED: {
		STATUS_ON();
		uint16_t i = T_TRIPPED;
		while (i--) {
			if (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) == SET) {
				ALARM_STATE = AS_IDLE;
				break;
			} else {
				delay_s(1);
			}

		}

		// send alerting sms/make call
		// sms will contain channel statuses
		// turn ACTION/RELAY channel on
		// wait 30s and switch ALARM_STATE to AS_ARMED
		if (ALARM_STATE == AS_TRIPPED) {
			TrippedAction();
			i = T_ALARM*5;
			while (1) {
				if (i <= 1) {
					ALARM_STATE = AS_ARMING;
					break;
				} else if (GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN) != 0) {
					ALARM_STATE = AS_IDLE;
					break;
				} else {
					delay_ms(100);
					STATUS_ON();
					ARMED_OFF();
					delay_ms(100);
					STATUS_OFF();
					ARMED_ON();
				}
				i--;
			}
		}

		break;
	}
	default:
		break;
	}
}

void CheckConfMessages(void) {
	// poll sim module for availability
	if (GSMSendCmd(GSM_CMD_AT) == GR_OK) {
		// set text mode
		if (GSMSendCmd(GSM_CMD_MSGF) == GR_OK) {
			// check if any messages are in memory
			if (GSMSendCmd(GSM_CMD_LISTSMSALL) == GR_OK) {
				// if messages found, parse first
				if (strstr(rxbuf,GSM_TXT_MSGUNREAD) != NULL) {
					// if receive successful, try to parse it
					ParseConfMessage(rxbuf);
					// clear messages if some present
				}
				GSMSendCmd(GSM_CMD_CLRMSG);
			}
		}
	}
	delay_s(5);
}

void ParseConfMessage(char * buffer) {
	// if any of received messages matches config format
	// save sender number as owner number
	// save delay values for
	// TARMXXX (arming delay in seconds)
	// TTRIPXXX (tripping delay in seconds)
	// TALRMXXX (alarm delay in seconds)

	// composed message
	char msg[50];

	// temporary number storage
	char numtemp[20];


	uint8_t i = 0;
	for (i=0;i<20;i++) {
		numtemp[i] = '\0';
	}
	i = 0;
	char *p;
	// find number start
	p = strstr(buffer,"\",\"+");
	if (p != NULL) {
		// skip first three symbols
		p++;
		p++;
		p++;
		do {
			numtemp[i] = *p++;
			i++;
		} while (*p != '"');
	}

	p = strstr(buffer,"TARM");
	if (p != NULL) {
		p++;
		p++;
		p++;
		p++;
		uint8_t v = (uint8_t)atoi(p);
		if (v>0) {
			T_ARMING = v;
		}

	}

	p = strstr(buffer,"TTRIP");
	if (p != NULL) {
		p++;
		p++;
		p++;
		p++;
		p++;
		uint8_t v = (uint8_t)atoi(p);
		if (v>0) {
			T_TRIPPED = v;
		}
	}

	p = strstr(buffer,"TALRM");
	if (p != NULL) {
		p++;
		p++;
		p++;
		p++;
		p++;
		uint8_t v = (uint8_t)atoi(p);
		if (v>0) {
			T_ALARM = v;
		}
	}

	// if any of the parameters matched, save number
	if (p != NULL) {
		strcpy(ownnum,numtemp);
		uint8_t i = 0;
		for (i = 0; i<5; i++) {
			BlinkStatus(100);
		}
		BlinkStatus(100);
		SaveConf();

		// send acknowledgement
		strcpy(msg,"ARM:");
		itoa(T_ARMING,strval,10);
		strcat(msg,strval);

		strcat(msg,",TRIP:");
		itoa(T_TRIPPED,strval,10);
		strcat(msg,strval);

		strcat(msg,",ALRM:");
		itoa(T_ALARM,strval,10);
		strcat(msg,strval);

		strcat(msg,GSM_TXT_SAVCONF);

		GSMSendSMS(msg,ownnum);
		delay_s(5);

	}
}

void LoadConfig(void) {
	// load owner's number and delay values from eeprom
	T_TRIPPED = EE_I2C_ReadAt(ADDR_T_TRIPPED);
	T_ARMING = EE_I2C_ReadAt(ADDR_T_ARMING);
	T_ALARM = EE_I2C_ReadAt(ADDR_T_ALARM);

	char c;
	uint8_t addr = ADDR_T_OWNUM;
	uint8_t i = 0;
	do {
		c = EE_I2C_ReadAt(addr++);
		ownnum[i] = c;
		i++;
	} while (c != '\0');
}

void SaveConf(void) {
	// save owner's number and delay values to eeprom
	EE_I2C_WriteAt(ADDR_T_TRIPPED,T_TRIPPED);
	EE_I2C_WriteAt(ADDR_T_ARMING,T_ARMING);
	EE_I2C_WriteAt(ADDR_T_ALARM,T_ALARM);
	EE_I2C_WriteStringAt(ADDR_T_OWNUM, ownnum);
}

void BlinkStatus(uint16_t ms) {
	STATUS_OFF();
	delay_ms(ms);
	STATUS_ON();
	delay_ms(ms);
}

void TrippedAction(void) {
	ACTION_ON();
	if (GSM_STATUS == GS_AVAILABLE) {
		GSMSendSMS(GSM_TXT_TRIPPED,ownnum);
	}
}



void delay_s(uint8_t s) {
	while (s) {
		delay_ms(1000);
		s--;
	}
}

void USend(uint8_t data) {
	USART_SendData(USART_If,data);
	while(!USART_GetFlagStatus(USART_If,USART_FLAG_TC));
}


void USendStr(char *str) {
	while (*str!='\0') {
		USend(*str++);
	}
}

void ClrBuffer(void) {
	uint16_t i;
	for (i = 0; i<RXBUF_LEN; i++)
		rxbuf[i] = '\0';
	rxbuf_index = RXBUF_START;
}

void GSMReset(void) {
	GPIO_ResetBits(SIM_PORT,SIM_PIN_RST);
	delay_s(1);
	GPIO_SetBits(SIM_PORT,SIM_PIN_RST);
	delay_s(2);
}


void GSMSendSMS(char * text, char * number) {
	char sendcmd[50];
	uint8_t i;
	for (i = 0; i<50; i++) {
		sendcmd[i] = '\0';
	}
	strcpy(sendcmd,GSM_CMD_SMSSend);
	strcat(sendcmd,number);
	strcat(sendcmd,"\"");
	GSMSendCmd(sendcmd);
	if (strstr(rxbuf,">") != NULL) {
		USendStr(text);
		USend(26);
	}

}

enum E_GSMRESULT GSMSendCmd(const char *cmd) {
	ClrBuffer();
	USendStr(cmd);
	USend(13);
	delay_ms(200);
	uint8_t att = 0;
	enum E_GSMRESULT result = GR_NONE;
	while (result == GR_NONE) {
		if ( strstr(rxbuf,GSM_RSP_OK) != NULL) {
			result = GR_OK;
			break;
		} else if (strstr(rxbuf,GSM_RSP_ERROR) != NULL) {
			result = GR_ERROR;
			break;
		}
		delay_s(1);
		att++;
		if (att>GSM_MAX_CMD_TIMEOUT)
			result = GR_TIMEOUT;
	}
	return result;
}
