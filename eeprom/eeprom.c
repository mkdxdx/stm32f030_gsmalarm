#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_gpio.h"
#include "eeprom.h"
#include "delay/delay.h"



void EE_I2C_Init(void) {
	RCC_APB1PeriphClockCmd(EE_I2C_IfRCC,ENABLE);

	GPIO_InitTypeDef gis;
	gis.GPIO_Pin = EE_I2C_Pin_SCL | EE_I2C_Pin_SDA;
	gis.GPIO_Speed = GPIO_Speed_50MHz;
	gis.GPIO_Mode = GPIO_Mode_AF;
	gis.GPIO_OType = GPIO_OType_OD;
	gis.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(EE_I2C_IfPort, &gis);

	GPIO_PinAFConfig(EE_I2C_IfPort,EE_I2C_PS_SCL, GPIO_AF_4);
	GPIO_PinAFConfig(EE_I2C_IfPort,EE_I2C_PS_SDA, GPIO_AF_4);

	I2C_InitTypeDef i2cis;
	i2cis.I2C_Mode = I2C_Mode_I2C;
	i2cis.I2C_Ack = I2C_Ack_Disable;
	i2cis.I2C_Timing = 0x00901A53;
	i2cis.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(EE_I2C_If,&i2cis);

	I2C_Cmd(EE_I2C_If,ENABLE);
}



uint8_t EE_I2C_ReadAt(uint8_t addr) {
	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(EE_I2C_If,EEAWR,1,I2C_SoftEnd_Mode,I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_TXIS) == RESET);

	I2C_SendData(EE_I2C_If,addr);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_TC) == RESET);

	I2C_TransferHandling(EE_I2C_If,EEAWR,1,I2C_AutoEnd_Mode,I2C_Generate_Start_Read);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_RXNE) == RESET);

	uint8_t regvalue = I2C_ReceiveData(EE_I2C_If);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_STOPF) == RESET);

	I2C_ClearFlag(EE_I2C_If,I2C_FLAG_STOPF);

	return regvalue;
}

void EE_I2C_WriteAt(uint8_t addr, uint8_t data) {

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(EE_I2C_If,EEAWR,2,I2C_AutoEnd_Mode,I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_TXIS) == RESET);

	I2C_SendData(EE_I2C_If,addr);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_TXIS) == RESET);

	I2C_SendData(EE_I2C_If,data);

	while(I2C_GetFlagStatus(EE_I2C_If,I2C_FLAG_STOPF) == RESET);

	I2C_ClearFlag(EE_I2C_If,I2C_FLAG_STOPF);

	delay_ms(EE_WRITE_DELAY);
}

void EE_I2C_WriteStringAt(uint8_t addr, char * str) {
	// it will write with null character so any string reading function will know when to stop
	do {
		EE_I2C_WriteAt(addr++,*str);
	} while (*str++!='\0');
}



