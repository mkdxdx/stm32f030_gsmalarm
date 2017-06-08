// Dead simple eeprom writing library

#define EEI2C					I2C1

#define EEAWR					0b10100000
#define EEARD					0b10100001

#define EE_I2C_If				I2C1
#define EE_I2C_IfPort			GPIOA
#define EE_I2C_IfPortRCC 		RCC_AHBPeriph_GPIOA
#define EE_I2C_IfRCC			RCC_APB1Periph_I2C1
#define EE_I2C_Pin_SCL			GPIO_Pin_9
#define EE_I2C_Pin_SDA			GPIO_Pin_10
#define EE_I2C_PS_SCL			GPIO_PinSource9
#define EE_I2C_PS_SDA			GPIO_PinSource10

#define EE_I2C_RstAddr()		EE_I2C_SetAddr(0x00)

#define EE_WRITE_DELAY			6	// absolute worst scenario, no ACK polling


void EE_I2C_Init(void);
uint8_t EE_I2C_ReadAt(uint8_t addr);
void EE_I2C_WriteAt(uint8_t addr, uint8_t data);

