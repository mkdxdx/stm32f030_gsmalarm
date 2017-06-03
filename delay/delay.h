#ifndef __DELAY_H
#define __DELAY_H

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

#define  u8    uint8_t
#define  u16   uint16_t
#define  u32	uint32_t

void delay_ms(u16 nms);
void delay_us(u32 nus);
void Delay(u32 count);
void Delay_Init(u8 SYSCLK);

extern u16 fac_ms;//全局变量
extern u8 fac_us;//全局变量


#endif

