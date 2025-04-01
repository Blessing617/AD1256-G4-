#ifndef __ADS1256_H__
#define __ADS1256_H__

#include "main.h"
#include "gpio.h"
#include "delay.h"
#include "stdio.h"

/*
    ADS1256模块    STM32-V7开发板(示波器接口)
      +5V   <------  5.0V      5V供电
      GND   -------  GND       地
      DRDY  ------>  PA10      准备就绪
      CS    <------  PB3       SPI_CS
      DIN   <------  PA8       SPI_MOSI
      DOUT  ------>  PA9       SPI_MISO
      SCLK  <------  PB10      SPI时钟
      GND   -------  GND       地
      PDWN  <------  PB4       掉电控制
      RST   <------  PB5       复位信号
      NC   空脚
      NC   空脚
*/
 

#define SCK_CLK_ENABLE() 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define SCK_GPIO				GPIOB
#define SCK_PIN					GPIO_PIN_10
#define SCK_1()					SCK_GPIO->BSRR = SCK_PIN
#define SCK_0()					SCK_GPIO->BSRR = ((uint32_t)SCK_PIN << 16U)	
 
#define DIN_CLK_ENABLE() 		__HAL_RCC_GPIOA_CLK_ENABLE()
#define DIN_GPIO				GPIOA
#define DIN_PIN					GPIO_PIN_8
#define DIN_1()					DIN_GPIO->BSRR = DIN_PIN
#define DIN_0()					DIN_GPIO->BSRR = ((uint32_t)DIN_PIN << 16U)	
 
#define CS_CLK_ENABLE() 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define CS_GPIO					GPIOB
#define CS_PIN					GPIO_PIN_3
#define CS_1()					CS_GPIO->BSRR = CS_PIN
#define CS_0()					CS_GPIO->BSRR = ((uint32_t)CS_PIN << 16U)	
 
#define DOUT_CLK_ENABLE() 		__HAL_RCC_GPIOA_CLK_ENABLE()
#define DOUT_GPIO				GPIOA
#define DOUT_PIN				GPIO_PIN_9
#define DOUT_IS_HIGH()			((DOUT_GPIO->IDR & DOUT_PIN) != 0)
 
#define DRDY_CLK_ENABLE() 		__HAL_RCC_GPIOA_CLK_ENABLE()
#define DRDY_GPIO				GPIOA
#define DRDY_PIN				GPIO_PIN_10
#define DRDY_IS_LOW()			((DRDY_GPIO->IDR & DRDY_PIN) == 0)
#define DRDY_IRQn 				EXTI15_10_IRQn
#define DRDY_IRQHandler			EXTI15_10_IRQHandler	
 
	/* PDWN  <------  PB7       掉电控制 */
#define PWDN_CLK_ENABLE() 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define PWDN_GPIO				GPIOB
#define PWDN_PIN				GPIO_PIN_4
#define PWDN_1()				PWDN_GPIO->BSRR = PWDN_PIN
#define PWDN_0()				PWDN_GPIO->BSRR = ((uint32_t)PWDN_PIN << 16U)			
	
	/*  RST   <------  PC3       复位信号	 */
#define RST_CLK_ENABLE() 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define RST_GPIO				GPIOB
#define RST_PIN					GPIO_PIN_5
#define RST_1()					RST_GPIO->BSRR = RST_PIN
#define RST_0()					RST_GPIO->BSRR = ((uint32_t)RST_PIN << 16U)		

/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2			= (1),	/* 增益2 */
	ADS1256_GAIN_4			= (2),	/* 增益4 */
	ADS1256_GAIN_8			= (3),	/* 增益8 */
	ADS1256_GAIN_16			= (4),	/* 增益16 */
	ADS1256_GAIN_32			= (5),	/* 增益32 */
	ADS1256_GAIN_64			= (6),	/* 增益64 */
}ADS1256_GAIN_E;

/* 采样速率选项 */
/* 数据转换率选择
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];			/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;			/* 当前通道 */
	uint8_t ScanMode;			/* 扫描模式，0表示单端8路， 1表示差分4路 */
}ADS1256_VAR_T;

void 	ADS1256_Init(void);
void 	ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);

uint8_t ADS1256_ReadChipID(void);
void 	ADS1256_StartScan(uint8_t _ucScanMode);
void 	ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);
void 	ADS1256_ISR(void);

extern 	ADS1256_VAR_T g_tADS1256;

#endif
