#include "ads1256.h"

/* 寄存器定义： Table 23. Register Map --- ADS1256数据手册第30页 */
enum
{
	/* 寄存器地址， 后面是复位后缺省值 */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* 命令定义： TTable 24. Command Definitions --- ADS1256数据手册第34页 */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

static void 	ADS1256_Send8Bit	(uint8_t _data);
static uint8_t 	ADS1256_Recive8Bit	(void);
static void 	ADS1256_WaitDRDY	(void);
static void	 	ADS1256_ResetHard	(void);
static void 	ADS1256_DelaySCLK	(void);
static void 	ADS1256_DelayDATA	(void);

static void 	ADS1256_WriteCmd	(uint8_t _cmd);
static void 	ADS1256_WriteReg	(uint8_t _RegID, uint8_t _RegValue);
static uint8_t 	ADS1256_ReadReg		(uint8_t _RegID);
static int32_t 	ADS1256_ReadData	(void);
static void 	ADS1256_SetChannal	(uint8_t _ch);

ADS1256_VAR_T g_tADS1256;

static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/* 复位时缺省值 */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

void bsp_DelayUS(uint16_t nus)
{
	delay_us(nus);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitADS1256
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 ADS1256
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	
	RST_1();
	PWDN_1();
	CS_1();
	SCK_0();		/* SPI总线空闲时，钟线是低电平 */
	DIN_1();
 
	/* 打开GPIO时钟 */
	SCK_CLK_ENABLE();
	DIN_CLK_ENABLE();
	CS_CLK_ENABLE();
	DOUT_CLK_ENABLE();
	DRDY_CLK_ENABLE();
	PWDN_CLK_ENABLE();
	RST_CLK_ENABLE();
 
	/* 配置几个推完输出IO */
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;		/* 设置推挽输出 */
	gpio_init.Pull = GPIO_NOPULL;				/* 上下拉电阻不使能 */
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;  	/* GPIO速度等级 */		
	
	gpio_init.Pin = SCK_PIN;	
	HAL_GPIO_Init(SCK_GPIO, &gpio_init);	
 
	gpio_init.Pin = DIN_PIN;	
	HAL_GPIO_Init(DIN_GPIO, &gpio_init);	
	
	gpio_init.Pin = CS_PIN;	
	HAL_GPIO_Init(CS_GPIO, &gpio_init);	
 
	gpio_init.Pin = PWDN_PIN;	
	HAL_GPIO_Init(PWDN_GPIO, &gpio_init);	
 
	/* DRDY 设置为输入 */
	gpio_init.Mode = GPIO_MODE_INPUT;			/* 设置输入 */
	gpio_init.Pull = GPIO_NOPULL;				/* 上下拉电阻不使能 */
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;  	/* GPIO速度等级 */
	
	gpio_init.Pin = DRDY_PIN;	
	HAL_GPIO_Init(DRDY_GPIO, &gpio_init);	
 
	gpio_init.Pin = DOUT_PIN;	
	HAL_GPIO_Init(DOUT_GPIO, &gpio_init);	
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_CfgADC
*	功能说明: 配置ADC参数，增益和数据输出速率
*	形    参: _gain : 支持增益参数。
*                    ADS1256_GAIN_1
*                    ADS1256_GAIN_2	
*                    ADS1256_GAIN_4
*                    ADS1256_GAIN_8
*                    ADS1256_GAIN_16
*                    ADS1256_GAIN_32
*                    ADS1256_GAIN_64
*
*			 _drate : 数据输出速率，不推荐超过1000SPS
*			 	   ADS1256_30000SPS
*			 	   ADS1256_15000SPS
*			 	   ADS1256_7500SPS
*			 	   ADS1256_3750SPS
*			 	   ADS1256_2000SPS
*			 	   ADS1256_1000SPS
*			 	   ADS1256_500SPS
*			 	   ADS1256_100SPS
*			 	   ADS1256_60SPS
*			 	   ADS1256_50SPS
*			 	   ADS1256_30SPS
*			 	   ADS1256_25SPS
*			 	   ADS1256_15SPS
*			 	   ADS1256_10SPS
*			 	   ADS1256_5SPS
*			 	   ADS1256_2d5SPS
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{	
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;
	
	ADS1256_StopScan();			/* 暂停CPU中断 */
	
	ADS1256_ResetHard();		/* 硬件复位 */
 
	ADS1256_WaitDRDY();
 
	{
		uint8_t buf[4];		/* 暂存ADS1256 寄存器配置参数，之后连续写4个寄存器 */
	
		buf[0] = (0 << 3) | (1 << 2) | (1 << 1);
		
		buf[1] = 0x08;	/* 高四位0表示AINP接 AIN0,  低四位8表示 AINN 固定接 AINCOM */
 
		buf[2] = (0 << 5) | (0 << 2) | (_gain << 1);
 
		/* 因为切换通道和读数据耗时 123uS, 因此扫描中断模式工作时，最大速率 = DRATE_1000SPS */
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	/* 选择数据输出速率 */
		
		CS_0();	/* SPI片选 = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* 写寄存器的命令, 并发送寄存器地址 */
		ADS1256_Send8Bit(0x03);			/* 寄存器个数 - 1, 此处3表示写4个寄存器 */
		
		ADS1256_Send8Bit(buf[0]);	/* 设置状态寄存器 */
		ADS1256_Send8Bit(buf[1]);	/* 设置输入通道参数 */
		ADS1256_Send8Bit(buf[2]);	/* 设置ADCON控制寄存器，增益 */
		ADS1256_Send8Bit(buf[3]);	/* 设置输出数据速率 */
		
		CS_1();	/* SPI片选 = 1 */		
	}
	bsp_DelayUS(50);	
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelaySCLK
*	功能说明: CLK之间的延迟，时序延迟.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelaySCLK(void)
{
	uint16_t i;

	for (i = 0; i < 15; i++);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_DelayDATA
*	功能说明: 读取DOUT之前的延迟
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		最小 50 个tCLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/* 最小延迟 6.5uS, 此处取10us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ResetHard
*	功能说明: 硬件复位 ADS1256芯片.低电平复位。最快4个时钟，也就是 4x0.13uS = 0.52uS
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_ResetHard(void)
{
	/* ADS1256数据手册第7页 */
	//RESET_0();			/* 复位 */
	//bsp_DelayUS(5);
	//RESET_1();

	//PWDN_0();			/* 进入掉电 同步*/
	//bsp_DelayUS(2);
	//PWDN_1();			/* 退出掉电 */

	bsp_DelayUS(5);

	//ADS1256_WaitDRDY();	/* 等待 DRDY变为0, 此过程实测: 630us */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_Send8Bit
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{
	uint8_t i;
 
	/* 连续发送多个字节时，需要延迟一下 */
	ADS1256_DelaySCLK();
	ADS1256_DelaySCLK();
 
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for(i = 0; i < 8; i++)
	{
		if (_data & 0x80)
		{
			DIN_1();
		}
		else
		{
			DIN_0();
		}
		SCK_1();				
		ADS1256_DelaySCLK();		
		_data <<= 1;		
		SCK_0();			/* <----  ADS1256 是在SCK下降沿采样DIN数据, 数据必须维持 50nS */
		ADS1256_DelaySCLK();		
	}
}
 
/*
*********************************************************************************************************
*	函 数 名: ADS1256_Recive8Bit
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t i;
	uint8_t read = 0;
 
	ADS1256_DelaySCLK();
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCK_1();
		ADS1256_DelaySCLK();
		read = read<<1;
		SCK_0();
		if (DOUT_IS_HIGH())
		{
			read++;
		}		
		ADS1256_DelaySCLK();
	}
	return read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadData
*	功能说明: 读ADC数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
 
	CS_0();	/* SPI片选 = 0 */
 
	ADS1256_Send8Bit(CMD_RDATA);	/* 读数据的命令 */
	
	ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */
 
	/* 读采样结果，3个字节，高字节在前 */
	read =  ADS1256_Recive8Bit() << 16;
	read += ADS1256_Recive8Bit() << 8;
	read += ADS1256_Recive8Bit() << 0;
 
	CS_1();	/* SPI片选 = 1 */
	
	/* 负数进行扩展。24位有符号数扩展为32位有符号数 */
	if (read & 0x800000)
	{
		read += 0xFF000000;
	}
	
	return (int32_t)read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WriteReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);		/* 寄存器个数 - 1, 此处写1个寄存器 */

	ADS1256_Send8Bit(_RegValue);	/* 发送寄存器值 */
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadReg
*	功能说明: 写指定的寄存器
*	形    参:  _RegID : 寄存器ID
*			  _RegValue : 寄存器值。
*	返 回 值: 读到的寄存器值。
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* 写寄存器的命令, 并发送寄存器地址 */
	ADS1256_Send8Bit(0x00);	/* 寄存器个数 - 1, 此处读1个寄存器 */

	ADS1256_DelayDATA();	/* 必须延迟才能读取芯片返回数据 */

	read = ADS1256_Recive8Bit();	/* 读寄存器值 */
	CS_1();	/* SPI片选 = 1 */

	return read;
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WriteCmd
*	功能说明: 发送单字节命令
*	形    参:  _cmd : 命令
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI片选 = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI片选 = 1 */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_ReadChipID
*	功能说明: 读芯片ID, 读状态寄存器中的高4bit
*	形    参: 无
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetChannal
*	功能说明: 配置通道号。多路复用。AIN- 固定接地（ACOM).
*	形    参: _ch : 通道号， 0-7
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN 固定接 AINCOM */
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_SetDiffChannal
*	功能说明: 配置差分通道号。多路复用。
*	形    参: _ch : 通道号,0-3；共4对
*	返 回 值: 8bit状态寄存器值的高4位
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are “don’t care”)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are “don’t care”)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* 差分输入 AIN0， AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/* 差分输入 AIN2， AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/* 差分输入 AIN4， AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/* 差分输入 AIN6， AIN7 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_WaitDRDY
*	功能说明: 等待内部操作完成。 自校准时间较长，需要等待。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t i;

	for (i = 0; i < 40000000; i++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (i >= 40000000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");		/* 调试语句. 用语排错 */
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StartScan
*	功能说明: 将 DRDY引脚 （PA10 ）配置成外部中断触发方式， 中断服务程序中扫描8个通道的数据。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode)
{
	/* PC6 外部中断，BUSY 
		配置 BUSY 作为中断输入口，下降沿触发 */
	{
		GPIO_InitTypeDef   GPIO_InitStructure;
		
		DRDY_CLK_ENABLE();	/* 打开GPIO时钟 */
 
		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = DRDY_PIN;
		HAL_GPIO_Init(DRDY_GPIO, &GPIO_InitStructure);	
 
		HAL_NVIC_SetPriority(DRDY_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(DRDY_IRQn);	
	}
	g_tADS1256.ScanMode = _ucScanMode;
	/* 开始扫描前, 清零结果缓冲区 */	
	{
		uint8_t i;
		
		g_tADS1256.Channel = 0;
		
		for (i = 0; i < 8; i++)
		{
			g_tADS1256.AdcNow[i] = 0;
		}	
	}
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_StopScan
*	功能说明: 停止 DRDY 中断
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_StopScan(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/*
*********************************************************************************************************
*	函 数 名: ADS1256_GetAdc
*	功能说明: 从缓冲区读取ADC采样结果。采样结构是由中断服务程序填充的。
*	形    参: _ch 通道号 (0 - 7)
*	返 回 值: ADC采集结果（有符号数）
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	__set_PRIMASK(1);	/* 禁止中断 */

	iTemp = g_tADS1256.AdcNow[_ch];

	__set_PRIMASK(0);	/* 使能中断 */

	return iTemp;
}

/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序.  此程序执行时间约 123uS
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
#ifdef EXTI9_5_ISR_MOVE_OUT		/* bsp.h 中定义此行，表示本函数移到 stam32f4xx_it.c。 避免重复定义 */
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}
 
/*
*********************************************************************************************************
*	函 数 名: EXTI9_5_IRQHandler
*	功能说明: 外部中断服务程序入口。PI6 / AD7606_BUSY 下降沿中断触发
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6)
	{
 
		ADS1256_ISR();
	}
}
#endif
 
/*
*********************************************************************************************************
*	函 数 名: ADS1256_ISR
*	功能说明: 定时采集中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	/* 读取采集结构，保存在全局变量 */					
	ADS1256_SetChannal(g_tADS1256.Channel);	/* 切换模拟通道 */	
	bsp_DelayUS(5);
	
	ADS1256_WriteCmd(CMD_SYNC);
	bsp_DelayUS(5);
	
	ADS1256_WriteCmd(CMD_WAKEUP);
	bsp_DelayUS(25);
	
	if (g_tADS1256.Channel == 0)
	{
		g_tADS1256.AdcNow[7] = ADS1256_ReadData();	/* 注意保存的是上一个通道的数据 */
	}
	else
	{
		g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData(); /* 注意保存的是上一个通道的数据 */
	}
				
	if (++g_tADS1256.Channel >= 8)
	{
		g_tADS1256.Channel = 0;
	}
}

