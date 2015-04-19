#include "stm32f10x.h"

/************************************
函数名：void SPI_Configuration(void)
说明：SPI配置
入口：无
出口：无
备注：配置SPI2
************************************/
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 			// 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;								// 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;							// 设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;									// 串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;								// 串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;									// NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;			// 定义波特率预分频的值:波特率预分频值为2，36MHz/2/4=4.5MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;							// 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;									// CRC值计算的多项式，在全双工模式中CRC作为最后一个字节发送出去
	SPI_Init(SPI2, &SPI_InitStructure);  										// 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI2, ENABLE); 														// 使能SPI外设
	
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);  				// PB13/14/15上拉
}

/***************************************
函数名：u8 SPI_ReadWriteByte(u8 txData)
说明：SPI读写一个字节函数
入口：u8 txData MOSI写出的字节
出口：u8	MISO写入的字节
备注：返回值为0时，表明没有发送/接收成功
****************************************/
u8 SPI_ReadWriteByte(u8 txData)
{		
	u8 retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) // 检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if (retry > 200)	// 超时
			return 0;
	}			  
	SPI_I2S_SendData(SPI2, txData); 								// 通过外设SPIx发送一个数据
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) // 检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry > 200)		// 超时
			return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); 								// 返回通过SPIx最近接收的数据					    
}
