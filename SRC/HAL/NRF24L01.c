#include "stm32f10x.h"
#include "NRF24L01.H"
#include "SPI.H"

//-----NRF24L01发送接收地址-----//
const u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; // 发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01}; // 接收地址

//-----NRF24L01接收数据-----//
u8 g_NRFRevData[RX_PLOAD_WIDTH] = {0,0};
//-----NRF24L01发送数据-----//
u8 g_NRFSendData[TX_PLOAD_WIDTH] = {0};

//-----NRF24L01接收计数器-----//
__IO u8 g_NRFRevCnt = 0;

/********************************
函数名：void NRF24L01_Init(void)
说明：初始化24L01
入口：无
出口：无
备注：无
*********************************/
void NRF24L01_Init(void)
{
	NRF24L01_CE_L; 		// 使能24L01
	NRF24L01_CSN_H;		// SPI片选取消  
}

/******************************
函数名：u8 NRF24L01_Check(void)
说明：检测24L01是否存在
入口：无
出口：u8
				0，成功;
				1，失败	
备注：无
*******************************/
u8 NRF24L01_Check(void)
{
	u8 buf[5] = {0XA5,0XA5,0XA5,0XA5,0XA5}, i;

	NRF24L01_Write_Buf(WRITE_REG_NRF + TX_ADDR,buf, 5);		// 写入5个字节的地址，TX_ADDR为地址，WRITE_REG_NRF为命令
	NRF24L01_Read_Buf(TX_ADDR,buf,5); 						// 读出写入的地址  
	for (i = 0; i < 5; i++)
		if (buf[i] != 0XA5)
			break;
	if (i != 5)
		return 1;		// 检测24L01错误	
	return 0;		 	// 检测到24L01
}

/************************************************
函数名：void NRF24L01_Write_Reg(u8 reg, u8 value)
说明：SPI写寄存器
入口：	u8 reg		寄存器地址
		u8 value	需要写入的值
出口：无
备注：无
************************************************/
void NRF24L01_Write_Reg(u8 reg, u8 value)
{
	NRF24L01_CSN_L;                 // 使能SPI传输
	SPI_ReadWriteByte(reg);			// 发送寄存器号，这里可以读取到寄存器的状态
	SPI_ReadWriteByte(value);      	// 写入寄存器的值
	NRF24L01_CSN_H;                 // 禁止SPI传输	   
}

/************************************
函数名：u8 NRF24L01_Read_Reg(u8 reg)
说明：读取SPI寄存器值
入口：u8 reg	寄存器地址
出口：u8		状态
备注：无
*************************************/
//
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;
	
 	NRF24L01_CSN_L;          				// 使能SPI传输		
	SPI_ReadWriteByte(reg);   				// 发送寄存器号
	reg_val = SPI_ReadWriteByte(0XFF);		// 读取寄存器内容，只需要读取，主机可以随意传送数据过去
	NRF24L01_CSN_H;          				// 禁止SPI传输		    
	return(reg_val);           				// 返回状态值
}

/*******************************************************
函数名：void NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
说明：在指定位置读出指定长度的数据
入口：	u8 reg		寄存器(位置)
		u8 *pBuf	数据指针
		u8 len		数据长度
出口：无
备注：无
*******************************************************/
void NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 i;
	
	NRF24L01_CSN_L;           					// 使能SPI传输
	SPI_ReadWriteByte(reg);						// 发送寄存器值(位置)  	   
	for (i = 0; i < len; i++)
		pBuf[i] = SPI_ReadWriteByte(0XFF);		// 读出数据
	NRF24L01_CSN_H;       						// 关闭SPI传输
}

/********************************************************
函数名：void NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
说明：在指定位置写指定长度的数据
入口：	u8 reg		寄存器(位置)
		u8 *pBuf	数据指针
		u8 len		数据长度
出口：无
备注：无
********************************************************/
void NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 i;
	
 	NRF24L01_CSN_L;          			// 使能SPI传输
	SPI_ReadWriteByte(reg);				// 发送寄存器值(位置)
	for	(i = 0; i < len; i++)
		SPI_ReadWriteByte(*pBuf++); 	// 写入数据	 
	NRF24L01_CSN_H;       				// 关闭SPI传输
}

/***************************************
函数名：u8 NRF24L01_TxPacket(u8 *txbuf)
说明：启动NRF24L01发送一次数据
入口：u8 *txbuf	待发送数据首地址
出口：U8		发送完成状况
备注：无
****************************************/
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	
	NRF24L01_CE_L;											// 片选
  NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	// 写数据到TX BUF  32个字节
 	NRF24L01_CE_H;
	while (NRF24L01_IRQ != 0);								// 等待发送完成
	sta = NRF24L01_Read_Reg(STATUS);  						// 读取状态寄存器的值
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta); 		// 清除TX_DS或MAX_RT中断标志
	if (sta & MAX_TX)										// 达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xFF);					// 清除TX FIFO寄存器 
		return MAX_TX;
	}
	if (sta & TX_OK)										// 发送完成
	{
		return TX_OK;
	}
	return 0xFF;											// 其他原因发送失败
}

/***************************************
函数名：u8 NRF24L01_RxPacket(u8 *rxbuf)
说明：启动NRF24L01接收一次数据
入口：u8 *txbuf	待接收数据首地址
出口：u8	0：	接收完成
			1：	接收不成功
备注：无
****************************************/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							    
	sta = NRF24L01_Read_Reg(STATUS);  								// 读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS,sta); 				// 清除TX_DS或MAX_RT中断标志
	if (sta & RX_OK)												// 接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);		// 读取数据
		NRF24L01_Write_Reg(FLUSH_RX, 0xFF);							// 清除RX FIFO寄存器 
		return 0; 
	}
	return 1;														// 没收到任何数据
}

/*****************************************************
函数名：void NRF24L01_RX_Mode(void)
说明：初始化NRF24L01到RX模式
入口：无
出口：无
备注：
设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
当CE变高后,即进入RX模式,并可以接收数据了
******************************************************/
void NRF24L01_RX_Mode(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;											// 定义EXTI结构体
	// 由于发送模式下已经将中断线切断，所以当切换成接收模式时，需要重新连接起来
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;										// EXTI线中断开通
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;								// 定义为中断还是事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;							// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;										// 使能EXTI线中断
	EXTI_Init(&EXTI_InitStructure);
	
	NRF24L01_CE_L;
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);			// 写TX节点地址 
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);		// 写RX节点地址
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);    								// 使能通道0的自动应答    
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01);								// 使能通道0的接收地址  	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,40);	    								// 设置RF通信频率		  
	NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);						// 选择通道0的有效数据宽度 	    
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);								// 设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(WRITE_REG_NRF+NRF24L01_CONFIG, 0x0f);						// 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF24L01_CE_H; 																	// CE为高,进入接收模式 
}

/*****************************************************************************************
函数名：void NRF24L01_TX_Mode(void)
说明：初始化NRF24L01到TX模式
入口：无
出口：无
备注：
设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
PWR_UP,CRC使能
CE为高大于10us,则启动发送
******************************************************************************************/
void NRF24L01_TX_Mode(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;											// 定义EXTI结构体
	// 由于接收模式下的IRQ引脚设置为中断输入引脚，所以当切换成发送模式时，需要将中断输入引脚的功能取消
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;										// EXTI线中断开通
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;								// 定义为中断还是事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;							// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;										// 失能EXTI线中断
	EXTI_Init(&EXTI_InitStructure);
	
	NRF24L01_CE_L;	    
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);			// 写TX节点地址 
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 		// 设置TX节点地址,主要为了使能ACK
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     								// 使能通道0自动应答    
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); 								// 使能通道0的接收地址  
	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0x1a);								// 设置自动重发间隔时间:500us + 86us;最大自动重发次数:15次
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,40);       								// 设置RF通道为40
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  								// 设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(WRITE_REG_NRF+NRF24L01_CONFIG,0x0e);    						// 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发送模式,开启所有中断
	NRF24L01_CE_H;																	// CE为高,10us后启动发送
}		  
