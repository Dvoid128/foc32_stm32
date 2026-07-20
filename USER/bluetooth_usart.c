#include "bluetooth_usart.h"

#if 0
volatile u8 bluetooth_rx_data = 0;
volatile u8 bluetooth_rx_flag = 0;
volatile u16 bluetooth_rx_count = 0;

static volatile u8 bluetooth_rx_buf[BLUETOOTH_RX_BUF_SIZE];
static volatile u8 bluetooth_rx_write = 0;
static volatile u8 bluetooth_rx_read = 0;

u8 show_data = 0;
u8 PID_Send = 0;

void Bluetooth_USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void Bluetooth_SendByte(u8 data)
{
	USART_SendData(USART2, data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Bluetooth_SendString(char *str)
{
	while(*str)
	{
		Bluetooth_SendByte((u8)*str++);
	}
}

u8 Bluetooth_Available(void)
{
	return bluetooth_rx_read != bluetooth_rx_write;
}

u8 Bluetooth_ReadByte(void)
{
	u8 data = 0;

	if(bluetooth_rx_read != bluetooth_rx_write)
	{
		data = bluetooth_rx_buf[bluetooth_rx_read];
		bluetooth_rx_read++;
		if(bluetooth_rx_read >= BLUETOOTH_RX_BUF_SIZE) bluetooth_rx_read = 0;
	}

	if(bluetooth_rx_read == bluetooth_rx_write) bluetooth_rx_flag = 0;
	return data;
}

void Bluetooth_ClearRxBuffer(void)
{
	bluetooth_rx_read = 0;
	bluetooth_rx_write = 0;
	bluetooth_rx_flag = 0;
}

static void Bluetooth_SaveRxByte(u8 data)
{
	u8 next_write = bluetooth_rx_write + 1;

	if(next_write >= BLUETOOTH_RX_BUF_SIZE) next_write = 0;
	if(next_write != bluetooth_rx_read)
	{
		bluetooth_rx_buf[bluetooth_rx_write] = data;
		bluetooth_rx_write = next_write;
		bluetooth_rx_flag = 1;
		bluetooth_rx_count++;
	}
}

static int Bluetooth_ParsePositiveInt(u8 *buf, u8 start, u8 end)
{
	int value = 0;

	while(start < end)
	{
		if(buf[start] < '0' || buf[start] > '9') break;
		value = value * 10 + buf[start] - '0';
		start++;
	}

	return value;
}

static void Bluetooth_SetDebugParam(u8 id, int value)
{
	switch(id)
	{
		case 0x30: Velocity_KP = (float)value; break;
		case 0x31: Velocity_KI = (float)value; break;
		case 0x32: BaseSpeed = (float)value; break;
		case 0x33: Turn90Angle = (float)value; break;
		case 0x34: TurnMaxAngle = (float)value; break;
		case 0x35: TurnMidAngle = (float)value; break;
		case 0x36: TurnMinAngle = (float)value; break;
		case 0x37: ForwardLimit = (float)value; break;
		case 0x38: break;
		default: break;
	}
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		static u8 Flag_PID = 0;
		static u8 i = 0;
		static u8 Receive[50];
		u8 data;
		int parsed_data;

		data = (u8)USART_ReceiveData(USART2);
		bluetooth_rx_data = data;
		Bluetooth_SaveRxByte(data);

		if(data == 0x7B) Flag_PID = 1;
		if(data == 0x7D) Flag_PID = 2;

		if(Flag_PID == 1)
		{
			if(i < sizeof(Receive))
			{
				Receive[i] = data;
				i++;
			}
			else
			{
				Flag_PID = 0;
				i = 0;
				memset(Receive, 0, sizeof(Receive));
			}
		}

		if(Flag_PID == 2)
		{
			if(i > 3 && Receive[3] == 0x50)
			{
				PID_Send = 1;
			}
			else if(i > 3 && Receive[1] != 0x23)
			{
				parsed_data = Bluetooth_ParsePositiveInt(Receive, 3, i);
				Bluetooth_SetDebugParam(Receive[1], parsed_data);
			}

			Flag_PID = 0;
			i = 0;
			memset(Receive, 0, sizeof(Receive));
		}
	}
}

static int Bluetooth_AbsInt(int value)
{
	return value < 0 ? -value : value;
}

void APP_Show(void)
{
	static u8 flag;
	int Encoder_Left_Show;
	int Encoder_Right_Show;
	int Voltage_Show;

	Voltage_Show = 0;
	Encoder_Right_Show = Bluetooth_AbsInt((int)(MotorB.Current_Encoder * 1000.0f));
	Encoder_Left_Show = Bluetooth_AbsInt((int)(MotorA.Current_Encoder * 1000.0f));

	flag = !flag;
	if(PID_Send == 1)
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
		       (int)Velocity_KP,
		       (int)Velocity_KI,
		       (int)BaseSpeed,
		       (int)Turn90Angle,
		       (int)TurnMaxAngle,
		       (int)TurnMidAngle,
		       (int)TurnMinAngle,
		       (int)ForwardLimit,
		       0);
		PID_Send = 0;
	}
	else if(flag == 0)
	{
		printf("{A%d:%d:%d}$", Encoder_Left_Show, Encoder_Right_Show, Voltage_Show);
	}
	else
	{
		printf("{B%d:%d:%d}$", 0, 0, 0);
	}
}
#endif

#define UART2_FRAME_LEN   8
#define UART2_HEAD        0x7B
#define UART2_END         0x7D

volatile u8 bluetooth_rx_data = 0;
volatile u8 bluetooth_rx_flag = 0;
volatile u16 bluetooth_rx_count = 0;
u8 show_data = 0;
u8 PID_Send = 0;

volatile int16_t g_follow_x = 0;
volatile int16_t g_follow_y = 0;
volatile u16 g_follow_area = 0;
volatile u8 g_data_ready = 0;
volatile u8 g_vision_frame_flag = 0;
volatile u16 g_vision_frame_count = 0;
u8 Packet[UART2_FRAME_LEN];

static void USART2_DMA_Restart(void)
{
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel6, UART2_FRAME_LEN);
	DMA_Cmd(DMA1_Channel6, ENABLE);
}

static void USART2_ParsePacket(void)
{
	u16 cx_u;
	u16 cy_u;
	u16 area_u;

	if(Packet[0] != UART2_HEAD || Packet[7] != UART2_END)
	{
		return;
	}

	cx_u = ((u16)Packet[1] << 8) | Packet[2];
	cy_u = ((u16)Packet[3] << 8) | Packet[4];
	area_u = ((u16)Packet[5] << 8) | Packet[6];

	g_follow_x = (int16_t)(cx_u - 32768);
	g_follow_y = (int16_t)(cy_u - 32768);
	g_follow_area = area_u * 10;
	g_data_ready = 1;
	g_vision_frame_flag = 1;
	g_vision_frame_count++;
	bluetooth_rx_flag = 1;
	bluetooth_rx_count++;
}

void Bluetooth_USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Packet;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = UART2_FRAME_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART2_DMA_Restart();
}

void DMA1_Channel6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC6) != RESET)
	{
		DMA_ClearITPendingBit(DMA1_IT_TC6);
		DMA_Cmd(DMA1_Channel6, DISABLE);
		USART2_ParsePacket();
		USART2_DMA_Restart();
	}
}

void USART2_IRQHandler(void)
{
	u16 dummy;
	u16 dma_left;

	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		dummy = USART2->SR;
		dummy = USART2->DR;
		(void)dummy;

		dma_left = DMA_GetCurrDataCounter(DMA1_Channel6);
		if(dma_left != 0 && dma_left != UART2_FRAME_LEN)
		{
			USART2_DMA_Restart();
		}
	}

	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		bluetooth_rx_data = USART_ReceiveData(USART2);
		show_data = bluetooth_rx_data;
		USART2_DMA_Restart();
	}
}

void Bluetooth_SendByte(u8 data)
{
	(void)data;
}

void Bluetooth_SendString(char *str)
{
	(void)str;
}

u8 Bluetooth_Available(void)
{
	return g_data_ready;
}

u8 Bluetooth_ReadByte(void)
{
	g_data_ready = 0;
	return 0;
}

void Bluetooth_ClearRxBuffer(void)
{
	g_data_ready = 0;
	bluetooth_rx_flag = 0;
	USART2_DMA_Restart();
}

void APP_Show(void)
{
}
