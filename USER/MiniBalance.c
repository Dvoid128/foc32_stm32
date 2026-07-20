#include "stm32f10x.h"
#include "sys.h"
#include "ir_track.h"
#include "h30_imu.h"
#include "control.h"
#include "bluetooth_usart.h"
#include <string.h>


/* OLED 主循环刷新节拍预留值，目前显示函数在 while(1) 中直接刷新。 */
#define SAMPLE_TICKS   5

/* 显示辅助函数：整数取绝对值，用于带符号数字显示。 */
static int abs_int(int value)
{
	return value < 0 ? -value : value;
}

/* 将控制中的 m/s 速度换算成 mm/s，方便 OLED 上显示整数。 */
static int speed_to_mmps(float value)
{
	if(value >= 0.0f) return (int)(value * 1000.0f + 0.5f);
	else              return (int)(value * 1000.0f - 0.5f);
}

/* 将浮点数放大 10 倍并四舍五入，用于显示一位小数。 */
static int float_to_deci(float value)
{
	if(value >= 0.0f) return (int)(value * 10.0f + 0.5f);
	else              return (int)(value * 10.0f - 0.5f);
}

/* 在 OLED 指定位置显示带正负号的整数。 */
static void OLED_ShowSignedInt(u8 x, u8 y, int value, u8 len)
{
	if(value < 0) OLED_ShowString(x, y, "-");
	else          OLED_ShowString(x, y, "+");

	OLED_ShowNumber(x + 8, y, abs_int(value), len, 12);
}

/* 在 OLED 指定位置显示带正负号的一位小数。 */
static void OLED_ShowSignedDeci(u8 x, u8 y, float value, u8 len)
{
	int deci_value = float_to_deci(value);
	int deci_abs = abs_int(deci_value);

	if(deci_value < 0) OLED_ShowString(x, y, "-");
	else               OLED_ShowString(x, y, "+");

	OLED_ShowNumber(x + 8, y, deci_abs / 10, len, 12);
	OLED_ShowString(x + 8 + len * 8, y, ".");
	OLED_ShowNumber(x + 16 + len * 8, y, deci_abs % 10, 1, 12);
}
extern u8 show_data;

/* OLED 调试界面：红外、模式启停、电机闭环、视觉坐标、帧率和陀螺仪 Z 轴。 */
static void OLED_ShowMotorData(void)
{
	memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
	IR_Track_Update();

	OLED_ShowString(0, 0, "IR");
	OLED_ShowNumber(24, 0, ir_dh1_state, 1, 12);
	OLED_ShowNumber(32, 0, ir_dh2_state, 1, 12);
	OLED_ShowNumber(40, 0, ir_dh3_state, 1, 12);
	OLED_ShowNumber(48, 0, ir_dh4_state, 1, 12);
	OLED_ShowString(56, 0, "M");
	if(Control_Work_Mode == CONTROL_MODE_RUN) OLED_ShowString(64, 0, "RUN");
	else                                      OLED_ShowString(64, 0, "SEA");
	OLED_ShowString(96, 0, "E");
	OLED_ShowNumber(104, 0, Control_Work_Enable, 1, 12);

	OLED_ShowString(0, 12, "L");
	OLED_ShowString(12, 12, "T");
	OLED_ShowSignedInt(24, 12, speed_to_mmps(MotorA.Target_Encoder), 3);
	OLED_ShowString(64, 12, "A");
	OLED_ShowSignedInt(76, 12, speed_to_mmps(MotorA.Current_Encoder), 3);

	OLED_ShowString(0, 24, "R");
	OLED_ShowString(12, 24, "T");
	OLED_ShowSignedInt(24, 24, speed_to_mmps(MotorB.Target_Encoder), 3);
	OLED_ShowString(64, 24, "A");
	OLED_ShowSignedInt(76, 24, speed_to_mmps(MotorB.Current_Encoder), 3);

	OLED_ShowString(0, 36, "X");
	OLED_ShowSignedInt(8, 36, g_follow_x, 5);
	OLED_ShowString(64, 36, "Y");
	OLED_ShowSignedInt(72, 36, g_follow_y, 5);

	OLED_ShowString(0, 48, "F");
	OLED_ShowNumber(8, 48, Vision_Fps, 3, 12);
	OLED_ShowString(40, 48, "GZ");
	OLED_ShowSignedDeci(56, 48, h30_imu_data.gyro_z, 3);
	OLED_Refresh_Gram();
}

int main(void)
{
	/* 基础系统初始化：中断优先级、调试接口、系统滴答、OLED、按键和示波器串口。 */
	MY_NVIC_PriorityGroupConfig(2);
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);
    //SysTick init
	SysTick_Init(1000);
	delay_ms(500);
	OLED_Init();
	OLED_Clear();
	KEY_Init();
	DataScope_DP_Init();
	delay_us(500);
	/* 配置前先关闭两个云台电机，防止上电瞬间沿用旧状态动作。 */
	BLDC_Disable(BLDC_ADDR_2);
	delay_us(500);
	BLDC_Disable(BLDC_ADDR_1);
    //Acceleration
	delay_us(500);
	/* 设置云台电机加速度，数值越小动作越柔和，越大响应越快。 */
	BLDC_SetAcc(BLDC_ADDR_1,100);
	delay_us(500);
	BLDC_SetAcc(BLDC_ADDR_2,100);
	delay_us(500);
    //Gimbal mode
	/* 一轴使用速度模式控制偏航，二轴使用单圈角度模式控制俯仰位置。 */
    BLDC_SetMode(BLDC_ADDR_1,MODE_SPEED);//speed
	delay_us(500);
	BLDC_SetSpeed(BLDC_ADDR_1,0);
	delay_us(500);
    BLDC_SetMode(BLDC_ADDR_2,MODE_SINGLE_POS_L);//single-angle
	delay_us(500);
	BLDC_SetSpeed(BLDC_ADDR_2,10);
	delay_us(500);
	BLDC_SetSingleAngle(BLDC_ADDR_2,0);
	delay_us(500);
	BLDC_Enable(BLDC_ADDR_1);
	delay_us(500);
	BLDC_Enable(BLDC_ADDR_2);
	delay_us(500);
	/* 运动控制相关外设初始化：红外巡线、蓝牙、PWM、编码器和 IMU。 */
	IR_Track_Init();
	Bluetooth_USART2_Init();
	MiniBalance_PWM_Init(7199, 0);
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	H30_IMU_Init();
	OLED_Clear();
	while(1)
	{
		/* 主循环只负责刷新调试显示，实时控制在 TIM2 中断中执行。 */
		OLED_ShowMotorData();
	}
}



