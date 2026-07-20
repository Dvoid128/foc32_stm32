#include "control.h"
#include "ir_track.h"
#include "bluetooth_usart.h"

/* 电机闭环输出和工作状态：模式选择与启动使能分开管理。 */
int Motor_Left = 0;
int Motor_Right = 0;
u8 Control_Work_Mode = CONTROL_MODE_SEARCH;
u8 Control_Work_Enable = 1;
static Encoder_Original OriginalEncoder;
volatile Motor_Parameter MotorA;
volatile Motor_Parameter MotorB;

/* 速度环 PI 参数和云台 IMU 偏航补偿系数。 */
float Velocity_KP = 400.0f;
float Velocity_KI = 400.0f;
float Gimbal_Yaw_Rpm_K = -1.0f / 6.0f;
volatile int16_t Gimbal_Joint1_Target_Rpm = 0;
volatile int16_t Gimbal_Joint2_Target_Rpm = 0;
volatile uint16_t Gimbal_Joint2_Target_Angle = 0;
volatile u16 Vision_Fps = 0;
volatile u8 Vision_Online = 0;
/* 视觉误差方向、死区、限幅以及可调参数步进。 */
#define VISION_DIR_X -1.0f				  /* 图像 X 误差到偏航输出的方向符号，方向反时只改这里。 */
#define VISION_DIR_Y -1.0f				  /* 图像 Y 误差到俯仰输出的方向符号，方向反时只改这里。 */
#define VISION_DEADZONE 3.0f			  /* 目标中心附近的像素死区，用来抑制小幅抖动。 */
#define VISION_LOST_TICKS 40			  /* 视觉连续丢帧计数阈值，超过后清空视觉 PD 输出。 */
#define VISION_YAW_LIMIT_RPM 30.0f		  /* 视觉偏航 PD 输出限幅，尚未叠加 IMU 补偿。 */
#define VISION_PITCH_LIMIT_RPM 10.0f	  /* 视觉俯仰 PD 输出限幅。 */
#define VISION_PITCH_STEP_LIMIT_X10 10.0f /* 俯仰角度每周期累加步进限幅，写入目标角前先限制。 */
#define GIMBAL_YAW_COMP_LIMIT_RPM 35.0f	  /* IMU z 轴角速度折算偏航补偿的限幅。 */
#define GIMBAL_YAW_LIMIT_RPM 100.0f		  /* 下发到云台一轴的最终偏航速度限幅。 */
#define VISION_FPS_SAMPLE_TICKS 200		  /* 帧率统计窗口，200 个 5ms 周期约为 1 秒。 */

/* 当前视觉瞄准中心点固定为视觉模块坐标原点，先去掉路线标定补偿。 */
int VISION_CENTER_X = 0;
int VISION_CENTER_Y = 0;

/*
 * 视觉 PD 调参说明：
 * X 轴对应云台偏航，主要负责让目标左右居中；Y 轴对应云台俯仰，主要负责让目标上下居中。
 * Kp 决定追踪力度，值大响应快但容易抖动或过冲，值小则跟踪慢、目标回中心不够积极。
 * Kd 决定阻尼，主要抑制快速变化和过冲，值大可能动作发涩或放大噪声，值小则刹不住。
 */
float KP_VISION_X = 0.7f;  /* X 轴比例系数：调左右偏航追踪力度，左右跟不住先加它，抖动明显就减小。 */
float KD_VISION_X = 0.02f; /* X 轴微分系数：给左右偏航加阻尼，转动过冲/来回摆时适当加大。 */
float KP_VISION_Y = 0.25f; /* Y 轴比例系数：调上下俯仰追踪力度，上下回中心慢时加大。 */
float KD_VISION_Y = 0.02f; /* Y 轴微分系数：给上下俯仰加阻尼，俯仰过冲或抖动时微调。 */

/*
 * 不同路段视觉系数：
 * 这些系数会作为 vision_segment_gain 参与视觉 PD 输出，用来适配不同路线段的车速、角度和目标偏移。
 * 某段目标跟踪偏软、偏慢时增大对应系数；某段云台抖动、抢得太猛或路线变化大时减小。
 * CD_VISION_FF 是 CD 段速度前馈系数，不是 PD 增益，主要用来按车速提前补偿横向偏移。
 */
float AC_VISION_K = 0.40f; /* AC 段视觉整体增益：只影响 AC 段，常用于单独压住或增强 AC 段跟踪。 */
float CD_VISION_K = 0.40f; /* CD 段视觉整体增益：只影响 CD 段，配合 CD_VISION_FF 一起调。 */
float DB_VISION_K = 0.40f; /* DB 段视觉整体增益：只影响 DB 段，方便最后一段单独修正。 */
float BA_VISION_FF = 0.05f;
float CD_VISION_FF = 0.045f; /* CD 段偏航速度前馈：车越快补偿越大，CD 段横向总偏一边时重点调它。 */

/* Visual PD state. Search mode locks after the first valid target. */
static u16 vision_lost_ticks = VISION_LOST_TICKS + 1;
static u8 vision_prev_valid = 0;
static float prev_err_x = 0.0f;
static float prev_err_y = 0.0f;
static float vision_yaw_cmd = 0.0f;
static float vision_pitch_cmd = 0.0f;
static float vision_ff_yaw_cmd = 0.0f;
static float vision_segment_gain = 1.0f;
static u16 search_scan_ticks = 0;
static u8 search_target_locked = 0;

/* 小工具函数：统一做取绝对值、限幅和浮点四舍五入，避免控制输出越界。 */
static int abs_int(int value)
{
	return value < 0 ? -value : value;
}
static int limit_int(int value, int max, int min)
{
	if (value > max)
		return max;
	if (value < min)
		return min;
	return value;
}
static float limit_float(float value, float max, float min)
{
	if (value > max)
		return max;
	if (value < min)
		return min;
	return value;
}
static int float_to_int_round(float value)
{
	if (value >= 0.0f)
		return (int)(value + 0.5f);
	else
		return (int)(value - 0.5f);
}
static float abs_float(float value)
{
	return value < 0.0f ? -value : value;
}

/* 视觉接收频率在 5ms 的 TIM2 循环中统计。 */
static void Vision_Frequency_Update(void)
{
	static u16 sample_ticks = 0;
	static u16 last_frame_count = 0;
	u16 current_frame_count;
	u16 frame_delta;

	if (g_vision_frame_flag)
	{
		/* 已消费一次视觉新帧标志，清零等待下一帧。 */
		g_vision_frame_flag = 0;
	}

	sample_ticks++;
	if (sample_ticks >= VISION_FPS_SAMPLE_TICKS)
	{
		current_frame_count = g_vision_frame_count;
		if (current_frame_count >= last_frame_count)
		{
			frame_delta = current_frame_count - last_frame_count;
		}
		else
		{
			frame_delta = (u16)(0xffff - last_frame_count + current_frame_count + 1);
		}
		Vision_Fps = frame_delta;
		Vision_Online = (frame_delta > 0) ? 1 : 0;
		last_frame_count = current_frame_count;
		sample_ticks = 0;
	}
}

/* 用 IMU 的 z 轴角速度抵消车体偏航对云台的影响。 */
static float Gimbal_GetYawCompRpm(void)
{
	float yaw_comp = h30_imu_data.gyro_z * Gimbal_Yaw_Rpm_K;
	return limit_float(-yaw_comp, GIMBAL_YAW_COMP_LIMIT_RPM, -GIMBAL_YAW_COMP_LIMIT_RPM);
}

static uint16_t wrap_single_angle_x10(int32_t angle_x10)
{
	angle_x10 %= 3600;
	if (angle_x10 < 0)
		angle_x10 += 3600;
	return (uint16_t)angle_x10;
}

static void Visual_Gimbal_ClearState(void)
{
	vision_lost_ticks = VISION_LOST_TICKS + 1;
	vision_prev_valid = 0;
	prev_err_x = 0.0f;
	prev_err_y = 0.0f;
	vision_yaw_cmd = 0.0f;
	vision_pitch_cmd = 0.0f;
	vision_ff_yaw_cmd = 0.0f;
	Gimbal_Joint1_Target_Rpm = 0;
	Gimbal_Joint2_Target_Rpm = 0;
}

/* Gimbal output: yaw uses speed mode, pitch uses 0~359.9 deg single-angle mode. */
static void Visual_Gimbal_Output(float yaw_rpm, float pitch_rpm)
{
	yaw_rpm = limit_float(yaw_rpm, GIMBAL_YAW_LIMIT_RPM, -GIMBAL_YAW_LIMIT_RPM);
	pitch_rpm = limit_float(pitch_rpm, VISION_PITCH_STEP_LIMIT_X10, -VISION_PITCH_STEP_LIMIT_X10);
	Gimbal_Joint1_Target_Rpm = (int16_t)float_to_int_round(yaw_rpm);
	Gimbal_Joint2_Target_Rpm = (int16_t)float_to_int_round(pitch_rpm);
	Gimbal_Joint2_Target_Angle =
		wrap_single_angle_x10((int32_t)Gimbal_Joint2_Target_Angle + Gimbal_Joint2_Target_Rpm);
	BLDC_SetSpeed(BLDC_ADDR_1, Gimbal_Joint1_Target_Rpm);
	delay_us(250);
	BLDC_SetSingleAngle(BLDC_ADDR_2, Gimbal_Joint2_Target_Angle);
}

static void Visual_Gimbal_SearchScan(void)
{
	if (search_target_locked)
	{
		Visual_Gimbal_Output(0.0f, 0.0f);
		return;
	}

	Gimbal_Joint2_Target_Angle = 3480;

	if (search_scan_ticks >= (u16)(CONTROL_FREQUENCY * 5.0f / 2.0f))
	{
		Visual_Gimbal_Output(0.0f, 0.0f);
		return;
	}

	search_scan_ticks++;
	Visual_Gimbal_Output(0.0f, 0.0f);
}

/* Clear visual PD memory when target is invalid; keep yaw base compensation in run mode. */
static void Visual_Gimbal_StopTrack(float yaw_base_rpm)
{
	prev_err_x = 0.0f;
	prev_err_y = 0.0f;
	vision_prev_valid = 0;
	vision_yaw_cmd = 0.0f;
	vision_pitch_cmd = 0.0f;
	vision_ff_yaw_cmd = 0.0f;
	Visual_Gimbal_Output(yaw_base_rpm, 0.0f);
}

/* work_mode=SEARCH：寻靶；work_mode=RUN：巡线打靶并叠加补偿。 */
static void Visual_Gimbal_Process(u8 work_mode)
{
	u8 use_compensation = (work_mode == CONTROL_MODE_RUN) ? 1 : 0;
	float yaw_base_rpm = use_compensation ? Gimbal_GetYawCompRpm() : 0.0f;
	float err_x;
	float err_y;
	float diff_err_x;
	float diff_err_y;
	float cmd_yaw;
	float cmd_pitch;

	if (g_data_ready == 0)
	{
		if (vision_lost_ticks < 0xffff)
			vision_lost_ticks++;
		if (vision_lost_ticks >= VISION_LOST_TICKS)
		{
			prev_err_x = 0.0f;
			prev_err_y = 0.0f;
			vision_prev_valid = 0;
			vision_yaw_cmd = 0.0f;
			vision_pitch_cmd = 0.0f;
			vision_ff_yaw_cmd = 0.0f;
		}

		if (use_compensation == 0)
		{
			Visual_Gimbal_SearchScan();
		}
		else
		{
			Visual_Gimbal_Output(yaw_base_rpm + vision_yaw_cmd + vision_ff_yaw_cmd,
								 0.0f);
		}
		return;
	}

	g_data_ready = 0;
	vision_lost_ticks = 0;

	if (g_follow_area == 0)
	{
		if (use_compensation == 0)
		{
			Visual_Gimbal_ClearState();
			Visual_Gimbal_SearchScan();
		}
		else
		{
			Visual_Gimbal_StopTrack(yaw_base_rpm);
		}
		return;
	}

	if (use_compensation == 0)
	{
		search_target_locked = 1;
	}
	else
	{
		search_scan_ticks = 0;
		search_target_locked = 0;
	}

	err_x = (float)VISION_CENTER_X - (float)g_follow_x;
	err_y = (float)VISION_CENTER_Y - (float)g_follow_y;
	if (vision_prev_valid == 0)
	{
		prev_err_x = err_x;
		prev_err_y = err_y;
		vision_prev_valid = 1;
	}

	diff_err_x = err_x - prev_err_x;
	diff_err_y = err_y - prev_err_y;
	prev_err_x = err_x;
	prev_err_y = err_y;

	cmd_yaw = VISION_DIR_X * vision_segment_gain * (KP_VISION_X * err_x + KD_VISION_X * diff_err_x);
	if (abs_float(err_x) <= VISION_DEADZONE)
		cmd_yaw = 0.0f;
	vision_yaw_cmd = limit_float(cmd_yaw, VISION_YAW_LIMIT_RPM, -VISION_YAW_LIMIT_RPM);

	cmd_pitch = VISION_DIR_Y * vision_segment_gain * (KP_VISION_Y * err_y + KD_VISION_Y * diff_err_y);
	if (abs_float(err_y) <= VISION_DEADZONE)
		cmd_pitch = 0.0f;
	vision_pitch_cmd = limit_float(cmd_pitch, VISION_PITCH_LIMIT_RPM, -VISION_PITCH_LIMIT_RPM);

	Visual_Gimbal_Output(yaw_base_rpm + vision_yaw_cmd +
							 (use_compensation ? vision_ff_yaw_cmd : 0.0f),
						 vision_pitch_cmd);
}
/* 根据左右电机目标 PWM 设置方向引脚和占空比，停车时同时关闭 H 桥输入。 */
void Motor_SetPwm(int motor_left, int motor_right)
{
	Motor_Left = limit_int(motor_left, PWM_MAX, -PWM_MAX);
	Motor_Right = limit_int(motor_right, PWM_MAX, -PWM_MAX);
	if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
	{
		Motor_Left = 0;
		Motor_Right = 0;
		AIN1 = 0;
		AIN2 = 0;
		BIN1 = 0;
		BIN2 = 0;
		PWMA = 0;
		PWMB = 0;
		return;
	}
	if (Motor_Right > 0)
		AIN1 = 1, AIN2 = 0;
	else if (Motor_Right < 0)
		AIN1 = 0, AIN2 = 1;
	else
		AIN1 = 0, AIN2 = 0;
	PWMA = abs_int(Motor_Right);
	if (Motor_Left > 0)
		BIN1 = 1, BIN2 = 0;
	else if (Motor_Left < 0)
		BIN1 = 0, BIN2 = 1;
	else
		BIN1 = 0, BIN2 = 0;
	PWMB = abs_int(Motor_Left);
}

/* 将本周期编码器增量换算为左右轮线速度，单位由 WHEEL_PERIMETER 等参数决定。 */
static void Get_Velocity_From_Encoder(int encoder_left, int encoder_right)
{
	float encoder_left_pr;
	float encoder_right_pr;
	OriginalEncoder.A = encoder_left;
	OriginalEncoder.B = encoder_right;
	encoder_left_pr = (float)OriginalEncoder.A * LEFT_ENCODER_SIGN;
	encoder_right_pr = (float)OriginalEncoder.B * RIGHT_ENCODER_SIGN;
	MotorA.Current_Encoder = encoder_left_pr * CONTROL_FREQUENCY * WHEEL_PERIMETER /
							 (ENCODER_MULTIPLES * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);
	MotorB.Current_Encoder = encoder_right_pr * CONTROL_FREQUENCY * WHEEL_PERIMETER /
							 (ENCODER_MULTIPLES * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);
}

/* 左轮增量式 PI：直接累加 PWM 修正量，停车时清空积分/历史误差。 */
static int Incremental_PI_Left(float encoder, float target)
{
	static float bias = 0.0f;
	static float pwm = 0.0f;
	static float last_bias = 0.0f;
	bias = target - encoder;
	pwm += Velocity_KP * (bias - last_bias) + Velocity_KI * bias;
	if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
	{
		pwm = 0.0f;
		last_bias = 0.0f;
	}
	pwm = limit_float(pwm, (float)PWM_MAX, (float)-PWM_MAX);
	last_bias = bias;
	return (int)pwm;
}

/* 右轮增量式 PI，与左轮分开保存状态，避免两侧误差互相影响。 */
static int Incremental_PI_Right(float encoder, float target)
{
	static float bias = 0.0f;
	static float pwm = 0.0f;
	static float last_bias = 0.0f;
	bias = target - encoder;
	pwm += Velocity_KP * (bias - last_bias) + Velocity_KI * bias;
	if (Control_Work_Mode == CONTROL_MODE_SEARCH || !Control_Work_Enable)
	{
		pwm = 0.0f;
		last_bias = 0.0f;
	}
	pwm = limit_float(pwm, (float)PWM_MAX, (float)-PWM_MAX);
	last_bias = bias;
	return (int)pwm;
}

static void Control_StopWork(void)
{
	/* 停止当前动作，但保留当前模式选择；下一次单击仍启动这个模式。 */
	Control_Work_Enable = 0;
	search_scan_ticks = 0;
	search_target_locked = 0;
	Visual_Gimbal_ClearState();
	Visual_Gimbal_Output(0.0f, 0.0f);
}

static void Control_StartWork(void)
{
	/* 启动当前模式：SEARCH 寻靶，RUN 巡线打靶。 */
	Control_Work_Enable = 1;
	search_scan_ticks = 0;
	search_target_locked = 0;
	Visual_Gimbal_ClearState();
}

static void Control_ToggleWork(void)
{
	/* 单击在“当前模式启动”和“当前模式待启动”之间来回切换。 */
	if (Control_Work_Enable)
	{
		Control_StopWork();
	}
	else
	{
		Control_StartWork();
	}
}

static void Control_SwitchMode(void)
{
	/* 长按只切换模式并停止当前动作；切换后必须再单击启动。 */
	Control_StopWork();
	Control_Work_Mode = (Control_Work_Mode == CONTROL_MODE_SEARCH) ? CONTROL_MODE_RUN : CONTROL_MODE_SEARCH;
}

/* 按键处理：单击启停当前模式，双击不使用，长按切换寻靶/巡线打靶模式。 */
void Key_Process(void)
{
	u8 key = click_N_Double(50);
	if (key == 1)
	{
		Control_ToggleWork();
		return;
	}
	if (key == 2)
	{
		return;
	}
	if (Long_Press())
	{
		Control_SwitchMode();
	}
}
/* TIM2 main 5 ms loop: line inspection and PI always run; stop/run only selects output logic. */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		int encoder_left;
		int encoder_right;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		//		H30_IMU_TIM_Callback();
		Vision_Frequency_Update();
		Key_Process();

		encoder_left = Read_Encoder(4);
		encoder_right = Read_Encoder(3);
		Get_Velocity_From_Encoder(encoder_left, encoder_right);
		IRDM_line_inspection();

		MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder, MotorA.Target_Encoder);
		MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder, MotorB.Target_Encoder);

		if (Control_Work_Mode == CONTROL_MODE_SEARCH)
		{
			Motor_SetPwm(0, 0);
			vision_segment_gain = 0.4f;
			vision_ff_yaw_cmd = 0.0f;
			VISION_CENTER_X = -2;
			VISION_CENTER_Y = 10;
		}
		else
		{
			search_scan_ticks = 0;
			search_target_locked = 0;
			Motor_SetPwm(LEFT_PWM_SIGN * (int)MotorA.Motor_Pwm,
						 RIGHT_PWM_SIGN * (int)MotorB.Motor_Pwm);

			vision_ff_yaw_cmd = 0.0f;
			if (turn_cnt < 500 && turn_cnt >= 100)
			{
				vision_segment_gain = 0.6f;
				VISION_CENTER_X = 1;
				VISION_CENTER_Y = 10;

				if (CurrentRoadSegment == ROAD_SEG_AC)
				{
					vision_segment_gain = 0.3f;
					VISION_CENTER_X = 3;
					VISION_CENTER_Y = 7;
				}
				else if (CurrentRoadSegment == ROAD_SEG_CD)
				{
					vision_segment_gain = 0.3f;
					VISION_CENTER_X = 3;
					VISION_CENTER_Y = 7;
				}
			}
			else if (CurrentRoadSegment == ROAD_SEG_BA)
			{
				float line_speed_x100;
				vision_segment_gain = 0.8f;
				VISION_CENTER_X = -4;
				VISION_CENTER_Y = 8;
				line_speed_x100 = abs_float((MotorA.Current_Encoder + MotorB.Current_Encoder) * 0.5f) * 100.0f;
				vision_ff_yaw_cmd = limit_float(-VISION_DIR_X * BA_VISION_FF * line_speed_x100,
												VISION_YAW_LIMIT_RPM, -VISION_YAW_LIMIT_RPM);
			}
			else if (CurrentRoadSegment == ROAD_SEG_AC)
			{
				vision_segment_gain = AC_VISION_K;
				VISION_CENTER_X = 4;
				VISION_CENTER_Y = 4;
			}
			else if (CurrentRoadSegment == ROAD_SEG_CD)
			{
				float line_speed_x100;
				vision_segment_gain = CD_VISION_K;
				VISION_CENTER_X = 1;
				VISION_CENTER_Y = 7;
				line_speed_x100 = abs_float((MotorA.Current_Encoder + MotorB.Current_Encoder) * 0.5f) * 100.0f;
				vision_ff_yaw_cmd = limit_float(VISION_DIR_X * CD_VISION_FF * line_speed_x100,
												VISION_YAW_LIMIT_RPM, -VISION_YAW_LIMIT_RPM);
			}
			else if (CurrentRoadSegment == ROAD_SEG_DB)
			{
				vision_segment_gain = DB_VISION_K;
				VISION_CENTER_X = 3;
				VISION_CENTER_Y = 11;
			}
		}

		if (Control_Work_Enable)
		{
			Visual_Gimbal_Process(Control_Work_Mode);
		}
	}
}
