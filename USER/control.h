#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#define PWM_MAX              6900
#define CONTROL_FREQUENCY    200.0f
#define ENCODER_MULTIPLES    4.0f
#define WHEEL_PERIMETER      0.204203519f
#define MOTOR_GEAR_RATIO     28.0f
#define ENCODER_RESOLUTION   13.0f
#define WHEEL_SPACING        0.1610f
#define SPEED_STEP           0.05f
#define SPEED_MAX            0.60f
#define CONTROL_MODE_RUN     0
#define CONTROL_MODE_SEARCH  1
#define LEFT_ENCODER_SIGN    1.0f
#define RIGHT_ENCODER_SIGN  1.0f
#define LEFT_PWM_SIGN       -1
#define RIGHT_PWM_SIGN       -1
typedef struct
{
	float Current_Encoder;
	float Motor_Pwm;
	float Target_Encoder;
} Motor_Parameter;
typedef struct
{
	int A;
	int B;
} Encoder_Original;
extern volatile Motor_Parameter MotorA;
extern volatile Motor_Parameter MotorB;
extern u8 Control_Work_Mode;
extern u8 Control_Work_Enable;
extern float Velocity_KP ;
extern float Velocity_KI ;
extern float Gimbal_Yaw_Rpm_K;
extern float KP_VISION_X;
extern float KD_VISION_X;
extern float KP_VISION_Y;
extern float KD_VISION_Y;
extern float AC_VISION_K;
extern float CD_VISION_K;
extern float DB_VISION_K;
extern float BA_VISION_FF;
extern float CD_VISION_FF;

extern int VISION_CENTER_X;
extern int VISION_CENTER_Y;
extern volatile u16 Vision_Fps;
extern volatile u8 Vision_Online;
extern volatile int16_t Gimbal_Joint1_Target_Rpm;
extern volatile int16_t Gimbal_Joint2_Target_Rpm;
extern volatile uint16_t Gimbal_Joint2_Target_Angle;
void Motor_SetPwm(int motor_left, int motor_right);
void Motor_ClosedLoop_5ms(void);
void Control_SetTargetSpeed(float speed);
void Control_AddTargetSpeed(float delta);
#endif
