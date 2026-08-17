#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "bujing.h"

#define ParaNum 60
//编码器步数
#define ENCODER_STEPS 		3999
#define ENCODER_STEPS_MID 2000
#define ENCODER_OFFSET    14					//偏置

//步进电机齿轮数
#define NR      					50	

//更改10K/20K PWM频率，PWM的输出步数也要改变
//#define PWM_FRE						20000		//10K
#define PWM_FRE						8000		//10K

//PWM输出的步数
#define PWM_STEPS					625				//配的TIM1是80MHz 预分频值 8 ，中心对齐的情况下需要/2


#define PWM_STEPS_OFFSET	0					//因为死区引起的偏置

//步进电机单相线圈允许输入的最小电压
#define MIN_SUPPLY_VOLTAGE	8.0f			//输入电压小于8.0f不启动



//Π值
#define M_PI 3.1415926f

//电机方向
#define CW		(0)
#define CCW		(1)

//电压读取参数
#define     MAX_ADC_COUNT                	(float)(4096)    
#define     MAX_ADC_INPUT_VOLTAGE        	(float)(3.3f)     
#define     HALF_ADC_INPUT_VOLTAGE       	(float)(1.65f)
//母线电压
#define		DCBUSP_SENSE_TOP_RESISTOR		(float)(10.0f)
#define		DCBUSP_SENSE_BOTTOM_RESISTOR	(float)(1.0f)
#define		DCBUSP_SENSE_RATIO				(float)(DCBUSP_SENSE_BOTTOM_RESISTOR/(DCBUSP_SENSE_BOTTOM_RESISTOR + DCBUSP_SENSE_TOP_RESISTOR))
#define		VOLTAGEP_ADC_PVCC_PHY_RATIO		(float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUSP_SENSE_RATIO))

//电流传感器
#define		CURRENT_SENSOR_COEFFICIENT		(float)(0.132f)
#define		CURRENT_COUNT									(float)(4096)
#define		MAX_CURRENT_INPUT_VOLTAGE			(float)(3.3f)
#define		VOLTAGE_CURRENT_RATIO					(float)(MAX_CURRENT_INPUT_VOLTAGE/CURRENT_COUNT)

//总电流
#define	  RSHUNT												(float)(0.05f)		//采样电阻
#define		RS_HIGH												(float)(18.0f)		
#define		RS_LOW												(float)(2.0f)			
#define   MAX_CURRENT_INPUT_VOLTAGE_T		(float)(3.3f)
#define		CURRENT_COUNT_T								(float)(4096)
#define   CAMP_SCALE                   	(float)((RS_HIGH/RS_LOW) + 1.0f)		//放大倍数
#define   ADC_CURRENT_SCALE							(float)(MAX_CURRENT_INPUT_VOLTAGE_T / CURRENT_COUNT_T / CAMP_SCALE / RSHUNT )
																				//电流 = AD*（3.3/（4096 * 放大倍数 *采样电阻））-->电流系数=（3.3/（4096 * 放大倍数 *采样电阻））
void Soft_Init(void);
void Hardware_Init(void);
void Lock_Init(void);
extern volatile uint16_t MCPara[ParaNum];
extern float sin_table[4000];
extern float cos_table[4000];
void TIM_PWM_Stop(void);
void TIM_PWM_Start(void);
#endif


