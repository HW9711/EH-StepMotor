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
/* 定位参数独立于运行PID；两次尝试均保留原参数46的电压上限。 */
#define ALIGN_PHASE_MAX_MS       75U   //每个B/A阶段的最长时间，四阶段单轮最多300ms。
#define ALIGN_RAMP_MS             25U   //幅值爬坡和相位过渡时间，降低安装管子后的冲击。
#define ALIGN_STABLE_MS           20U   //同一原始计数窗口内稳定20ms才接受阶段结果。
#define ALIGN_RELEASE_MS          50U   //合法零速、PB12低和编码器静止连续满足的时间。
#define ALIGN_RETRY_OFF_MS       100U   //两轮之间关闭功率至少100ms，避免连续硬顶管路。
#define ALIGN_MAX_ATTEMPTS         2U   //每轮授权最多两次定位，连续零速不会无限重试。
#define ALIGN_ZERO_FRAMES          2U   //至少两帧连续合法零速才能开始定位。
#define ALIGN_ENCODER_SIGN         1   //A正相向B正相过渡时编码器应增加，与现有正弦表一致。
#define ALIGN_STEP_COUNTS ((ENCODER_STEPS + 1U) / (4U * NR)) //当前4000计数、50齿，四分之一电周期为20计数。
#define ALIGN_STEP_TOLERANCE       4   //单程允许20±4计数，不能只凭编码器有变化就放行。
#define ALIGN_RETURN_TOLERANCE     3   //两次A相位置最多差3计数，排除明显回位偏差。
#define ALIGN_STILL_COUNTS         1   //稳定窗口相对固定参考最多变化1计数，缓慢爬行会重置窗口。
#define ALIGN_BUS_LIMIT_RATIO   0.8f   //定位软限流为既有母线过流阈值的80%，不提高原保护值。
#define ALIGN_PHASE_LIMIT_A     1.2f  //用户确认的额定相电流，定位达到此值立即关闭输出。
#define ALIGN_PHASE_SOFT_A (ALIGN_PHASE_LIMIT_A * 0.9f) //约1.08A开始降压，给额定相电流上限预留余量。

typedef enum {
    ALIGN_READY = 0,                 //定位通过，等待新的零速握手和启动命令。
    ALIGN_WAIT_ZERO,                 //等待合法零速与物理安全条件。
    ALIGN_B_FIRST,                   //B相预定位，尚不建立编码器零点。
    ALIGN_A_FIRST,                   //第一次A相定位，保存候选参考。
    ALIGN_B_CHECK,                   //A向B运动，检查正向位移。
    ALIGN_A_CHECK,                   //B向A回位，检查反向位移与重复性。
    ALIGN_RETRY_OFF,                 //第一次失败后的断电静止间隔。
    ALIGN_FAILED                    //本轮失败锁存，需一次新的启动请求才能再申请定位。
} MotorAlignState;

extern volatile uint8_t AlignValid;          //只有校验通过才为1，零速清故障不能直接置位。
extern volatile uint8_t AlignState;          //当前定位阶段，便于仿真器直接观察。
extern volatile uint8_t AlignAttempt;        //当前授权轮中的尝试次数。
extern volatile uint8_t AlignFailure;        //最近失败原因：1不稳定、2去程、3回程、4回位、5保护、6命令中断。
extern volatile int16_t AlignDeltaForward;   //实测A向B的有符号计数位移。
extern volatile int16_t AlignDeltaReturn;    //实测B向A的有符号计数位移。
extern volatile int16_t AlignReturnError;    //最后A位置相对第一次A位置的误差。
extern volatile float AlignPower;           //当前受限的定位电压幅值。
void Motor_AlignInit(void);                   //初始化上电校准门禁，功率保持关闭。
void Motor_AlignProcess(void);                //主循环按真实毫秒节拍推进定位。
void Motor_AlignOutput(void);                 //PWM中断在新鲜采样后更新定位输出。
void Motor_AlignOnFault(void);                //硬停时撤销定位或让首次上电继续等待通信。
uint8_t Motor_AlignCommand(uint16_t speed);   //返回1表示命令由校准门禁消费，不能进入普通运行处理。
extern volatile uint16_t MCPara[ParaNum];
extern float sin_table[4000];
extern float cos_table[4000];
void TIM_PWM_Stop(void);
void TIM_PWM_Start(void);
#endif
