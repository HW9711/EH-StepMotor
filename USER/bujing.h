#ifndef __BUJING_H_
#define __BUJING_H_

#include "gpio.h"
#include "motor.h"

//LED
#define LED_GPIO			GPIOB
#define LED_PIN				GPIO_PIN_6

#define LED_ON()				  HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()				  HAL_GPIO_WritePin(GPIOB, LED_PIN, GPIO_PIN_SET)
#define LED_Toggle()			HAL_GPIO_TogglePin(GPIOB, LED_PIN)

//串口3
#define  BUFFER_SIZE  	200

typedef struct{
	
	uint16_t Tim_Triggered;				//1ms执行一次主函数
	volatile uint16_t Tim_Spd;		//10ms执行一次计算速度	
	volatile int Set_Speed;						//设定的转速
	volatile uint16_t StaLedCyc;	//指示灯1ms计时
	
	float	Offset_Encode_Set;		//目标编码器偏置
	float	Offset_Encode_Now;		//实际编码器偏置
	uint16_t	Offset_Cyc;						//爬坡的周期
	volatile uint16_t	Offset_CycCnt;				//爬坡计数
	
	volatile uint16_t	ErrReturnCnt;		//错误重启
	volatile uint16_t	ErrClearCnt;		//已经错误清除次数
	
}LogType;

typedef struct{
	volatile uint8_t 	Phase;			//相
	volatile uint32_t Tim_Cnt;		//开环换相时间计数
	volatile uint32_t Fre_Cnt;		//换相需要的计数次数
	
	float k;
	float b;
	float power;
	float set_power;								//目标力量
	
	float Power_Goal;
	float Step_Temp;
	volatile uint16_t Step;
	float Set_Step_Unit;					//目标换相间隔
	float Step_Unit;
	
	float RampUpTime;							//开环启动时间
	float RampUpTimeCnt;					//开环启动时间计数
	
	float PoweUpUnit;									//开环力量爬升
	float RampUp_Step;						//爬升步数的累加值，相当于foc中的开环角度累加值
	
	float RampUpInc;							//开环逐步增加步数的累加值
	float RampUpTemp;							//开环逐步加速步数的缓存值
	volatile uint8_t Stage;				//开环阶段				-0	：开环加速阶段， -1 正式开环阶段
	float Acceleration;						//开环加速度
	float PowerTemp;							//力量缓存值
	
	float PowerClimb;							//开环力量爬坡
	float StepClimb;							//开环换相时间爬坡
	
}OpenLoopType;

typedef struct{
	
	float Fil;
  float Now;
	volatile uint16_t Now_FilTemp;			//滤波速度，用来反馈速度
	volatile uint16_t Now_Fil;			//滤波速度，用来反馈速度
	float Temp1;
	float Temp;
	float Rad;
	float Set;
	float Set_SpdTemp;
	volatile uint16_t RampUpDown_Cycle;			//爬坡的周期
	
	uint16_t Spd_RampUp	;			//速度爬坡上升值
	uint16_t Spd_RampDown;		//速度爬坡下降值
	float	Spd_Time;
	volatile uint32_t EnTime_Cnt;
	volatile uint32_t En_Cnt;
	volatile uint32_t	En_Cnt_Last;
	volatile uint32_t En_Cnt_Abs;
	volatile int32_t	En_Err;
	volatile int8_t		En_Dir;
	
	volatile int8_t		Spd_Cal_10ms_Cnt;			//10ms计数
	volatile int8_t		Spd_Cal_10ms_Flag;		//10ms标志
}SpdType;


typedef struct{
	
  float Current_Median1;
	float Current_Median2;
	
	float Cur_Now1;
	float Cur_Now2;
	
	float Cur_temp1;
	float Cur_temp2;
	
	float Cur_AD1;
	float Cur_AD2;	
	
	float voltage;
	
	float Current_AllTemp;
	float Current_All;
	
}ADCType;



typedef struct{
	float voltage;		//计算电压
	float Current1;		//计算电流1
	float Current2;		//计算电流2
	float CurrentT;		//总电流

	float Sin_Val;		//得到的sin值
	float Cos_Val;		//得到的cos值
	
	float I_d_Measure;		//测量的Id值
	float I_q_Measure;		//测量的Iq值
	
	float I_d_Ref;				//参考Id值
	float I_q_Ref;				//参考Iq值
	
	float V_d;						//PI后输出给逆Park变换的D轴电压值
	float V_q;						//PI后输出给逆Park变换的Q轴电压值
	
	float V_a;						//Park逆变换后的a轴值
	float V_b;						//Park逆变换后的b轴值
	
	
	
  int16_t 	cnt;				//编码器步数
	int16_t   cnt_now;		//这一次编码器的步数		用来计算差值
  int16_t   cnt_last;		//上一次编码器的步数		用来计算差值
 	int16_t 	cnt_err;		//步数差值
}CalType;


typedef struct{
	
	float Kp;							//比例系数kp
	float Ki;							//积分系数Ki
	float Kd;							//微分系数Kd

	float PID_MaxOutput;	//PID最大输出值	
	float PID_MinOutput;	//PID最小输出值
	
	float Err;						//期望与实际误差
	float Err_Integral;		//积分误差
	
	float Measuer;				//测量
	
	float Out;						//比例积分输出
	float Out_Set;				//目标输出

}PIDType;

typedef struct{

	volatile uint8_t 	Uart_End_Flag;		//接收一帧完成标志
	volatile uint16_t Uart_SetSpd;			//串口设定转速
	volatile int8_t	Uart_Dir;					//串口设定方向
	
}UARType;

typedef struct{
	float Allow_Voltage;			//允许启动电压
	float UnderVoltage;				//欠压保护值
	uint32_t UnderVoltage_Tim;		//欠压保护的时间
	volatile int32_t	UnderVoltage_Cnt;	//欠压计数
	float OverVoltage;				//过压保护值
	uint32_t OverVoltage_Tim;		//过压保护的时间
	volatile int32_t	OverVoltage_Cnt;	//过压计数
	float OverCurrent;				//过流保护值
	uint32_t OverCurrent_Tim;		//过流保护的时间
	volatile int32_t	OverCurrent_Cnt;	//过流计数
	volatile uint8_t	Protect_1msFg;	//1ms标志

}ProtectType;

typedef struct{

	volatile uint16_t	LockACnt;			//定位次数
	float	LockPowerANow;							//定位力量
	float LockPowerAUnit;						//单位向上爬坡力量
	volatile uint16_t	LockBCnt;			//定位次数
	float	LockPowerBNow;							//定位力量
	float LockPowerBUnit;						//单位向上爬坡力量
	
}MotorStartType ;






typedef struct{
	volatile uint8_t AllRun;		//
	volatile uint8_t Status;		//状态
	volatile uint8_t Start;		//启动标志
	volatile uint8_t Err;		//错误标志
	LogType Log;			//逻辑
	SpdType Spd;			//速度
	ADCType Adc;			//ADC值
	CalType Cal;			//计算
	PIDType PID_Id;		//D轴PID
	PIDType PID_Iq;		//Q轴PID	
	PIDType PID_Spd;	//速度PID
	OpenLoopType 		Openloop;	//开环参数
	UARType 				Uart;			//串口通讯
	ProtectType	 		Prot;	//保护参数
	MotorStartType 	Lock;	//定位参数
	
	
	
}AppType;
 
 typedef struct 
{
    float   qdSum;  // Integrator Output of the PI Compensator
    float   qKp;    // Proportional Coefficient of the PI Compensator
    float   qKi;    // Integral Coefficient of the PI Compensator
    float   qKc;    // Anti-windup Coefficient of the PI Compensator
    float   qOutMax;// Max output limit of the PI Compensator
    float   qOutMin;// Min output limit of the PI Compensator
    float   qInRef; // Reference input of the PI Compensator
    float   qInMeas;// Feedback input of the PI Compensator
    float   qOut;   // Proportional + Integral Output of the PI Compensator
		
    float   qErr;   // Error input of the PI Compensator
    float   qErrDelta;
    float   qLastErr;
		float   qI_Max;	//积分限幅
		float   qI_Min;	//积分限幅
} mcParam_PIController;

extern AppType App;
void Motor_UpdateSamples(void); //更新DMA中的电压与电流，PWM关闭后仍用于安全放行判断。

extern mcParam_PIController mcApp_Q_PIParam;
extern mcParam_PIController mcApp_D_PIParam;
extern mcParam_PIController mcApp_Spd_PIParam;

void mcLib_CalcPI1( mcParam_PIController *pParam);
void Set_OutPut_A(float v_a , float v_supply);
void Set_OutPut_B(float v_b , float v_supply);

void PWM_InterruptService(void);
void Get_Encoder(void);

void Update_PhaseStep(void);			//更新换相时间
void Update_PowerStep(void);			//更新开环力量
void Update_Spd(void);						//更新爬坡速度
void Return_Uart(void);
#endif


