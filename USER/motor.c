#include "motor.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "logic.h"
#include "protect.h"

extern MCUART_Type SerUart3;
volatile uint16_t MCPara[ParaNum] ={0};

uint16_t SMCPara[ParaNum] = {

	0,		// -0  地址
	0,		// -1  电流极性							0 极性是正			1 极性是负
	1,		// -2  开/闭环控制方式				0 开环					1 闭环
						   
	100,	// -3  积分限幅				
	24,		// -4  输出限幅
	
	300,		// -5  电机初始化定位时间
	
	1,		// -6  开环电机运动方向			0 正转					1 反转??ssc 1-0
	
	14,		// -7  开环力量				
	1,		// -8  开环步数延时时间 
	100,	// -9  开环转速NC
	
	80,		// -10 开环转速力量比例系数K	0.075					10转1  30转2   60转3  80转4  100转5 120转6  140转7  160转8		180转10  200转12  220转14
	2000,	// -11 开环转速力量比例系数b -2.5
	1000, // -12 开环转速力量比例系数倍数
	
	//D轴PI
	3,		// -13 D轴PI输出最大值
	3,		// -14 D轴PI输出最小值（-）
	80,		// -15 D轴Kp			倍率1000
	0,		// -16 D轴KI			倍率1000
	
	//Q轴PI
	13,		// -17 Q轴PI输出最大值
	13,		// -18 Q轴PI输出最小值（-）
	100,	// -19 Q轴Kp			倍率1000
	0,		// -20 Q轴KI			倍率1000
	
	//速度PI
	200,		// -21 速度环输出最大值
	200,		// -22 速度输出最小值（-）
	
	900,		// -23 速度Kp			倍率1000				800
	5,			// -24 速度KI			倍率1000				10
	
	3000,	// -25 开环启动加速时间	（ms）			更平滑原始1000
	100,	// -26 开环启动加速度(0-1000)       原始300
	
	96,		// -27 速度滤波值	（0-100）		85-95
	
	1,		// -28 开环换相时间爬坡
	15,		// -29 开环变化力量爬坡
	
	130,	// -30 允许启动电压
	
	120,	// -31 欠压保护值
	1000,	// -32 欠压保护时间
	300,	// -33 过压保护值
	1000,	// -34 过压保护时间
	
	0,		// -35 允许启动转速
	
	3,		// -36 -NC	
	10,		// -37 -NC
	
	80,		// -38 欠压保护值	/10V
	3000,	// -39 欠压保护时间	ms
	
	360,	// -40 过压保护值	/10V
	3000,	// -41 过压保护时间 ms
	
	14,		// -42 过流保护值	/10A
	500,	// -43 过流保护时间	ms
	
	100,		// -44 电机每次启动定位时长ms
	4,			// -45 编码器偏置变化爬坡周期
	50,			// -46 开机启动定位的力量	/10			-130
	
	200,		// -47 停机转速
	
	3,		// -48 加速度-速度向上爬坡值			-3
	5,		// -49 减速度-速度向下爬坡值			-4
	20,		// -50 爬坡周期-速度爬坡计算周期		-10
	
	1,		// -51 启动定位爬坡值，向上爬坡到启动定位的力量	
	500,		// -52 向上爬坡的次数规划。
	
	
	0,			//-53	 禁止故障自动复转；故障后必须先收到合法零速帧重新授权。
	60000,			//-54	 最大重启次数	
	50,		//-55  恢复时间ms
	
	
};

/*
 * 函数功能：初始化泵驱动硬件并建立上电零速重新授权状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Hardware_Init(void){
	HAL_StatusTypeDef uart_rx_status;		//记录USART3 DMA接收是否真正启动成功。
	
	SerUart3.LastValidTick = HAL_GetTick();	//从硬件初始化时开始计算首帧100ms超时。
	SerUart3.CommandSeen = 0U;			//上电后尚未收到任何完整合法控制帧。
	SerUart3.NeedZeroRearm = 1U;			//上电默认禁止非零命令，必须先收到合法零速帧。
//使能串口空闲中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能IDLE中断
	uart_rx_status = HAL_UART_Receive_DMA(&huart3,SerUart3.R_DATA,BUFFER_SIZE);	//启动DMA并保留返回值供安全判定。
	
	//开启PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);	//开启定时器1的通道1的PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);	//开启定时器1的通道1的PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);	//开启定时器1的通道1的PWM输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);	//开启定时器1的通道1的PWM输出
	
	//初始化PWM输出控制/测试
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	if(uart_rx_status != HAL_OK){
		Ctl_HardStop(E_COMM5);			//DMA未启动时保持功率关闭，禁止沿用任何运行目标。
	}
	
	//使能编码器
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	//使能ADC通过DMA进行采样
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);		//使用ADC前对其校准
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf,4);				//开启DMA转换	
	
	
    /* 启用TIM1刹车中断 */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);       // 设置高优先级
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	
}





//软件初始化
void Soft_Init(void){
	int16_t i;
	float temp = 0;
	float temp1 = 0;	
	
	for(i=0;i<300;i++){		//循环300次去检测ADC的初始电流中间值
		HAL_Delay(1);		
		temp = adc_buf[1];
		App.Adc.Current_Median1 = ( App.Adc.Current_Median1*0.70f + temp*0.30f );	
		temp1 = adc_buf[2];
		App.Adc.Current_Median2 = ( App.Adc.Current_Median2*0.70f + temp1*0.30f);		
	}
	for(i = 0;i < ParaNum;i ++){
			MCPara[i] = SMCPara[i];
	}
	
	//参数的初始化
	TIM2->CNT = 0;		//编码器计数的初始化
	App.Log.Offset_Cyc = MCPara[45];	
	//反馈速度滤波值
	App.Spd.Fil = (float)MCPara[27] /100.0f;
	
	App.Status = 0;		//状态初始化	
	
	//闭环速度爬坡
	App.Spd.Spd_RampUp = MCPara[48];
	App.Spd.Spd_RampDown = MCPara[49];
	
	//保护参数
	
	//允许启动电压
	App.Prot.Allow_Voltage = (float)MCPara[30]/10.0f;

	//欠压保护值
	App.Prot.UnderVoltage = (float)MCPara[38]/10.0f;	// /10v
	App.Prot.UnderVoltage_Tim = MCPara[39];		//	ms
	
	//过压保护值
	App.Prot.OverVoltage  =(float) MCPara[40]/10.0f;
	App.Prot.OverVoltage_Tim = MCPara[41];	//ms
	
	//过流保护值
	App.Prot.OverCurrent = (float)MCPara[42]/10.0f;
	App.Prot.OverCurrent_Tim = MCPara[43];	//ms
	
	
}



//用于爬坡定位
void Lock_Init(void){
	
	App.Lock.LockPowerBNow = 0.0f;
	App.Lock.LockPowerBUnit = (float)MCPara[46]/10.0f/MCPara[52];
	for(App.Lock.LockBCnt = 0;App.Lock.LockBCnt<MCPara[52];App.Lock.LockBCnt++){
		App.Lock.LockPowerBNow +=	App.Lock.LockPowerBUnit;
		Set_OutPut_B(App.Lock.LockPowerBNow,24.0f);		//琐死在A+
		HAL_Delay(MCPara[51]);//爬坡间隔
	}
	HAL_Delay(MCPara[5]);								//开环定位时间
	Set_OutPut_B(0.0f,24.0f);		//解锁
	
	
	App.Lock.LockPowerANow = 0.0f;
	App.Lock.LockPowerAUnit = (float)MCPara[46]/10.0f/MCPara[52];
	for(App.Lock.LockACnt = 0;App.Lock.LockACnt<MCPara[52];App.Lock.LockACnt++){
		App.Lock.LockPowerANow +=	App.Lock.LockPowerAUnit;
		Set_OutPut_A(App.Lock.LockPowerANow,24.0f);		//琐死在A+
		HAL_Delay(MCPara[51]);//爬坡间隔
	}
	HAL_Delay(MCPara[5]);								//开环定位时间
	TIM2->CNT = 0;		//编码器计数的初始化
	Set_OutPut_A(0.0f,24.0f);		//解锁

}



/*
 * 函数功能：幂等关闭四路PWM并清除比较寄存器，重复调用也保持功率级关闭。
 * 输入参数：无。
 * 返回参数：无。
 */
void TIM_PWM_Stop(void)
{
	TIM1->CCR1 = 0U;                        //先清A/B两相全部占空比，避免重新使能时带出旧值。
	TIM1->CCR2 = 0U;
	TIM1->CCR3 = 0U;
	TIM1->CCR4 = 0U;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); //关闭每个CC通道使能位。
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_MOE_DISABLE(&htim1);          //无条件清主输出使能，BREAK恢复也不能自动重新出波。
	__HAL_TIM_DISABLE(&htim1);              //停止TIM1计数器，确保关闭后不存在持续换相。
}


void TIM_PWM_Start(void){
	
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
}
