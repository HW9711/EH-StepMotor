#include "motor.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "logic.h"
#include "protect.h"
#include <stdlib.h> //定位校验用有符号位移绝对值，不依赖滤波转速。
#include <float.h> //诊断只保留有限电流/电压样本，不将NaN或无穷值作为有效记录。

extern MCUART_Type SerUart3;
volatile uint16_t MCPara[ParaNum] ={0};

uint16_t SMCPara[ParaNum] = {

	0,		// -0  地址
	0,		// -1  电流极性							0 极性是正			1 极性是负
	1,		// -2  开/闭环控制方式				0 开环					1 闭环
						   
	100,	// -3  积分限幅				
	24,		// -4  输出限幅
	
	300,		// -5 满幅后的慢响应观察预算，单位ms；稳定100ms且位移合格可提前结束，未到位仍按原300ms观察。
	
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
	15,		// -17 Q轴正向输出上限，单位V，当前为15V；调大增加可用输出电压，调小限制运行输出。
	15,		// -18 Q轴负向输出限幅的绝对值，单位V，当前为15V；调大增加负向可用输出电压，调小限制负向输出。
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
	
	100,		// -44 兼容保留旧启动定位时间；已验证零点的普通启停不重复定位。
	4,			// -45 编码器偏置变化爬坡周期
	60,			// -46 定位电压，单位0.1V，当前为6.0V；调大增强定位励磁并增加发热，调小减弱励磁，首轮和补试使用同一上限。
	
	200,		// -47 停机转速
	
	3,		// -48 加速度-速度向上爬坡值			-3
	5,		// -49 减速度-速度向下爬坡值			-4
	20,		// -50 爬坡周期-速度爬坡计算周期		-10
	
	1,		// -51 原定位每步延时，单位ms；与参数52相乘得到非阻塞爬升时长。
	500,		// -52 原定位爬升步数；与参数51组成默认500ms爬升，不再阻塞循环。
	
	
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



/* 定位状态独立于运行状态；调试变量不改变既有串口帧长度。 */
volatile uint8_t AlignValid = 0U;          //初始零点不可信，必须经过往返校验。
volatile uint8_t AlignState = ALIGN_WAIT_ZERO; //上电等待零速授权。
volatile uint8_t AlignAttempt = 0U;        //当前授权已尝试次数。
volatile uint8_t AlignFailure = 0U;        //保存最近失败原因，重试成功后也保留供调试。
volatile int16_t AlignDeltaForward = 0;    //A向B的位移观测。
volatile int16_t AlignDeltaReturn = 0;     //B向A的位移观测。
volatile int16_t AlignReturnError = 0;     //前后A位置重复性观测。
volatile float AlignPower = 0.0f;          //按真实毫秒爬升、由PWM服务提交的定位电压幅值。
volatile MotorAlignFailureSnapshot AlignFirstFailure = {0}; //首个失败发布后保留，直到明确开始新一轮定位。
volatile MotorAlignSuccessSnapshot AlignLastSuccess = {0}; //完整六阶段通过后发布，保留成功时实际位置及稳定质量。
static MotorAlignSuccessSnapshot AlignCandidate = {0}; //只收集当前尝试的候选，未通过全部判据不能作为已验证零点。
volatile uint16_t AlignRejectedStarts = 0U; //普通启动拒绝只累计次数，不破坏定位或覆盖首次原因。
static float AlignSampleA = 0.0f;          //保存当前尝试最近一次有效A相采样，不用于电流闭环。
static float AlignSampleB = 0.0f;          //保存与A相同一提交周期的B相采样。
static float AlignSampleVoltage = 0.0f;    //保存同一周期的母线电压。
static uint32_t AlignSampleTick = 0U;      //记录最近有效样本时间，供失效样本后的故障排查。
static uint8_t AlignSampleValid = 0U;      //新阶段出波前清零，避免将上一阶段采样当成本阶段结果。
static uint8_t AlignZeroFrames = 0U;       //安全零速授权计数；故障或新校准清零，忙时非零拒绝不撤销有界校准。
static uint32_t AlignLastTick = 0U;        //保证主循环只在毫秒改变后推进一次。
static uint32_t AlignPhaseTick = 0U;       //当前阶段的真实开始时间。
static uint32_t AlignAttemptTick = 0U;     //单轮加力的总超时基准。
static uint32_t AlignOffTick = 0U;         //轮间关闭PWM的起点。
static uint32_t AlignStillTick = 0U;       //固定编码器参考窗口开始时间。
static int16_t AlignStillAnchor = 0;       //防止每毫秒缓慢爬动被误当静止。
static int16_t AlignStillMin = 0;          //末段窗口相对锚点的最小位移，用于整体峰峰值而非相邻采样判断。
static int16_t AlignStillMax = 0;          //末段窗口相对锚点的最大位移，缓慢累积或往复摆动均计入质量。
static int16_t AlignFirstB = 0;            //第一次正B位置，首段B到A也必须检查方向和距离。
static int16_t AlignFirstA = 0;            //第一次A相定位的候选计数，尚未写入零点。
static int16_t AlignCheckB = 0;            //校验B相到达后的计数。
static int16_t AlignSecondA = 0;           //第二次同侧返回A的位置，后续与另一侧返回结果交叉比较。
static int16_t AlignOppositeB = 0;          //反B相到位位置，最终A必须从此处正向返回而非重复同侧吸合。
static volatile uint16_t AlignTableIndex = ALIGN_STEP_COUNTS; //静止坐标磁场方向，不能使用未经校准的编码器角度。
static volatile uint16_t AlignPreviousIndex = ALIGN_STEP_COUNTS; //保存上一阶段已到位磁场，用于换相交接，避免带载时先撤力再吸合。

/*
 * 函数功能：计算按TIM2实际4000计数周期回绕的原始位移。
 * 输入参数：now为当前计数，before为比较基准。
 * 返回参数：最近半圈内的有符号位移。
 */
static int16_t Align_Delta(int16_t now, int16_t before)
{
    int32_t delta = (int32_t)now - before; //先用有符号宽类型相减，避免0/3999附近下溢。
    if(delta > (int32_t)((ENCODER_STEPS + 1U) / 2U)){
        delta -= ENCODER_STEPS + 1U;     //计数周期是ARR+1，不能减3999。
    }else if(delta < -(int32_t)((ENCODER_STEPS + 1U) / 2U)){
        delta += ENCODER_STEPS + 1U;     //负向跨零同样按4000修正。
    }
    return (int16_t)delta;               //定位运动远小于半圈。
}

/*
 * 函数功能：PWM关闭后锁存本授权轮首次失败，避免补试或后续启动拒绝覆盖原始证据。
 * 输入参数：reason为定位判据原因，fault_code为本次原始保护或定位故障码；调用方须已关PWM并处于临界区。
 * 返回参数：无。
 */
static void Align_LatchFailure(uint8_t reason, uint8_t fault_code)
{
    uint32_t now;                         //只有尚未保存首次失败时才读取时间，减少保护路径工作量。
    if(AlignFirstFailure.valid != 0U){ return; } //自动补试和重复保护不能替换本轮首个失败。
    now = HAL_GetTick();                  //关功率之后才采集诊断时间，不延迟硬停寄存器操作。
    AlignFirstFailure.tick_ms = now;     //保存与相位超时可对照的故障时刻。
    AlignFirstFailure.phase_elapsed_ms = (AlignState >= ALIGN_B_FIRST && AlignState <= ALIGN_A_VERIFY) ? now - AlignPhaseTick : 0U; //六个加力阶段均记录真实时长，未加力阶段记0。
    AlignFirstFailure.state = AlignState; //在FAILED覆盖前记录真实失败阶段。
    AlignFirstFailure.attempt = AlignAttempt; //保留首个失败属于哪次尝试。
    AlignFirstFailure.reason = reason;   //几何判据和保护码分开，避免把所有失败统称过流。
    AlignFirstFailure.raw_error = fault_code; //记录硬停最终采用的保护码，而非尚未发布的App.Err旧值。
    AlignFirstFailure.encoder_count = (int16_t)TIM2->CNT; //不清零编码器，只保留停机位置。
    AlignFirstFailure.delta_forward = AlignDeltaForward; //复制当前去程测量，阶段字段说明其有效范围。
    AlignFirstFailure.delta_return = AlignDeltaReturn; //复制当前回程测量，不借用补试后的数值。
    AlignFirstFailure.return_error = AlignReturnError; //复制两次A相重复性误差。
    AlignFirstFailure.current_a = AlignSampleA; //保留最近有限值A相采样，不能据此自动升压。
    AlignFirstFailure.current_b = AlignSampleB; //保留同一采样周期B相值。
    AlignFirstFailure.bus_voltage = AlignSampleVoltage; //保留同周期母线电压。
    AlignFirstFailure.sample_tick_ms = AlignSampleTick; //保留样本新旧信息，允许判别故障后的采样失效。
    AlignFirstFailure.sample_valid = AlignSampleValid; //样本不存在时调用方不得使用旧电流数值。
    __DMB();                              //先提交所有诊断字段，再发布可读标志。
    AlignFirstFailure.valid = 1U;         //该标志只在下一次明确定位授权时清除。
}

/*
 * 函数功能：重新开始一个原始编码器稳定窗口。
 * 输入参数：now为当前毫秒时间。
 * 返回参数：无。
 */
static void Align_ResetStill(uint32_t now)
{
    AlignStillAnchor = (int16_t)TIM2->CNT; //固定参考而非相邻两点比较，缓慢滑动也会被发现。
    AlignStillTick = now;                 //窗口不继承上一相的稳定时间。
    AlignStillMin = 0;                    //新窗口丢弃旧的最小位移，不能跨相位复用稳定质量。
    AlignStillMax = 0;                    //新窗口从当前计数开始累计完整摆幅。
}

/*
 * 函数功能：检查编码器在固定位置附近保持静止的时间。
 * 输入参数：now为当前毫秒时间，duration为要求的静止时间。
 * 返回参数：稳定窗口满足返回1，否则返回0。
 */
static uint8_t Align_IsStill(uint32_t now, uint32_t duration)
{
    if(abs(Align_Delta((int16_t)TIM2->CNT, AlignStillAnchor)) > ALIGN_STILL_COUNTS){
        Align_ResetStill(now);           //位移越界即重新计时，不能把卡滞爬行当稳定。
    }
    return ((uint32_t)(now - AlignStillTick) >= duration) ? 1U : 0U; //无符号差值支持HAL节拍回绕。
}

/*
 * 函数功能：只对加力末段检查连续100ms的整体位置质量，等待/补试的原静止门禁不变。
 * 输入参数：now为当前毫秒时间。
 * 返回参数：整个稳定窗口峰峰值合格且时长足够返回1，否则重新观察且不延长阶段截止。
 */
static uint8_t Align_IsSettled(uint32_t now)
{
    int16_t delta = Align_Delta((int16_t)TIM2->CNT, AlignStillAnchor); //按真实4000周期比较固定锚点，跨零不能误判跳变。
    if(delta < AlignStillMin){ AlignStillMin = delta; } //保留窗口内最低点，往复返回锚点也不能抹掉摆动。
    if(delta > AlignStillMax){ AlignStillMax = delta; } //保留窗口内最高点，连续微小爬动会累积成超限。
    if((int32_t)AlignStillMax - AlignStillMin > ALIGN_QUALITY_SPAN_COUNTS){
        Align_ResetStill(now);            //超过1计数峰峰值后从当前点重计100ms，不接受最后短暂静止。
        return 0U;                       //本周期不能记录候选位置，原相位最大时限仍继续计时。
    }
    return ((uint32_t)(now - AlignStillTick) >= ALIGN_STABLE_MS) ? 1U : 0U; //只返回观测结果，不设置零点或运行授权。
}

/*
 * 函数功能：保存已通过本阶段判据的候选数据，调用方须在阶段提交临界区内。
 * 输入参数：index为六阶段数组下标，count为本次已检查的原始位置，now为当前毫秒时间。
 * 返回参数：无，候选仅供全部阶段通过后的成功快照使用。
 */
static void Align_RecordPhase(uint8_t index, int16_t count, uint32_t now)
{
    uint32_t stable_ms = now - AlignStillTick; //记录真实连续稳定长度，不能用固定保持时间代替。
    AlignCandidate.encoder_count[index] = count; //保留清零前的六个原始位置，便于核对两侧偏差。
    AlignCandidate.stable_ms[index] = (stable_ms > 0xFFFFU) ? 0xFFFFU : (uint16_t)stable_ms; //长时间参数用饱和值记录，避免诊断回绕。
    AlignCandidate.swing_counts[index] = (uint16_t)(AlignStillMax - AlignStillMin); //保留实际峰峰值，明确为何通过稳定判据。
    AlignCandidate.current_a[index] = AlignSampleA; //临界区内与其余样本一起复制，不读取半更新电流。
    AlignCandidate.current_b[index] = AlignSampleB; //保持反B相电流符号，诊断不改变原电流保护。
    AlignCandidate.bus_voltage[index] = AlignSampleVoltage; //保存同周期母线样本，不据此推断实际转矩。
    AlignCandidate.sample_tick_ms[index] = AlignSampleTick; //最近有限值可能早于本阶段末端，必须同时给出时刻。
    if(AlignSampleValid != 0U){
        AlignCandidate.sample_valid_mask |= (uint8_t)(1U << index); //有样本才置位，未置位字段不得当作有效测量。
    }
    AlignCandidate.phase_mask |= (uint8_t)(1U << index); //先写全部字段再标记该阶段已经完整收集。
}

/*
 * 函数功能：判断关闭PWM后的电压、电流和硬件输入是否允许定位。
 * 输入参数：无。
 * 返回参数：所有物理条件满足返回1，否则返回0。
 */
static uint8_t Align_SafeToStart(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
            App.Cal.voltage >= App.Prot.Allow_Voltage &&
            App.Cal.voltage <= App.Prot.OverVoltage - 1.0f &&
            App.Cal.CurrentT >= 0.0f &&   //仍拒绝无效母线采样，不把未知电流状态作为重新加力条件。
            Protect_CanClearFault(E_OC1) != 0U) ? 1U : 0U; //复用原母线过流恢复回差，不新增独立相电流门槛。
}

/*
 * 函数功能：建立上电校准门禁，替代阻塞式Lock_Init。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_AlignInit(void)
{
    AlignValid = 0U;                      //上电清计数仅供测量，不能视作定位成功。
    AlignState = ALIGN_WAIT_ZERO;         //等待后续新鲜的连续零速帧。
    AlignAttempt = 0U;                    //本次上电尚未加力。
    AlignZeroFrames = 0U;                 //丢弃初始化期间留下的授权计数。
    AlignPower = 0.0f;                    //禁止残留定位幅值。
    AlignFailure = 0U;                    //新的上电校准不继承旧诊断原因。
    AlignFirstFailure.valid = 0U;         //上电开始新授权轮，后续第一次失败重新建立快照。
    AlignLastSuccess.valid = 0U;          //上电没有可信成功记录，不能显示上次RAM残留候选。
    AlignRejectedStarts = 0U;             //从本次上电开始累计未就绪启动请求。
    AlignSampleValid = 0U;                //上电尚未有定位期间的有效采样。
    SerUart3.NeedZeroRearm = 1U;          //校准通过前禁止普通启动。
    TIM_PWM_Stop();                       //编码器和ADC继续采样，只关闭功率输出。
    AlignLastTick = HAL_GetTick();        //从初始化完成时开始非阻塞调度。
    Align_ResetStill(AlignLastTick);      //等待阶段从当前真实位置确认静止。
}

/*
 * 函数功能：未通过定位时撤销故障中的校准输出并限制重复加力，运行停机的有效基准不进入本函数。
 * 输入参数：fault_code为硬停已经选定的真实故障码，调用前PWM必须已经关闭。
 * 返回参数：无。
 */
void Motor_AlignOnFault(uint8_t fault_code)
{
    uint8_t wait_release = (AlignState == ALIGN_WAIT_ZERO && AlignAttempt == 0U) ? 1U : 0U; //尚未加力的上电等待可在故障释放后校准；校准中故障锁住本轮。
    if(wait_release == 0U){ Align_LatchFailure(5U, fault_code); } //未开始的上电通信等待不占用首个定位失败记录。
    AlignValid = 0U;                      //未通过校准的零点继续无效，不能借清故障放行普通启动。
    AlignState = wait_release ? ALIGN_WAIT_ZERO : ALIGN_FAILED; //连续零速不能反复重启失败的定位。
    if(wait_release != 0U){
        AlignAttempt = 0U;               //首次故障释放后可申请一轮有限校准。
    }
    AlignFailure = 5U;                    //诊断保留“保护中断”原因，App.Err仍保留具体故障码。
    AlignZeroFrames = 0U;                 //旧零速帧不能作为故障后的授权。
    AlignPower = 0.0f;                    //重进定位时幅值必须从零爬升。
    Align_ResetStill(HAL_GetTick());      //再次安全确认必须从故障后开始。
}

/*
 * 函数功能：消费校准期间的速度帧，连续零速维持校准但不能启动运行。
 * 输入参数：speed为CRC和范围已校验的速度幅值。
 * 返回参数：返回1表示已处理，调用者不能执行普通零速硬停或运行赋值。
 */
uint8_t Motor_AlignCommand(uint16_t speed)
{
    if(AlignValid != 0U){
        return 0U;                       //正常运行仍使用原串口路径。
    }
    if(speed != 0U){
        if(AlignRejectedStarts < 0xFFFFU){ AlignRejectedStarts++; } //仅累计误操作，不中断已经授权的有界校准。
        App.Log.Set_Speed = 0;            //未就绪非零命令永远不保存为待运行速度。
        return 1U;                       //BUSY保持原阶段，FAILED保持原故障；均不能进入普通运行处理。
    }
    if(AlignState == ALIGN_FAILED){
        if(Align_SafeToStart() != 0U &&
           (App.Err == E_ALIGN6 || Protect_CanClearFault(App.Err) != 0U)){
            if(AlignZeroFrames < ALIGN_ZERO_FRAMES){ AlignZeroFrames++; } //失败后只累计新的安全零速帧，给独立重定位请求使用。
        }else{
            AlignZeroFrames = 0U;         //故障物理条件尚未恢复时撤销重新加力资格。
        }
        return 1U;                       //零速不清定位失败显示，也不恢复尝试次数或自动加力。
    }
    if(App.Err != E_NONE0){
        if(Align_SafeToStart() == 0U ||
           (App.Err != E_ALIGN6 && Protect_CanClearFault(App.Err) == 0U)){
            return 1U;                   //物理条件未恢复时保留原故障和关闭状态。
        }
        Protect_ResetCounters();         //新的零速握手仅清保护累计值，不设置AlignValid。
        Ctl_HardStop(E_NONE0);            //故障确认时功率保持关闭。
        if(App.Err != E_NONE0){ return 1U; } //清理边界出现新的BREAK时不发布任何零速授权。
        SerUart3.NeedZeroRearm = 1U;      //仅清首次等待阶段的保护，校准完成前仍禁止普通启动。
    }
    if(AlignZeroFrames < ALIGN_ZERO_FRAMES){
        AlignZeroFrames++;               //最多记录两帧，帧的新鲜度由100ms租约保证。
    }
    return 1U;                           //定位中合法零速只续约，不执行普通硬停。
}

/*
 * 函数功能：只在失败锁存、物理恢复和新零速握手后开启一轮有限定位，绝不缓存运行目标。
 * 输入参数：无，调用方只允许从CRC正确且速度为0的独立重定位操作码进入，并处于命令提交临界区。
 * 返回参数：接受新定位轮返回1；定位中、已就绪或恢复条件不足时返回0且不重新加力。
 */
uint8_t Motor_AlignRequest(void)
{
    if(AlignValid != 0U || AlignState != ALIGN_FAILED || AlignZeroFrames < ALIGN_ZERO_FRAMES ||
       Align_SafeToStart() == 0U || (App.Err != E_ALIGN6 && Protect_CanClearFault(App.Err) == 0U)){
        return 0U;                       //重复请求不能重置正在进行的尝试，真实保护未恢复时不能清故障。
    }
    Ctl_HardStop(E_NONE0);                //新授权建立前继续保持功率关闭，硬件BREAK仍可拒绝清故障。
    if(App.Err != E_NONE0){ return 0U; }  //硬停提交期间出现新硬件保护时保留原失败与首次快照。
    Protect_ResetCounters();             //物理恢复确认后清旧保护计时，不修改任何阈值。
    AlignAttempt = 0U;                   //只有显式新定位轮才能复位两次尝试上限。
    AlignFailure = 0U;                   //新轮的最近原因从空记录开始。
    AlignFirstFailure.valid = 0U;        //本轮首个失败将在下一次失败时一次性建立。
    AlignLastSuccess.valid = 0U;         //明确新定位授权后，必须等本轮完整通过才能再发布成功记录。
    AlignSampleValid = 0U;               //不能把上一轮采样误认为新的定位样本。
    AlignZeroFrames = 0U;                //接下来还需新的普通零速帧，独立请求本身不授权直接出波。
    AlignPower = 0.0f;                   //新轮必须从零幅值按原500ms爬升。
    SerUart3.NeedZeroRearm = 1U;         //即使后续定位成功，也必须新零速、新启动，不能自动运行。
    AlignState = ALIGN_WAIT_ZERO;        //只转入安全等待，不在串口处理中直接操作定位输出。
    Align_ResetStill(HAL_GetTick());    //原50ms静止确认从本次明确请求之后重新建立。
    return 1U;                           //反馈将变为BUSY，主控继续零速通信而非补发启动速度。
}

/*
 * 函数功能：取得固定7字节反馈使用的定位状态与原始故障码，旧主控收到非零状态会安全停机。
 * 输入参数：无，调用方在反馈快照临界区内读取。
 * 返回参数：高半字节为READY/BUSY/FAILED，低半字节保持0~6原始保护码。
 */
uint8_t Motor_AlignFeedback(void)
{
    uint8_t status = ALIGN_FEEDBACK_BUSY; //默认未就绪，不能把App.Err=0当作已校准。
    if(AlignValid != 0U && SerUart3.NeedZeroRearm == 0U){
        status = ALIGN_FEEDBACK_READY;   //已验证基准且零速授权完成，仍须检查同帧故障码。
    }else if(AlignValid == 0U && AlignState == ALIGN_FAILED){
        status = ALIGN_FEEDBACK_FAILED;  //失败期间始终回报失败，普通零速不隐藏该状态。
    }
    return (uint8_t)(status | (App.Err & 0x0FU)); //原故障优先级保持，由主控分别解析状态和保护。
}

/*
 * 函数功能：明确全停时立即取消定位；已验证零点保留，未完成定位必须再次明确申请才能加力。
 * 输入参数：无，调用方已校验独立取消帧为零速并进入命令提交临界区。
 * 返回参数：无。
 */
void Motor_AlignCancel(void)
{
    uint8_t failure = AlignFailure;       //已失败时保留最近判据，重复取消不能把物理失败改成用户取消。
    uint8_t fault = App.Err;              //全停不是清故障命令，原始保护码保持原优先级。
    if(AlignValid != 0U){
        Ctl_HardStop(fault);              //已验证运行仅执行原硬停，编码器计数和有效基准不变。
        if(App.Err == E_NONE0){ SerUart3.NeedZeroRearm = 0U; } //无故障的明确零速停机完成授权，后续仍需新的非零命令。
        return;                          //已验证零点不进入失败重定位流程。
    }
    TIM_PWM_Stop();                       //先关功率，再保存诊断，取消不等待相位计时结束。
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_RESET ||
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET){ fault = E_Hardware; } //取消边界的硬件保护仍优先。
    if(fault == E_NONE0){ fault = E_ALIGN6; } //未验证定位被取消后继续使用未定位故障码禁止运行。
    if(AlignState != ALIGN_FAILED){
        failure = 6U;                    //只给本次主动取消建立原因，不覆盖已存在失败的最近判据。
        Align_LatchFailure(fault == E_Hardware ? 5U : failure, fault); //已有首次记录不会被替换，硬件保护仍记录保护原因。
    }
    Ctl_HardStop(fault);                  //同时清目标速度、运行环缓存和零速授权，保留真实保护。
    AlignState = ALIGN_FAILED;           //即使尚在上电WAIT，也锁住本轮，后续普通零速不能再次自动加力。
    AlignFailure = failure;              //恢复取消前的判据或明确取消原因，不接受硬停通用原因覆盖。
    AlignZeroFrames = 0U;                //再次校准必须重新收到安全零速并收到独立重定位请求。
}

/*
 * 函数功能：结束一次未通过的尝试，最多安排一次断电后的补试。
 * 输入参数：reason为定位失败原因。
 * 返回参数：无。
 */
static void Align_Fail(uint8_t reason)
{
    uint32_t primask = __get_PRIMASK();    //保护“结束输出→选择补试”免受BREAK竞争覆盖。
    __disable_irq();
    TIM_PWM_Stop();                       //任何判据失败先关CC、MOE及计数器。
    Align_LatchFailure(reason, App.Err != E_NONE0 ? App.Err :
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_RESET ||
         __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET ? E_Hardware : E_ALIGN6)); //关功率后记录首个判据与真实保护，补试不覆盖。
    AlignPower = 0.0f;
    AlignFailure = reason;               //保留失败阶段供断点观察。
    if(App.Err == E_NONE0 &&
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET &&
       AlignAttempt < ALIGN_MAX_ATTEMPTS){
        AlignState = ALIGN_RETRY_OFF;     //只有单纯校验失败才允许一次补试。
        App.Status = CTLS_WAIT;           //断电间隔不进入运行状态。
        AlignOffTick = HAL_GetTick();     //真实关闭时间用于100ms间隔。
        Align_ResetStill(AlignOffTick);
    }else{
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err :
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_RESET ||
             __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET ? E_Hardware : E_ALIGN6)); //物理保护优先于定位失败码。
        AlignState = ALIGN_FAILED;        //次数耗尽后持续零速保留失败，只有独立重定位请求能重新授权。
        AlignFailure = reason;
    }
    if(primask == 0U){ __enable_irq(); }  //恢复进入前的中断状态。
}

/*
 * 函数功能：提前检查发现位移不合格时保留原观察时间，避免加快正常定位后误拒绝慢响应泵。
 * 输入参数：reason为当前几何不合格原因；elapsed为本阶段已用毫秒数。
 * 返回参数：无；原观察时间未满时继续同相励磁，超时与电气保护仍由原路径独立处理。
 */
static void Align_WaitOrFail(uint8_t reason, uint32_t elapsed)
{
    if(elapsed >= ALIGN_RAMP_MS + ALIGN_HOLD_MS){
        Align_Fail(reason);              //满500ms爬升和原300ms观察后仍不合格，才结束本次尝试；不提前消耗补试次数。
    }
}

/*
 * 函数功能：在安全临界区内启动一轮完整六阶段定位，所有候选必须来自本次尝试。
 * 输入参数：now为当前毫秒时间。
 * 返回参数：无。
 */
static void Align_Begin(uint32_t now)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                      //避免检查PB12后被硬停中断打断再重新使能PWM。
    if(App.Err == E_NONE0 &&
       (AlignState == ALIGN_WAIT_ZERO || AlignState == ALIGN_RETRY_OFF) && //BREAK已改变状态时不能重新发布旧的定位请求。
       Align_SafeToStart() != 0U &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
        TIM_PWM_Stop();                   //清除可能由被BREAK打断的旧PWM服务留下的CCR，再从零使能。
        AlignAttempt++;                  //本轮次数在出波前累计。
        AlignState = ALIGN_B_FIRST;       //先吸合B相，不能提前清编码器。
        AlignPhaseTick = now;
        AlignAttemptTick = now;
        AlignPower = 0.0f;                //每轮重新从零电压爬升。
        AlignSampleValid = 0U;            //每次尝试必须采到自己的电流/电压，首次失败记录则跨补试保留。
        AlignDeltaForward = 0;            //新尝试的去程数据重新建立，不把前次位移带入快照。
        AlignDeltaReturn = 0;             //新尝试的回程数据从未测状态开始，由阶段字段区分有效性。
        AlignReturnError = 0;             //新的A相候选不能使用上次回位误差。
        AlignFirstB = 0;                  //首段参考本轮重新采集，不能沿用上次受扰位置。
        AlignFirstA = 0;                  //第一A候选在到位前无效，必须重新测量。
        AlignCheckB = 0;                  //正B复查不复用旧尝试的测量值。
        AlignSecondA = 0;                 //第二A候选只能由本轮正B返回后建立。
        AlignOppositeB = 0;               //另一侧参考只能由本轮反B吸合建立。
        AlignCandidate.phase_mask = 0U;   //撤销旧候选完整性，六个阶段必须逐个重新通过。
        AlignCandidate.sample_valid_mask = 0U; //本轮各阶段的采样有效性重新记录，不借用上轮电流。
        AlignCandidate.valid = 0U;        //候选工作区始终未发布，只在最终成功快照中置有效标志。
        AlignTableIndex = ALIGN_STEP_COUNTS; //B相正电压对应四分之一电周期。
        AlignPreviousIndex = ALIGN_STEP_COUNTS; //每次首轮或补试重建起点，首段从零爬升，不继承上次故障前输出。
        Align_ResetStill(now);
        App.Start = 0U;
        App.AllRun = 0U;
        App.Log.Set_Speed = 0;            //绝不携带任何待执行启动命令。
        App.Status = CTLS_ALIGN;          //PWM服务进入独立定位输出分支。
        TIM_PWM_Start();                 //上次关闭已清空CCR，第一次中断从零幅值建立输出。
        HAL_TIM_Base_Start_IT(&htim1);    //PWM计数器关闭后恢复采样和定位输出中断。
    }
    if(primask == 0U){ __enable_irq(); }
}

/*
 * 函数功能：六阶段及双侧A候选全部通过后建立零点，记录成功质量并关闭PWM待机。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Align_Finish(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                      //零点、PWM关闭及授权状态在同一临界区发布。
    if(AlignCandidate.phase_mask != (uint8_t)((1U << ALIGN_PHASE_COUNT) - 1U)){
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err : E_ALIGN6); //缺任何阶段都禁止清编码器，保留更早的真实保护。
        if(primask == 0U){ __enable_irq(); } //异常返回前恢复进入状态，不能遗留全局关中断。
        return;                          //未完整验证的候选不能发布为READY或成功快照。
    }
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN &&
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
        TIM2->CNT = 0U;                   //仅六阶段通过后的最终正A位置可成为零点，不平均掩盖两侧差异。
        TIM_PWM_Stop();                   //定位结束不保持加力，编码器继续记录后续回弹。
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
           __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
            App.Spd.En_Cnt = 0U;
            App.Spd.En_Cnt_Last = 0U;     //清测速基准，避免清零动作被误计为速度突变。
            App.Spd.Now = 0.0f;
            App.Spd.Now_Fil = 0U;
            App.Spd.EnTime_Cnt = 1U;
            App.Status = CTLS_WAIT;
            AlignPower = 0.0f;
            AlignState = ALIGN_READY;
            AlignCandidate.tick_ms = HAL_GetTick(); //记录真正提交成功的时刻，而不是首个A候选时刻。
            AlignCandidate.duration_ms = AlignCandidate.tick_ms - AlignAttemptTick; //只统计本次成功尝试的六阶段时长。
            AlignCandidate.attempt = AlignAttempt; //与首个失败记录并存，可明确是第几次补试通过。
            AlignLastSuccess.valid = 0U;  //复制期间保持未发布，外部不能读取半份成功数据。
            AlignLastSuccess = AlignCandidate; //在当前短临界区内一次复制六阶段诊断，不进行串口或Flash写入。
            __DMB();                      //先提交完整位置/质量/采样，再发布可读标志。
            AlignLastSuccess.valid = 1U;  //仅到此处表示全部新判据通过，不能等同绝对磁零点已获台架验证。
            AlignValid = 1U;             //最后发布已验证基准；后续普通运行保护仍沿用原零速恢复流程。
            SerUart3.RxEnd = 0U;          //丢弃完成边界前的旧命令快照。
            SerUart3.NeedZeroRearm = 1U;  //必须再收到新的零速帧，然后才能接受新的非零命令。
            Protect_ResetCounters();
        }else{
            Ctl_HardStop(E_Hardware);    //关闭边界出现BREAK时不得发布定位成功。
        }
    }else{
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err : E_Hardware);
    }
    if(primask == 0U){ __enable_irq(); }
}

/*
 * 函数功能：首段从零爬升，后续阶段在原时长内线性交接两相电压，避免换相失力且不提高电压上限。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_AlignOutput(void)
{
    uint32_t primask;                     //仅最终输出提交使用短临界区，时间计算不阻塞中断。
    float maximum = (float)MCPara[46] / 10.0f; //将参数46换算为V，默认60对应6.0V；补试使用相同定位电压上限。
    float previous_power;                //上一磁场的剩余幅值；与新磁场幅值之和不超过参数46，避免两相同时满幅。
    uint32_t ramp_ms = ALIGN_RAMP_MS;     //原参数51×52给出真实爬升时间，不依赖PWM_FRE注释或中断次数。
    uint32_t elapsed = HAL_GetTick() - AlignPhaseTick; //同一阶段按毫秒时钟计时，PWM频率变化不再加快定位爬升。
    if(App.Err != E_NONE0 || AlignState < ALIGN_B_FIRST || AlignState > ALIGN_A_VERIFY){
        TIM_PWM_Stop();                   //被异步故障撤销后不能再写非零定位输出。
        return;
    }
    if(!(App.Cal.CurrentT >= 0.0f)){
        Ctl_HardStop(E_OC1);              //无效采样仍硬停；有效电流按原母线阈值和累计延时保护，不作相电流单点判决。
        return;
    }
    if(ramp_ms == 0U){
        Ctl_HardStop(E_ALIGN6);           //错误时间参数不得产生除零或直接满幅吸合。
        return;
    }
    if(elapsed > ramp_ms){ elapsed = ramp_ms; } //爬升完成后维持原参数46上限，不随持续输出继续增大。
    primask = __get_PRIMASK();
    __disable_irq();                      //防止BREAK清CCR后，返回此处又写回非零比较值。
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN &&
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
        if((AlignSampleValid == 0U || AlignSampleTick != HAL_GetTick()) &&
           App.Cal.Current1 >= -FLT_MAX && App.Cal.Current1 <= FLT_MAX &&
           App.Cal.Current2 >= -FLT_MAX && App.Cal.Current2 <= FLT_MAX &&
           App.Cal.voltage >= 0.0f && App.Cal.voltage <= FLT_MAX){ //每毫秒最多保留一次有限值样本；不增加相电流跳闸门槛。
            AlignSampleA = App.Cal.Current1; //在已有输出临界区内保存完整样本，BREAK不能读到半更新数据。
            AlignSampleB = App.Cal.Current2; //保留相电流正负号，后续可区分励磁方向和偏置。
            AlignSampleVoltage = App.Cal.voltage; //和相电流一起记录供电情况，不使用电源面板值代替。
            AlignSampleTick = HAL_GetTick(); //记录样本时刻，无效新样本不会覆盖旧的有效记录。
            AlignSampleValid = 1U;         //所有样本字段写完后才允许失败快照引用。
        }
        AlignPower = maximum * (float)elapsed / (float)ramp_ms; //新磁场仍按500ms爬升，满幅后才允许累计定位稳定窗口。
        previous_power = (AlignState == ALIGN_B_FIRST) ? 0.0f : maximum - AlignPower; //首段保持零起步；后续逐渐撤去上一相，消除换相零励磁窗口。
        Set_OutPut_A(AlignPower * cos_table[AlignTableIndex] + previous_power * cos_table[AlignPreviousIndex], App.Cal.voltage); //A相提交前后磁场加权值，沿原顺序平滑转向，不使用未校准编码器角度。
        Set_OutPut_B(AlignPower * sin_table[AlignTableIndex] + previous_power * sin_table[AlignPreviousIndex], App.Cal.voltage); //B相同步交接，反B仍保持负向；正交向量合成幅值不超过定位上限。
    }else{
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err : E_Hardware); //关功率并保留先发生的保护。
    }
    if(primask == 0U){ __enable_irq(); }
}

/*
 * 函数功能：稳定且位移合格后提前推进六阶段双侧定位；慢响应保留原观察预算，有限重试后仍失败则锁存。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_AlignProcess(void)
{
    uint32_t now = HAL_GetTick();          //独立于TIM1的毫秒节拍，关闭PWM后仍能推进等待。
    uint32_t elapsed;
    uint32_t primask;                     //只保护阶段发布，不将整个毫秒处理置于关中断区。
    uint8_t state;                        //保存本次阶段，拒绝覆盖异步故障改变的状态。
    int16_t count;
    int16_t step_delta;                   //新增首段及反B往返的实际有符号位移，不借用旧去程测量。
    if(now == AlignLastTick){ return; }   //主循环重复调用不能把计时加快。
    AlignLastTick = now;
    if((TIM1->CR1 & TIM_CR1_CEN) == 0U){
        Motor_UpdateSamples();           //停PWM期间用仍运行的ADC DMA刷新保护依据。
    }
    if(AlignValid != 0U || AlignState == ALIGN_FAILED){ return; } //正常运行和失败锁存不自动定位。
    if(App.Err != E_NONE0 || AlignZeroFrames < ALIGN_ZERO_FRAMES ||
       SerUart3.CommandSeen == 0U || (uint32_t)(now - SerUart3.LastValidTick) > UART_COMMAND_TIMEOUT_MS){
        Align_ResetStill(now);           //安全窗口必须建立在持续合法的零速通信上。
        return;                          //通信超时由原解析器硬停，不能重建启动授权。
    }
    if(AlignState == ALIGN_WAIT_ZERO || AlignState == ALIGN_RETRY_OFF){
        if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET){
            Ctl_HardStop(E_Hardware);    //保留短BREAK脉冲证据，不能无报警地永远等待出波。
            return;
        }
        if(Align_SafeToStart() == 0U){
            Align_ResetStill(now);       //PB12、电压或电流恢复前保持关闭。
            return;
        }
        if(AlignState == ALIGN_RETRY_OFF && (uint32_t)(now - AlignOffTick) < ALIGN_RETRY_OFF_MS){ return; }
        if(Align_IsStill(now, ALIGN_RELEASE_MS) != 0U){
            if(MCPara[46] == 0U || MCPara[46] > 240U || App.Prot.OverCurrent <= 0.0f ||
               ALIGN_RAMP_MS == 0U || ALIGN_HOLD_MS == 0U ||
               ALIGN_RAMP_MS > (0xFFFFFFFFU / ALIGN_PHASE_COUNT) - ALIGN_HOLD_MS - ALIGN_PHASE_EXTRA_MS){ //先约束爬升值，避免阶段求和或六阶段总时限乘法溢出。
                AlignAttempt = ALIGN_MAX_ATTEMPTS; //无效参数不得出波或反复重试。
                Align_Fail(5U);
            }else{
                Align_Begin(now);        //两帧零速加50ms连续物理静止之后才能首次出波。
            }
        }
        return;
    }
    if(App.Status != CTLS_ALIGN){ return; } //中断硬停后主循环不得推进旧的定位阶段。
    state = AlignState;                   //后续判断使用同一个阶段快照。
    if(state < ALIGN_B_FIRST || state > ALIGN_A_VERIFY){
        Ctl_HardStop(E_ALIGN6);           //新增六阶段记录使用状态作为下标，异常状态必须硬停，不能越界写候选。
        return;                          //只有六个明确的加力阶段可以进入稳定性和几何检查。
    }
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_RESET){
        Ctl_HardStop(E_Hardware);
        return;
    }
    if(!(App.Cal.voltage >= App.Prot.Allow_Voltage)){
        Ctl_HardStop(E_UV2);              //定位中电压不足立即中止，不能用失去力量的位置作为零点。
        return;
    }
    if(!(App.Cal.voltage <= App.Prot.OverVoltage - 1.0f)){
        Ctl_HardStop(E_OV3);
        return;
    }
    elapsed = now - AlignPhaseTick;
    if(elapsed >= ALIGN_PHASE_MAX_MS || (uint32_t)(now - AlignAttemptTick) >= ALIGN_PHASE_COUNT * ALIGN_PHASE_MAX_MS){
        Align_Fail(1U);                  //超时也不能清零；最多安排一次补试。
        return;
    }
    if(elapsed < ALIGN_RAMP_MS || AlignPower <= 0.1f){
        Align_ResetStill(now);            //磁场尚未到位或无有效输出时不能累计末段静止。
        return;
    }
    if(Align_IsSettled(now) == 0U){ return; } //末段整体摆幅和连续100ms均合格后才允许检查位置，阶段截止不重置。
    if(elapsed < ALIGN_RAMP_MS + ALIGN_STABLE_MS){ return; } //至少500ms爬升和满幅后100ms质量观察，合格后不再空等原300ms；慢响应仍受820ms截止约束。
    count = (int16_t)TIM2->CNT;            //只使用原始计数，停机路径会清除滤波速度。
    if(state == ALIGN_B_FIRST){
        AlignFirstB = count;             //先保存首个B参考，下一段必须证明从这里真实向A移动。
    }else if(state == ALIGN_A_FIRST){
        step_delta = Align_Delta(count, AlignFirstB); //补齐以往漏检的首B到首A，受扰首段不能被后面的往返掩盖。
        if(abs(step_delta + ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_WaitOrFail(7U, elapsed); //首B到A不合格先保留原观察时间，不能只凭后续同侧重复放行。
            return;                     //首个A位置不合格，不能作为后面各阶段的基准。
        }
        AlignFirstA = count;             //暂存候选位置，后续仍必须验证实际可移动。
    }else if(state == ALIGN_B_CHECK){
        AlignDeltaForward = Align_Delta(count, AlignFirstA);
        if(abs(AlignDeltaForward - ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_WaitOrFail(2U, elapsed); //卡住、反向或位移不足先等原观察预算，仍不合格才拒绝，不放宽位移门槛。
            return;
        }
        AlignCheckB = count;
    }else if(state == ALIGN_A_CHECK){
        AlignDeltaReturn = Align_Delta(count, AlignCheckB);
        AlignReturnError = Align_Delta(count, AlignFirstA);
        if(abs(AlignDeltaReturn + ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_WaitOrFail(3U, elapsed); //回程不合格先等待慢响应，达到原观察时长后仍不回位才失败。
            return;
        }
        if(abs(AlignReturnError) > ALIGN_RETURN_TOLERANCE){
            Align_WaitOrFail(4U, elapsed); //回到同一A的重复性不合格时保留原观察时间，不能提前认定零点可靠。
            return;
        }
        AlignSecondA = count;            //同侧重复合格仍只是候选，必须追加另一侧返回验证。
    }else if(state == ALIGN_B_OPPOSITE){
        step_delta = Align_Delta(count, AlignSecondA); //从A负向移动到反B，检查另一侧确实可以响应磁场。
        if(abs(step_delta + ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_WaitOrFail(8U, elapsed); //另一侧位移未合格时继续原观察预算，不提前进入最终零点确认。
            return;                     //反B参考不可信，禁止用后续正A返回掩盖本段异常。
        }
        AlignOppositeB = count;          //保存反B实际到位位置，最终返回必须以它计算正向位移。
    }else if(state == ALIGN_A_VERIFY){
        step_delta = Align_Delta(count, AlignOppositeB); //从反B正向返回同一A，增加不同于前两次同侧返回的证据。
        if(abs(step_delta - ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_WaitOrFail(9U, elapsed); //最后返回未合格时先等原观察时间，不能为加快定位而发布错误零点。
            return;                     //保留编码器原始计数，等待有界补试或明确重定位。
        }
        if(abs(Align_Delta(count, AlignFirstA)) > ALIGN_RETURN_TOLERANCE ||
           abs(Align_Delta(count, AlignSecondA)) > ALIGN_RETURN_TOLERANCE){
            Align_WaitOrFail(4U, elapsed); //最终A不一致时先等原观察预算，仍不能将两侧不一致平均后放行。
            return;                     //重复位置存在方向依赖，零点继续无效。
        }
    }
    primask = __get_PRIMASK();
    __disable_irq();                      //BREAK可随时撤销校准，不能被随后一次阶段递增覆盖。
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN && AlignState == state){
        Align_RecordPhase((uint8_t)(state - ALIGN_B_FIRST), count, now); //与阶段提交同一临界区收集位置和完整电流样本。
        if(state == ALIGN_A_VERIFY){
            Align_Finish();               //最后一段也已记录且全部几何判据通过，才清编码器并发布成功。
        }else{
            AlignState = state + 1U;      //按正B/正A/正B/正A/反B/正A顺序推进，不跳过另一侧确认。
            AlignPhaseTick = now;         //每个阶段仍限制为原500/300ms加20ms余量，失败不会无限延时。
            AlignPower = 0.0f;            //仅重置新磁场爬升进度，输出由上一相连续交接，不实际撤去全部励磁。
            AlignSampleValid = 0U;        //新阶段必须采集自己的样本，未采到时诊断有效位保持0。
            AlignPreviousIndex = AlignTableIndex; //临界区内先保存刚通过校验的磁场方向，再发布下一目标方向。
            AlignTableIndex = (AlignState == ALIGN_B_CHECK) ? ALIGN_STEP_COUNTS :
                ((AlignState == ALIGN_B_OPPOSITE) ? (3U * ALIGN_STEP_COUNTS) : 0U); //反B为四分之三电周期，其余新增返回仍为正A。
            Align_ResetStill(now);        //各阶段重新累计稳定质量，不能借用前一相的静止时间。
        }
    }
    if(primask == 0U){ __enable_irq(); }
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
