#include "motor.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "logic.h"
#include "protect.h"
#include <stdlib.h> //定位校验用有符号位移绝对值，不依赖滤波转速。

extern MCUART_Type SerUart3;
volatile uint16_t MCPara[ParaNum] ={0};

uint16_t SMCPara[ParaNum] = {

	0,		// -0  地址
	0,		// -1  电流极性							0 极性是正			1 极性是负
	1,		// -2  开/闭环控制方式				0 开环					1 闭环
						   
	100,	// -3  积分限幅				
	24,		// -4  输出限幅
	
	300,		// -5  兼容保留旧定位保持时间；新校准时限由motor.h的ALIGN宏控制。
	
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
	
	100,		// -44 兼容保留旧启动定位时间；已验证零点的普通启停不重复定位。
	4,			// -45 编码器偏置变化爬坡周期
	50,			// -46 开机启动定位的力量	/10			-130
	
	200,		// -47 停机转速
	
	3,		// -48 加速度-速度向上爬坡值			-3
	5,		// -49 减速度-速度向下爬坡值			-4
	20,		// -50 爬坡周期-速度爬坡计算周期		-10
	
	1,		// -51 兼容保留旧定位爬坡值；新校准按ALIGN_RAMP_MS及相电流限流。
	500,		// -52 兼容保留旧阻塞循环次数；新校准不执行该循环。
	
	
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
volatile float AlignPower = 0.0f;          //PWM节拍逐渐建立的实际电压幅值。
static uint8_t AlignZeroFrames = 0U;       //连续零速帧计数，非零命令或故障将其清零。
static uint32_t AlignLastTick = 0U;        //保证主循环只在毫秒改变后推进一次。
static uint32_t AlignPhaseTick = 0U;       //当前阶段的真实开始时间。
static uint32_t AlignAttemptTick = 0U;     //单轮加力的总超时基准。
static uint32_t AlignOffTick = 0U;         //轮间关闭PWM的起点。
static uint32_t AlignStillTick = 0U;       //固定编码器参考窗口开始时间。
static int16_t AlignStillAnchor = 0;       //防止每毫秒缓慢爬动被误当静止。
static int16_t AlignFirstA = 0;            //第一次A相定位的候选计数，尚未写入零点。
static int16_t AlignCheckB = 0;            //校验B相到达后的计数。
static volatile uint16_t AlignTableIndex = ALIGN_STEP_COUNTS; //静止坐标磁场方向，不能使用未经校准的编码器角度。

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
 * 函数功能：重新开始一个原始编码器稳定窗口。
 * 输入参数：now为当前毫秒时间。
 * 返回参数：无。
 */
static void Align_ResetStill(uint32_t now)
{
    AlignStillAnchor = (int16_t)TIM2->CNT; //固定参考而非相邻两点比较，缓慢滑动也会被发现。
    AlignStillTick = now;                 //窗口不继承上一相的稳定时间。
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
 * 函数功能：判断关闭PWM后的电压、电流和硬件输入是否允许定位。
 * 输入参数：无。
 * 返回参数：所有物理条件满足返回1，否则返回0。
 */
static uint8_t Align_SafeToStart(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
            App.Cal.voltage >= App.Prot.Allow_Voltage &&
            App.Cal.voltage <= App.Prot.OverVoltage - 1.0f &&
            App.Adc.Current_AllTemp >= 0.0f &&
            App.Cal.Current1 > -ALIGN_PHASE_SOFT_A && App.Cal.Current1 < ALIGN_PHASE_SOFT_A &&
            App.Cal.Current2 > -ALIGN_PHASE_SOFT_A && App.Cal.Current2 < ALIGN_PHASE_SOFT_A && //关PWM后相电流也必须回落，不能只看母线。
            App.Adc.Current_AllTemp <= App.Prot.OverCurrent * ALIGN_BUS_LIMIT_RATIO) ? 1U : 0U; //浮点比较也会拒绝NaN。
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
    SerUart3.NeedZeroRearm = 1U;          //校准通过前禁止普通启动。
    TIM_PWM_Stop();                       //编码器和ADC继续采样，只关闭功率输出。
    AlignLastTick = HAL_GetTick();        //从初始化完成时开始非阻塞调度。
    Align_ResetStill(AlignLastTick);      //等待阶段从当前真实位置确认静止。
}

/*
 * 函数功能：故障硬停时使角度基准失效，并限制故障中的自动加力。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_AlignOnFault(void)
{
    uint8_t wait_release = (AlignValid != 0U ||
        (AlignState == ALIGN_WAIT_ZERO && AlignAttempt == 0U)) ? 1U : 0U; //正常运行故障可在零速释放后校准；校准中再故障则锁住本轮。
    AlignValid = 0U;                      //真实故障不能留下可启动的旧零点。
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
        if(AlignState == ALIGN_FAILED && App.Err == E_NONE0 && SerUart3.NeedZeroRearm == 0U){
            AlignState = ALIGN_WAIT_ZERO; //失败已被零速确认后，新启动只申请新校准，不直接出波。
            AlignAttempt = 0U;           //只有这个新的请求才能复位本轮次数上限。
        }
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err : E_ALIGN6); //拒绝提前启动并通知主控返回零速，绝不缓存目标速度。
        AlignFailure = 6U;               //区分操作打断与编码器/过流失败。
        return 1U;
    }
    if(App.Err != E_NONE0){
        if(Align_SafeToStart() == 0U ||
           (App.Err != E_ALIGN6 && Protect_CanClearFault(App.Err) == 0U)){
            return 1U;                   //物理条件未恢复时保留原故障和关闭状态。
        }
        Protect_ResetCounters();         //新的零速握手仅清保护累计值，不设置AlignValid。
        Ctl_HardStop(E_NONE0);            //故障确认时功率保持关闭。
        if(App.Err != E_NONE0){ return 1U; } //清理边界出现新的BREAK时不发布任何零速授权。
        SerUart3.NeedZeroRearm = (AlignState == ALIGN_FAILED) ? 0U : 1U; //失败锁存只允许后续申请校准，不能运行。
    }
    if(AlignState == ALIGN_FAILED){
        return 1U;                       //已耗尽重试次数，持续零速不得再自动加力。
    }
    if(AlignZeroFrames < ALIGN_ZERO_FRAMES){
        AlignZeroFrames++;               //最多记录两帧，帧的新鲜度由100ms租约保证。
    }
    return 1U;                           //定位中合法零速只续约，不执行普通硬停。
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
        AlignState = ALIGN_FAILED;        //次数耗尽后持续零速只能清报警，不能重复加力。
        AlignFailure = reason;
    }
    if(primask == 0U){ __enable_irq(); }  //恢复进入前的中断状态。
}

/*
 * 函数功能：在安全临界区内启动一轮B/A/B/A定位。
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
        AlignTableIndex = ALIGN_STEP_COUNTS; //B相正电压对应四分之一电周期。
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
 * 函数功能：在最终A相保持且往返校验通过后建立零点，再关闭PWM待机。
 * 输入参数：无。
 * 返回参数：无。
 */
static void Align_Finish(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                      //零点、PWM关闭及授权状态在同一临界区发布。
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN &&
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
        TIM2->CNT = 0U;                   //仅已通过往返验证的A相位置可成为新电角度零点。
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
            AlignValid = 1U;             //最后发布有效零点；中断恢复后真实故障仍能撤销。
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
 * 函数功能：在PWM节拍内限制定位电流并生成与编码器零点无关的A/B电压。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_AlignOutput(void)
{
    uint32_t primask;                     //仅最终输出提交使用短临界区，限流计算不阻塞中断。
    float maximum = (float)MCPara[46] / 10.0f; //沿用现有5V上限，补试不自动提高力量。
    float step = maximum * 1000.0f / ((float)ALIGN_RAMP_MS * PWM_FRE); //幅值约25ms从零爬到设定值。
    float phase1 = App.Cal.Current1 >= 0.0f ? App.Cal.Current1 : -App.Cal.Current1;
    float phase2 = App.Cal.Current2 >= 0.0f ? App.Cal.Current2 : -App.Cal.Current2;
    if(App.Err != E_NONE0 || AlignState < ALIGN_B_FIRST || AlignState > ALIGN_A_CHECK){
        TIM_PWM_Stop();                   //被异步故障撤销后不能再写非零定位输出。
        return;
    }
    if(!(App.Adc.Current_AllTemp >= 0.0f) ||
       App.Adc.Current_AllTemp >= App.Prot.OverCurrent ||
       !(phase1 < ALIGN_PHASE_LIMIT_A) || !(phase2 < ALIGN_PHASE_LIMIT_A)){
        Ctl_HardStop(E_OC1);              //任一相达1.2A或母线达原过流阈值立即硬停，不等待运行500ms计时。
        return;
    }
    if(App.Adc.Current_AllTemp >= App.Prot.OverCurrent * ALIGN_BUS_LIMIT_RATIO ||
       phase1 >= ALIGN_PHASE_SOFT_A || phase2 >= ALIGN_PHASE_SOFT_A){
        AlignPower -= step * 2.0f;        //电流接近上限时优先降低电压，不能持续顶着阈值加力。
        if(AlignPower < 0.0f){ AlignPower = 0.0f; }
    }else{
        AlignPower += step;              //允许范围内逐渐建立原定位力量。
        if(AlignPower > maximum){ AlignPower = maximum; }
    }
    primask = __get_PRIMASK();
    __disable_irq();                      //防止BREAK清CCR后，返回此处又写回非零比较值。
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN &&
       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
       __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
        Set_OutPut_A(AlignPower * cos_table[AlignTableIndex], App.Cal.voltage); //相位由定位程序给定，不用未知的编码器角度。
        Set_OutPut_B(AlignPower * sin_table[AlignTableIndex], App.Cal.voltage);
    }else{
        Ctl_HardStop(App.Err != E_NONE0 ? App.Err : E_Hardware); //关功率并保留先发生的保护。
    }
    if(primask == 0U){ __enable_irq(); }
}

/*
 * 函数功能：非阻塞推进安全等待、B/A/B/A校验、有限重试及完成待机。
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
            if(MCPara[46] == 0U || MCPara[46] > 240U || App.Prot.OverCurrent <= 0.0f){
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
    if(elapsed >= ALIGN_PHASE_MAX_MS || (uint32_t)(now - AlignAttemptTick) >= 4U * ALIGN_PHASE_MAX_MS){
        Align_Fail(1U);                  //超时也不能清零；最多安排一次补试。
        return;
    }
    if(state == ALIGN_B_FIRST){
        AlignTableIndex = ALIGN_STEP_COUNTS; //首次B相只爬升电压，初始位置未知不直接验证位移。
    }else{
        uint32_t travel = elapsed < ALIGN_RAMP_MS ? elapsed : ALIGN_RAMP_MS;
        uint16_t fraction = (uint16_t)(ALIGN_STEP_COUNTS * travel / ALIGN_RAMP_MS);
        AlignTableIndex = state == ALIGN_B_CHECK ? fraction : ALIGN_STEP_COUNTS - fraction; //两相连续过渡，减少磁场突然跳变。
    }
    if(elapsed < ALIGN_RAMP_MS || AlignPower <= 0.1f){
        Align_ResetStill(now);            //磁场尚未到位或无有效输出时不能累计末段静止。
        return;
    }
    if(Align_IsStill(now, ALIGN_STABLE_MS) == 0U){ return; }
    count = (int16_t)TIM2->CNT;            //只使用原始计数，停机路径会清除滤波速度。
    if(state == ALIGN_A_FIRST){
        AlignFirstA = count;             //暂存候选位置，后续仍必须验证实际可移动。
    }else if(state == ALIGN_B_CHECK){
        AlignDeltaForward = Align_Delta(count, AlignFirstA);
        if(abs(AlignDeltaForward - ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_Fail(2U);              //包括卡住、反向或移动不足。
            return;
        }
        AlignCheckB = count;
    }else if(state == ALIGN_A_CHECK){
        AlignDeltaReturn = Align_Delta(count, AlignCheckB);
        AlignReturnError = Align_Delta(count, AlignFirstA);
        if(abs(AlignDeltaReturn + ALIGN_ENCODER_SIGN * (int16_t)ALIGN_STEP_COUNTS) > ALIGN_STEP_TOLERANCE){
            Align_Fail(3U);
            return;
        }
        if(abs(AlignReturnError) > ALIGN_RETURN_TOLERANCE){
            Align_Fail(4U);              //能移动但未回到同一A位置仍不能认定零点可靠。
            return;
        }
        Align_Finish();
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();                      //BREAK可随时撤销校准，不能被随后一次阶段递增覆盖。
    if(App.Err == E_NONE0 && App.Status == CTLS_ALIGN && AlignState == state){
        AlignState = state + 1U;          //B预定位→A候选→B检查→A检查，仅提交未被撤销的快照。
        AlignPhaseTick = now;             //每个阶段单独限制最长加力时间。
        Align_ResetStill(now);            //上一相静止时间不能借给下一相。
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
