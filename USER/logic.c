#include "logic.h"
#include "main.h"
#include "usart.h"
#include "tim.h" //启停提交前核对TIM1 BREAK标志，显式引用定时器句柄声明。
#include <stdlib.h>
#include "protect.h"
#include "motor.h"

MCUART_Type SerUart3;
volatile  int text_Speed = 0;
extern uint8_t Tx_text[5];
/*
 * 函数功能：结合命令、电压、故障和已验证零点建立运行条件。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Logic(void){
	//根据速度判断转向
	if(App.Log.Set_Speed >0){
		MCPara[6]=0;
	}else if(App.Log.Set_Speed <0){
		MCPara[6]=1;
	}else{
	}
	
	if( App.Cal.voltage > App.Prot.Allow_Voltage
		&& (abs(App.Log.Set_Speed)>MCPara[35])
		&& App.Err == E_NONE0
		&& AlignValid != 0U                //零速清故障不能替代定位校验，未知零点始终禁止启动。
		&& SerUart3.NeedZeroRearm == 0U     //定位完成边界后仍需新的零速握手。
	){
		App.Start =1;
	}else{
		App.Start =0;
	}

	STA_Led(App.Err);
	
}



/*
 * 函数功能：立即撤销全部运动授权并关闭四路PWM，故障停机不等待编码器速度下降。
 * 输入参数：fault_code为需要锁存的故障码，E_NONE0表示正常零速停机。
 * 返回参数：无。
 */
void Ctl_HardStop(uint8_t fault_code)
{
	uint32_t primask = __get_PRIMASK();      //硬停状态一次发布，不能被更高优先级BREAK打断后覆盖。
	__disable_irq();                        //仅保护寄存器与控制状态清理，不含延时或通信等待。
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_RESET ||
	   __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET){
		fault_code = E_Hardware;             //硬件输入或待处理刹车标志始终优先于普通清故障。
	}
	App.Start = 0U;                         //先撤销启动条件，阻止状态机再次进入初始化。
	App.AllRun = 0U;                        //先关闭高频控制计算，避免停PWM时中断继续改写CCR。
	App.Log.Set_Speed = 0;                  //清除永久保存的目标速度，失联后不能沿用旧命令。
	App.Spd.Set_SpdTemp = 0.0f;             //清除速度爬坡值，重新启动必须从零开始。
	App.Spd.RampUpDown_Cycle = 0U;          //清除速度爬坡周期，避免恢复后继承旧节拍。
	App.Openloop.Step_Unit = 0.0f;          //清除开环相位增量，停止换相。
	App.Openloop.Set_Step_Unit = 0.0f;      //清除开环目标相位增量，禁止旧速度重新生效。
	App.Openloop.RampUpTemp = 0.0f;         //清除开环加速缓存。
	App.Openloop.RampUpTimeCnt = 0.0f;      //清除开环启动计时。
	App.Openloop.Stage = 0U;                //下一次合法启动重新从开环初始阶段进入。
	App.Cal.V_a = 0.0f;                     //清除A相目标电压。
	App.Cal.V_b = 0.0f;                     //清除B相目标电压。
	mcApp_D_PIParam.qdSum = 0.0f;           //清除D轴积分，防止旧积分在复位后冲击输出。
	mcApp_D_PIParam.qOut = 0.0f;            //清除D轴输出缓存。
	mcApp_D_PIParam.qInRef = 0.0f;          //清除D轴参考值。
	mcApp_Q_PIParam.qdSum = 0.0f;           //清除Q轴积分，防止旧积分在复位后冲击输出。
	mcApp_Q_PIParam.qOut = 0.0f;            //清除Q轴输出缓存。
	mcApp_Q_PIParam.qInRef = 0.0f;          //清除Q轴参考值。
	mcApp_Spd_PIParam.qdSum = 0.0f;         //清除速度环积分。
	mcApp_Spd_PIParam.qOut = 0.0f;          //清除速度环输出缓存。
	mcApp_Spd_PIParam.qInRef = 0.0f;        //清除速度环参考值。
	TIM_PWM_Stop();                         //清零CCR并关闭CC、MOE和TIM1计数器。
	if(fault_code != E_NONE0 &&
	   (AlignValid == 0U || fault_code == E_OC1 || fault_code == E_Hardware)){
		Motor_AlignOnFault();                //定位中故障及运行过流/BREAK撤销零点，普通零速不丢失基准。
	}
	App.Err = fault_code;                   //最后发布故障码，反馈帧和状态灯读取同一锁存结果。
	App.Status = (fault_code == E_NONE0) ? CTLS_WAIT : CTLS_ERROR; //正常停机回等待，故障停机保持锁存。
	if(fault_code != E_NONE0){
		SerUart3.NeedZeroRearm = 1U;         //故障后拒绝直接恢复非零命令，必须先收到合法零速帧。
	}
	if(primask == 0U){ __enable_irq(); }    //恢复进入前状态，ISR中的调用不会提前开中断。
}

/*
 * 函数功能：在短临界区内取走ISR封存的帧，避免DMA和主循环同时访问同一数据块。
 * 输入参数：frame接收6字节快照，length接收本次真实帧长。
 * 返回参数：有待处理快照返回1，否则返回0。
 */
static uint8_t UartCommand_TakeSnapshot(uint8_t *frame, uint16_t *length)
{
	uint32_t primask;                       //保存进入临界区前的全局中断状态。
	uint8_t index;                          //逐字节复制固定6字节快照。

	if((frame == NULL) || (length == NULL) || (SerUart3.RxEnd == 0U)){
		return 0U;                            //参数无效或当前没有封存帧时保持通信状态不变。
	}

	primask = __get_PRIMASK();              //记录调用前是否已经关闭中断。
	__disable_irq();                        //复制期间禁止USART3中断替换快照。
	if(SerUart3.RxEnd == 0U){
		if(primask == 0U){
			__enable_irq();                    //只恢复本函数进入前处于开启状态的中断。
		}
		return 0U;                            //进入临界区前帧已被其它路径取走。
	}
	*length = SerUart3.RxLen;               //先锁存实际长度，解析器必须按该长度判断。
	if(*length == UART_COMMAND_FRAME_SIZE){
		for(index = 0U; index < UART_COMMAND_FRAME_SIZE; index++){
			frame[index] = SerUart3.RxFrame[index]; //只复制ISR已经封存完成的6字节数据。
		}
	}
	SerUart3.RxEnd = 0U;                    //快照取走后释放单槽邮箱，允许ISR发布下一帧。
	__DMB();                                //确保清标志动作不会越过前面的数据读取。
	if(primask == 0U){
		__enable_irq();                       //主循环调用时恢复中断，ISR内调用时保持原状态。
	}
	return 1U;
}

/*
 * 函数功能：严格校验一帧泵控制命令，坏帧不得改变速度或刷新通信租约。
 * 输入参数：frame为封存帧，length为真实长度，direction和speed返回已校验字段。
 * 返回参数：帧头、长度、CRC、方向和速度范围全部合法返回1，否则返回0。
 */
static uint8_t UartCommand_IsValid(const uint8_t *frame,
                                   uint16_t length,
                                   uint8_t *direction,
                                   uint16_t *speed)
{
	uint16_t calc_crc;                       //保存前4字节计算得到的Modbus CRC。
	uint16_t received_crc;                   //保存帧内低字节在前的CRC值。

	if((frame == NULL) || (direction == NULL) || (speed == NULL)){
		return 0U;                             //空指针不能参与协议解析。
	}
	if((length != UART_COMMAND_FRAME_SIZE) || (frame[0] != 0xAAU)){
		return 0U;                             //短帧、长帧和错误帧头全部丢弃，禁止复用旧尾部。
	}
	if(frame[1] > 1U){
		return 0U;                             //方向只能为0或1，非法方向不能保留旧目标继续运行。
	}
	received_crc = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8U); //协议CRC按低字节、高字节发送。
	calc_crc = CRC_Calc((unsigned char *)frame, 4U); //只校验帧头、方向和速度两个字节。
	if(calc_crc != received_crc){
		return 0U;                             //生产协议不再接受BB AA测试尾旁路。
	}
	*speed = ((uint16_t)frame[2] << 8U) | (uint16_t)frame[3]; //先按无符号16位组合速度幅值。
	if(*speed > UART_COMMAND_MAX_SPEED_RPM){
		return 0U;                             //超过当前整机630最大合法值时直接拒绝，不能只在输出端饱和。
	}
	*direction = frame[1];                  //全部校验通过后才发布方向字段。
	return 1U;
}

/*
 * 函数功能：检查最近合法命令年龄，通信静默超过100ms时立即锁存通信故障并硬停。
 * 输入参数：无。
 * 返回参数：无。
 */
static void UartCommand_CheckTimeout(void)
{
	uint32_t now_tick = HAL_GetTick();       //HAL毫秒节拍天然支持无符号回绕差值计算。
	uint32_t command_age = now_tick - SerUart3.LastValidTick; //计算距最近合法命令的时间。

	if(((SerUart3.CommandSeen == 0U) || (command_age > UART_COMMAND_TIMEOUT_MS)) &&
	   ((App.Err == E_NONE0) || (App.AllRun != 0U) || (App.Log.Set_Speed != 0))){
		uint8_t stop_fault = (App.Err == E_NONE0 || App.Err == E_COMM5) ? E_COMM5 : App.Err; //不覆盖已经锁存的硬件保护故障。
		Ctl_HardStop(stop_fault);              //超时不做软减速，立即关闭功率并要求零速重新授权。
	}
}

/*
 * 函数功能：处理一帧封存的泵命令并维护驱动端100ms命令租约。
 * 输入参数：无。
 * 返回参数：无。
 */
void UartDealResponse(void)
{
	uint32_t primask;                       //命令只在校验后短暂原子提交，DMA反馈仍在开中断状态执行。
	uint8_t frame[UART_COMMAND_FRAME_SIZE];  //主循环只解析局部不可变快照。
	uint16_t frame_length = 0U;              //记录ISR上报的真实接收长度。
	uint8_t direction = 0U;                  //保存通过校验的方向字段。
	uint16_t speed = 0U;                     //保存通过校验的速度幅值。

	if(UartCommand_TakeSnapshot(frame, &frame_length) != 0U){
		if(UartCommand_IsValid(frame, frame_length, &direction, &speed) != 0U){
			primask = __get_PRIMASK();         //防止零速清故障与校准撤销在同一帧中互相覆盖。
			__disable_irq();                   //仅保护下面的目标/授权状态更新。
			SerUart3.LastValidTick = HAL_GetTick(); //只有完整合法帧能够刷新通信租约。
			SerUart3.CommandSeen = 1U;          //记录上电后已经建立有效通信。

			if(Motor_AlignCommand(speed) != 0U){
				/* 校准门禁已消费该帧：合法零速维持校准，非零请求不能被缓存为待运行目标。 */
			}
			else if(speed == 0U){
				if(Protect_CanClearFault(App.Err) != 0U){
					Protect_ResetCounters();        //安全输入恢复后清除旧保护计数，防止下一周期误复发。
					Ctl_HardStop(E_NONE0);          //零速帧始终执行硬停，停止不依赖编码器反馈。
					if(App.Err == E_NONE0){
						SerUart3.NeedZeroRearm = 0U; //只有硬停确认无新BREAK时才完成零速授权。
					}
				}
			}
			else if((SerUart3.NeedZeroRearm == 0U) && (App.Err == E_NONE0)){
				App.Log.Set_Speed = (direction == 0U) ? (int)speed : -(int)speed; //确认安全授权后才写运行目标。
				if(App.Log.Set_Speed * App.Spd.Set_SpdTemp < 0.0f){
					App.Log.Set_Speed = 0;          //换向时先把目标降到零，下一周期合法帧再建立反向目标。
				}
			}
			if(primask == 0U){ __enable_irq(); } //发送前恢复中断，通信不能占住功率保护路径。
			Return_Uart();                      //合法帧均返回实测速度和当前锁存故障码。
		}
		else{
			SerUart3.RxRejectCnt++;             //坏帧只计数，不改变目标、故障或命令时间戳。
		}
	}

	UartCommand_CheckTimeout();              //无论本周期是否收到数据都执行命令年龄检查。
}

//返回数据
void Return_Uart(void){
	uint16_t TxCRC = 0;
	uint8_t	 TxCRCL= 0;
	uint8_t	 TxCRCH= 0;
	SerUart3.T_DATA[0] = 0xAA;		//帧头
	SerUart3.T_DATA[1] = App.Spd.En_Dir;	//方向		0正转，1反转
	SerUart3.T_DATA[2] = (App.Spd.Now_Fil&0xFF00)/256;	
	SerUart3.T_DATA[3] = (uint8_t)(App.Spd.Now_Fil & 0x00FFU);	//低字节必须按256取值，不能使用模255。
	SerUart3.T_DATA[4] = App.Err;		//错误码返回
	TxCRC = CRC_Calc(SerUart3.T_DATA,5);
	TxCRCH = (uint8_t)((TxCRC & 0xFF00)>>8);
	TxCRCL = (uint8_t)(TxCRC & 0x00FF);
	
	SerUart3.T_DATA[5] =  TxCRCL;		//CRC低位
	SerUart3.T_DATA[6] =  TxCRCH;		//CRC高位
//		SerUart3.T_DATA[3] = 0xBB;
//		SerUart3.T_DATA[4] = 0xAA;
	HAL_UART_Transmit_DMA(&huart3,SerUart3.T_DATA,7);
}

//CRC校验
uint16_t CRC_Calc(unsigned char *snd, unsigned char num)
{
    unsigned char i, j;
    unsigned int c,crc=0xFFFF;
    for(i = 0; i < num; i ++)
    {
        c = snd[i] & 0x00FF;
        crc ^= c;
        for(j = 0;j < 8; j ++)
        { 
            if (crc & 0x0001)
            {
                crc>>=1;
                crc^=0xA001;
            }
            else crc>>=1;
        }
    }    
    return(crc);
}





/*
 * 函数功能：分发运行状态；独立定位状态不进入普通闭环初始化。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_Ctl(void){
	switch(App.Status){
		case CTLS_CLOSELOOP_RUN:		//闭环
				Ctl_CloseLoop();
					break;
		case CTLS_WAIT:				//等待
				Ctl_Wait();
					break;
		case CTLS_OPENLOOP_RUN:			//开环
				Ctl_OpenLoop();   
					break;
		case CTLS_ERROR:				//错误
				Ctl_Error();
					break;
		case CTLS_INIT:					//初始化
				Ctl_Init();
					break;
		case CTLS_STOP:     	//停止         
				Ctl_Stop();
					break;
		case CTLS_BRAKE:			//刹车
				Ctl_Brake();
					break;
		case CTLS_ALIGN:
				break;                      //定位由主循环毫秒状态机推进，不能执行运行初始化。
		default:
			  Ctl_Wait();   	//空闲等待          
					break;
	}
}



void Ctl_Wait(void){
	if(App.Start == 1){
		App.Status = CTLS_INIT;
	}
	if(App.Err > 0){
		App.Status = CTLS_ERROR; 
	}
}

/*
 * 函数功能：仅在定位有效且启动授权仍成立时初始化运行控制。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Init(void){
	uint32_t primask;                       //最后使能功率时再次原子核对授权，参数初始化本身不开功率。
	if(AlignValid == 0U || App.Err != E_NONE0 || SerUart3.NeedZeroRearm != 0U ||
	   App.Log.Set_Speed == 0){
		Ctl_HardStop(App.Err);               //再次核对入口，快速启停不能越过校准或故障门禁。
		return;
	}
	//编码器偏置初始化
	App.Log.Offset_Encode_Now = 0;
	App.Log.Offset_Encode_Set = 0;
	App.Spd.Now = 0;
	//
	App.Spd.Set_SpdTemp = 0;	//速度环爬坡值
	//开环参数初始化
	App.Openloop.Fre_Cnt = 100000/(200*MCPara[9]/60);				//以10us为单位去计算次数，1S=100000（10us）
	App.Openloop.k = (float)MCPara[10]/MCPara[12];
	App.Openloop.b = (float)MCPara[11]/MCPara[12];
	App.Openloop.Power_Goal = App.Openloop.k*MCPara[9] + App.Openloop.b;
	if(App.Openloop.Power_Goal <0){
		App.Openloop.Power_Goal = 0;
	}
	//缓存值
	App.Openloop.RampUpTemp =0;
	App.Openloop.RampUpTimeCnt = 0;
	App.Openloop.RampUpTemp = 0;
	App.Openloop.Step_Temp= 0;
	App.Openloop.Stage = 0;
	//开环启动时间
	App.Openloop.RampUpTime =(float)MCPara[25]/1000.0f*PWM_FRE;
	//开环加速度
	App.Openloop.Acceleration = (float)MCPara[26]/1000;
	if(MCPara[6] == 0){
		//开环力量增加的最小单位
		App.Openloop.PoweUpUnit	= (float)App.Openloop.Power_Goal /((MCPara[25]/1000.0f)*PWM_FRE);
		//开环累加最小单位计算
		App.Openloop.Step_Unit 	=(float)App.Log.Set_Speed* ENCODER_STEPS	/60.0f/PWM_FRE/2;
		//开环启动加速累加的最小单位
		App.Openloop.RampUpInc  =(float)App.Log.Set_Speed * ENCODER_STEPS /60.0f/PWM_FRE/((MCPara[25]/1000.0f)*PWM_FRE)/2; 
	}else if(MCPara[6] == 1){
		//开环力量增加的最小单位
		App.Openloop.PoweUpUnit	= (float)App.Openloop.Power_Goal/((MCPara[25]/1000.0f)*PWM_FRE);
		//开环累加最小单位计算
		App.Openloop.Step_Unit 	= (float)App.Log.Set_Speed * ENCODER_STEPS	/60.0f/PWM_FRE/2;
		//开环启动加速累加的最小单位
		App.Openloop.RampUpInc  = (float)App.Log.Set_Speed * ENCODER_STEPS /60.0f/PWM_FRE/((MCPara[25]/1000.0f)*PWM_FRE)/2; 		
	}
	//正式开环爬升下降数值
	App.Openloop.PowerClimb = (float)MCPara[29]/1000;
	App.Openloop.StepClimb  = (float)MCPara[28]/10000;
	//闭环参数初始化
	//d轴PI的初始化
	mcApp_D_PIParam.qKp			= (float)MCPara[15]/1000.0f;
	mcApp_D_PIParam.qKi 		= (float)MCPara[16]/1000.0f;
	mcApp_D_PIParam.qOutMax =	(float)MCPara[13];
	mcApp_D_PIParam.qOutMin =-(float)MCPara[14];	
	mcApp_D_PIParam.qI_Max  = mcApp_D_PIParam.qOutMax/2;
	mcApp_D_PIParam.qI_Min	= mcApp_D_PIParam.qOutMin/2;
	mcApp_D_PIParam.qdSum =0;
	mcApp_D_PIParam.qInMeas=0;
	mcApp_D_PIParam.qOut = 0;
	//q轴PI的初始化
	mcApp_Q_PIParam.qKp			= (float)MCPara[19]/1000.0f;
	mcApp_Q_PIParam.qKi 		= (float)MCPara[20]/1000.0f;
	mcApp_Q_PIParam.qOutMax =	(float)MCPara[17];
	mcApp_Q_PIParam.qOutMin =-(float)MCPara[18];
	mcApp_Q_PIParam.qI_Max  = mcApp_Q_PIParam.qOutMax/2;
	mcApp_Q_PIParam.qI_Min	= mcApp_Q_PIParam.qOutMin/2;	
	mcApp_Q_PIParam.qdSum =0;
	mcApp_Q_PIParam.qInMeas=0;
	mcApp_Q_PIParam.qOut = 0;
	//速度PI的初始化
	mcApp_Spd_PIParam.qKp			= (float)MCPara[23]/1000.0f;
	mcApp_Spd_PIParam.qKi 		= (float)MCPara[24]/1000.0f;
	mcApp_Spd_PIParam.qKc 		= 0.001f;
	mcApp_Spd_PIParam.qOutMax =	(float)MCPara[21];
	mcApp_Spd_PIParam.qOutMin =-(float)MCPara[22];
	mcApp_Spd_PIParam.qI_Max	= mcApp_Spd_PIParam.qOutMax/2;
	mcApp_Spd_PIParam.qI_Min  = mcApp_Spd_PIParam.qOutMin/2;
	mcApp_Spd_PIParam.qdSum =0;
	mcApp_Spd_PIParam.qInMeas=0;	
	mcApp_Spd_PIParam.qOut = 0;
	
	//速度设定参考初始化
	App.Spd.Set_SpdTemp = 0;
	App.Spd.Now = 0;
	primask = __get_PRIMASK();              //参数计算期间若发生BREAK，最终提交必须拒绝启动。
	__disable_irq();                        //避免检查通过后硬停，再从此处重新打开PWM。
	if(AlignValid != 0U && App.Err == E_NONE0 && SerUart3.NeedZeroRearm == 0U &&
	   App.Log.Set_Speed != 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET &&
	   __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) == RESET){
		TIM_PWM_Stop();                      //重新启动不继承旧CCR，编码器计数不清零。
		App.Status = MCPara[2] == 0U ? CTLS_OPENLOOP_RUN : CTLS_CLOSELOOP_RUN; //沿用原开闭环选择。
		App.AllRun = 1U;                     //仅有效零点和新启动命令能获得高频运行授权。
		TIM_PWM_Start();                     //临界区内最后开启功率，BREAK仍由硬件直接关闭MOE。
	}else{
		Ctl_HardStop(App.Err);               //失去授权时保持关闭，不能继续使用旧启动状态。
	}
	if(primask == 0U){ __enable_irq(); }
}



void Spd_Init(void){
	
	//速度PI的初始化
	mcApp_Spd_PIParam.qKp			= (float)MCPara[23]/1000.0f;
	mcApp_Spd_PIParam.qKi 		= (float)MCPara[24]/1000.0f;
	mcApp_Spd_PIParam.qKc 		= 0.001f;
	mcApp_Spd_PIParam.qOutMax =	(float)MCPara[21];
	mcApp_Spd_PIParam.qOutMin =-(float)MCPara[22];
	mcApp_Spd_PIParam.qI_Max	= mcApp_Spd_PIParam.qOutMax/2;
	mcApp_Spd_PIParam.qI_Min  = mcApp_Spd_PIParam.qOutMin/2;
	mcApp_Spd_PIParam.qdSum =0;
	mcApp_Spd_PIParam.qInMeas=0;	
	mcApp_Spd_PIParam.qOut = 0;
}



void PI_SPD(void){
		//更新爬坡速度
		Update_Spd();			
		//速度环1ms计算一次
		mcApp_Spd_PIParam.qInRef = App.Spd.Set_SpdTemp;
		//速度测量
		mcApp_Spd_PIParam.qInMeas = App.Spd.Now;
//		mcApp_Spd_PIParam.qInMeas = App.Spd.Now_Fil;
		//速度PI
		mcLib_CalcPI1(&mcApp_Spd_PIParam);		
	
}

void Ctl_Brake(void){
	
}

void Ctl_OpenLoop(void){
	
	if(App.Start == 0){
		App.Status = CTLS_STOP;
	}
	
	if(App.Err > 0){
		App.Status = CTLS_ERROR; 
	}
}
void Ctl_CloseLoop(void){
	if(App.Start == 0U){
		App.Status = CTLS_STOP;			//零速或启动条件撤销后立即进入停机，不再等待编码器低于200rpm。
	}
	if(App.Err > 0U){
		App.Status = CTLS_ERROR; 
	}
}


/*
 * 函数功能：执行正常零速硬停并返回等待状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Stop(void)
{
	Ctl_HardStop(E_NONE0);                   //正常停机也清目标、积分和CCR，停止不依赖编码器。
}

/*
 * 函数功能：保持当前故障锁存并重复确保功率级处于关闭状态。
 * 输入参数：无。
 * 返回参数：无。
 */
void Ctl_Error(void)
{
	uint8_t fault_code = App.Err;            //保存触发本状态的故障码，硬停过程中不得自动清除。
	if((App.AllRun != 0U) || (App.Log.Set_Speed != 0) ||
	   (TIM1->CCER != 0U) || ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)){
		Ctl_HardStop(fault_code);             //仅在输出仍可能使能时补做一次硬停，确保所有故障入口安全收敛。
	}
	else{
		App.Start = 0U;                       //输出已经关闭时只维持禁止启动状态，不重复调用HAL停机接口。
		SerUart3.NeedZeroRearm = 1U;         //故障锁存期间持续要求先收到合法零速帧。
	}
}


//状态灯控制
void STA_Led(uint8_t fault_num){
	static uint8_t cnt = 0;
	if(App.Log.StaLedCyc >= 500){
		App.Log.StaLedCyc = 0;
		
		if(fault_num > 0){
			cnt ++;
			if(cnt <= 2){//1 2 3
				LED_OFF();//关
			}else if(cnt <= (fault_num * 2 - 1) + 2){
				LED_Toggle();
			}
//			else if(cnt <= (App.Err * 2 -1) + 1){
//				STALED_OFF;
//			}
			else{
				cnt = 0;
				LED_OFF();//关
			}
		}else if(App.AllRun == 0){
//			LED_Toggle();
			LED_ON();//在等待状态下 状态指示灯常亮
			cnt = 0;
		}else if(App.AllRun == 1){
//			LED_Toggle();
			LED_OFF();
			cnt = 0;
		}
	}
}
