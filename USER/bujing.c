#include "bujing.h"
#include <stdlib.h>
#include "logic.h"

AppType App;
mcParam_PIController mcApp_Q_PIParam;
mcParam_PIController mcApp_D_PIParam;
mcParam_PIController mcApp_Spd_PIParam;

int16_t text_power=0;


int16_t text_Spd = 0;
/*
 * 函数功能：从持续运行的ADC DMA更新电压、两相电流和母线电流；停PWM不冻结保护采样。
 * 输入参数：无。
 * 返回参数：无。
 */
void Motor_UpdateSamples(void){
//计算电流
	//电流AD值1
	App.Adc.Cur_AD1 = adc_buf[1];
	App.Adc.Cur_temp1 = App.Adc.Cur_AD1;		//不滤波
//App.Adc.Cur_temp1 = App.Adc.Cur_temp1 *(0.90f) + App.Adc.Cur_AD1 *(0.10f);				//滤波
	App.Adc.Cur_Now1 = (App.Adc.Cur_temp1 - App.Adc.Current_Median1)*VOLTAGE_CURRENT_RATIO;		//(AD-ADMid)*3.3f/4096/0.132f				
//App.Adc.Cur_Now1 = (App.Adc.Current_Median1 - App.Adc.Cur_temp1)*VOLTAGE_CURRENT_RATIO;		//(AD-ADMid)*3.3f/4096/0.132f				
	App.Cal.Current1 = App.Adc.Cur_Now1 / CURRENT_SENSOR_COEFFICIENT ;												//0.132-10A  0.006 -20A  0.044 -30A						
	//电流AD值2
	App.Adc.Cur_AD2 	= adc_buf[2];		
	App.Adc.Cur_temp2 = App.Adc.Cur_AD2;			//不滤波
//App.Adc.Cur_temp2 = App.Adc.Cur_temp2 *(0.90f) + App.Adc.Cur_AD2 *(0.10f);				//滤波
	App.Adc.Cur_Now2 = (App.Adc.Cur_temp2-App.Adc.Current_Median2)*VOLTAGE_CURRENT_RATIO;
//App.Adc.Cur_Now2 = (App.Adc.Current_Median2 - App.Adc.Cur_temp2 )*VOLTAGE_CURRENT_RATIO;
	App.Cal.Current2 = App.Adc.Cur_Now2 / CURRENT_SENSOR_COEFFICIENT ;
		
//母线电压
	App.Adc.voltage  = adc_buf[0] *VOLTAGEP_ADC_PVCC_PHY_RATIO;
	App.Cal.voltage  = App.Adc.voltage;

//总电流
	App.Adc.Current_AllTemp = adc_buf[3] * ADC_CURRENT_SCALE;
	App.Cal.CurrentT = 	App.Cal.CurrentT*0.5f+App.Adc.Current_AllTemp*0.5f;
	
}

/*
 * 函数功能：按PWM节拍更新采样并执行定位或原运行控制，定位输出与闭环输出互斥。
 * 输入参数：无。
 * 返回参数：无。
 */
void PWM_InterruptService(void){
	Motor_UpdateSamples();                  //先取得新鲜电流，定位输出前即可执行限流与快速保护。
	if(App.Status == CTLS_ALIGN){
		Motor_AlignOutput();                 //定位只使用原始编码器及静止坐标电压，不依赖未建立的电角度。
		return;                             //禁止下方AllRun=0分支把有效定位PWM清零。
	}

	
	Get_Encoder();
	
	

	

	if(App.AllRun == 1){

	if(MCPara[2] == 0){				//开环			
		if(App.Openloop.Stage == 0){				//开环加速阶段 角度给定
			if(App.Openloop.RampUpTimeCnt<App.Openloop.RampUpTime){			//开环加
				App.Openloop.RampUpTimeCnt++;			//0.0001S累加					
				//每一步增加的长度逐渐增加
				App.Openloop.RampUpTemp += App.Openloop.RampUpInc;			//增加的每一步长度逐渐增加		
				if(MCPara[6] == 0){				//正转
					if(App.Openloop.RampUpTemp >App.Openloop.Step_Unit){
						App.Openloop.RampUpTemp = App.Openloop.Step_Unit;		//限幅，后面加加速度
					}
					App.Openloop.Step_Temp  += App.Openloop.RampUpTemp;			//总步长累加
					if(App.Openloop.Step_Temp >ENCODER_STEPS){
						App.Openloop.Step_Temp = 0;
					}
				}else{										//反转
					if(App.Openloop.RampUpTemp < App.Openloop.Step_Unit){
						App.Openloop.RampUpTemp = App.Openloop.Step_Unit;		//限幅，后面加加速度
					}
					App.Openloop.Step_Temp  += App.Openloop.RampUpTemp;			//总步长累加
					if(App.Openloop.Step_Temp < 0){
						App.Openloop.Step_Temp = ENCODER_STEPS;
					}					
				}
				App.Openloop.Step  = App.Openloop.Step_Temp;						//位置给定
					//开环D轴给定
				mcApp_D_PIParam.qInRef = 7.0f;
					//开环Q轴给定
//				mcApp_Q_PIParam.qInRef = App.Openloop.power;				
				mcApp_Q_PIParam.qInRef = MCPara[7];	
			}else if(App.Openloop.RampUpTimeCnt >= App.Openloop.RampUpTime){
				App.Openloop.Stage = 1;					//转正式开环
				App.Openloop.RampUpTimeCnt = 0;	//
			}
		}
		else if(App.Openloop.Stage ==1){		//正式开环阶段  角度给定
			//开环D轴给定
			mcApp_D_PIParam.qInRef = 7.0f;
			//爬坡换相时间计算
			Update_PhaseStep();
			//爬坡力量计算
//			Update_PowerStep();		
			
			//开环Q轴给定
			if(MCPara[6] == 0){				//正转
				mcApp_Q_PIParam.qInRef = MCPara[7];				//自己给的固定力量
				
				App.Openloop.Step_Temp += App.Openloop.Step_Unit;							//10KHz 
				if(App.Openloop.Step_Temp >ENCODER_STEPS){
					App.Openloop.Step_Temp = 0;
				}
				App.Openloop.Step  = App.Openloop.Step_Temp;
			}else{									//反转
				mcApp_Q_PIParam.qInRef = -MCPara[7];	
				
				App.Openloop.Step_Temp += App.Openloop.Step_Unit;							//10KHz 
				if(App.Openloop.Step_Temp <0){
					App.Openloop.Step_Temp = ENCODER_STEPS;
				}
				App.Openloop.Step  = App.Openloop.Step_Temp;				
			}
		}
		
		App.Cal.Sin_Val 	 = sin_table[App.Openloop.Step];					//查表得到角度相对应的Sin值
		App.Cal.Cos_Val 	 = cos_table[App.Openloop.Step];					//查表得到角度相对应的Cos值				
		
		App.Cal.V_d	= mcApp_D_PIParam.qInRef ;
		App.Cal.V_q	=	mcApp_Q_PIParam.qInRef ;
	}else if(MCPara[2] == 1){									//闭环
		//速度环放外面
		
		//编码器偏置赋值和爬坡
		if( App.Spd.Now_Fil <50){
			App.Log.Offset_Encode_Set = 0;					
		}else if(App.Spd.Now_Fil <600 && App.Spd.Now_Fil>=50){				//速度在50-500的区间内，编码器赋值，并且实现爬坡
			App.Log.Offset_Encode_Set  = (float)App.Spd.Now_Fil /35.0f;						//均分500均分15 大概等于35
		}else{
			App.Log.Offset_Encode_Set  = ENCODER_OFFSET;								//满偏置15
		}
		App.Log.Offset_CycCnt ++;
		if(App.Log.Offset_CycCnt >App.Log.Offset_Cyc){
			App.Log.Offset_CycCnt = 0;
			if(App.Log.Offset_Encode_Set >App.Log.Offset_Encode_Now){
				App.Log.Offset_Encode_Now += 0.01f;					
			}else if(App.Log.Offset_Encode_Now >App.Log.Offset_Encode_Set){
				App.Log.Offset_Encode_Now -= 0.01f;						
			}else{
				App.Log.Offset_Encode_Now = App.Log.Offset_Encode_Set;							//爬坡值只能默认1
			}
		}
			
		//获取编码器值
		if(App.Spd.En_Dir ==CW){
			//固定偏置
//			App.Cal.cnt = TIM2->CNT + ENCODER_OFFSET;
			//变化偏置
			App.Cal.cnt = TIM2->CNT + (int16_t)App.Log.Offset_Encode_Now;
			if(App.Cal.cnt > ENCODER_STEPS){
				App.Cal.cnt -= ENCODER_STEPS;
			}
		}else if(App.Spd.En_Dir == CCW){
			//固定偏置
//			App.Cal.cnt = TIM2->CNT - ENCODER_OFFSET;
			App.Cal.cnt = TIM2->CNT - App.Log.Offset_Encode_Now;
			if(App.Cal.cnt < 0){
				App.Cal.cnt += ENCODER_STEPS;
			}			
		}
		App.Cal.Sin_Val = sin_table[App.Cal.cnt];					//查表得到角度相对应的Sin值
		App.Cal.Cos_Val = cos_table[App.Cal.cnt];					//查表得到角度相对应的Cos值				
	
		//力矩测量
		App.Cal.I_d_Measure =  App.Cal.Cos_Val  * App.Cal.Current1 + App.Cal.Sin_Val * App.Cal.Current2;		
		App.Cal.I_q_Measure = -App.Cal.Sin_Val  * App.Cal.Current1 + App.Cal.Cos_Val * App.Cal.Current2;			
		
		mcApp_D_PIParam.qInMeas = App.Cal.I_d_Measure;
		mcApp_Q_PIParam.qInMeas = App.Cal.I_q_Measure;
		
		//D轴参考给定
		mcApp_D_PIParam.qInRef = 5.0f;
//		mcApp_D_PIParam.qInRef = 0.0f;
		//Q轴参考给定
		mcApp_Q_PIParam.qInRef = mcApp_Spd_PIParam.qOut;
		
		//D轴PI
		mcLib_CalcPI1(&mcApp_D_PIParam);
		//Q轴PI
		mcLib_CalcPI1(&mcApp_Q_PIParam);
		
//		if(mcApp_Spd_PIParam.qInRef>0){
//			if(mcApp_Q_PIParam.qOut<0){
//				mcApp_Q_PIParam.qOut =0;
//			}
//		}else if(mcApp_Q_PIParam.qOut <0){
//			if(mcApp_Q_PIParam.qOut>0){
//				mcApp_Q_PIParam.qOut =0;
//			}			
//		}
		
		App.Cal.V_d	= mcApp_D_PIParam.qOut;
		App.Cal.V_q	= mcApp_Q_PIParam.qOut;	
	
	}
	
		//Park逆变换
		App.Cal.V_a = App.Cal.Cos_Val * App.Cal.V_d - App.Cal.Sin_Val * App.Cal.V_q;				//park逆变换
		App.Cal.V_b = App.Cal.Sin_Val * App.Cal.V_d + App.Cal.Cos_Val * App.Cal.V_q;				//park逆变换				
		//简单限幅一下
		if(App.Cal.V_a>MCPara[4]){
			App.Cal.V_a = MCPara[4];
		}else if(App.Cal.V_a<-MCPara[4]){
			App.Cal.V_a = -MCPara[4];
		}
		
		if(App.Cal.V_b>MCPara[4]){
			App.Cal.V_b = MCPara[4];
		}else if(App.Cal.V_b<-MCPara[4]){
			App.Cal.V_b = -MCPara[4];
		}
		//输出
		Set_OutPut_A(App.Cal.V_a,24);
		Set_OutPut_B(App.Cal.V_b,24);
		
	}else{
		if(App.Status == CTLS_INIT ){
			//如果是电机初始化状态，就不对其进行控制，因为要开环定位
		}else{
			App.Cal.V_a = 0;
			App.Cal.V_b = 0;
			Set_OutPut_A(0,App.Cal.voltage);
			Set_OutPut_B(0,App.Cal.voltage);		
			App.Log.Offset_Encode_Now = 0;
			App.Log.Offset_Encode_Set = 0;
			
		}
	}
}	

void Get_Encoder(void){
	
	App.Spd.En_Cnt = TIM2->CNT;
	App.Spd.En_Err = App.Spd.En_Cnt-App.Spd.En_Cnt_Last;			//计算差值
	
	if(App.Spd.En_Err > ENCODER_STEPS_MID){										//超过半长，判定是4000->0的跳变
		App.Spd.En_Err -= ENCODER_STEPS;	
	}
	if(App.Spd.En_Err < -ENCODER_STEPS_MID){
		App.Spd.En_Err += ENCODER_STEPS;												//超过半长，判定是0->4000的跳变
	}
	//判断方向，如果是大于0->正	<0->反  =0可能是没有运动
	if(App.Spd.En_Err > 0){
		App.Spd.En_Dir = CW;
	}
	else if(App.Spd.En_Err < 0){
		App.Spd.En_Dir = CCW;
	}
	else if(App.Spd.En_Err == 0){
		//可能是没有运动，看看后面能不能去判断停止
//		App.Spd.Now = 0;
	}
	App.Spd.En_Cnt_Abs = abs(App.Spd.En_Err);
	
	if((App.Spd.En_Cnt_Abs > 79)||(App.Spd.Spd_Cal_10ms_Flag ==1)){			//大于80个脉冲才计算一次转速，转速高速更加稳定		或者是大于10ms就计数一次，低速会更好
		//需要确保App.Spd.EnTime_Cnt 不能为0，但是又不能产生偏差，只能忽略掉0的这个值，最小为1，其余的全部需要向下偏移1
		if(App.Spd.EnTime_Cnt >=2){
			App.Spd.EnTime_Cnt = App.Spd.EnTime_Cnt -1;	
		}else{
			App.Spd.EnTime_Cnt =1;
		}
		App.Spd.Spd_Cal_10ms_Flag = 0;	//清除10ms计数标志
		App.Spd.Spd_Cal_10ms_Cnt = 0;		//清除10ms计数值
		App.Spd.Spd_Time = App.Spd.EnTime_Cnt*0.1f;				//放大1k倍来计算
//		App.Spd.Spd_Time = App.Spd.EnTime_Cnt*0.01f;				//放大1k倍来计算		
		if(App.Spd.En_Dir ==CW){						//正
			App.Spd.Temp = App.Spd.En_Cnt_Abs*60.0f*1000/ENCODER_STEPS/App.Spd.Spd_Time;
			App.Spd.Now	 = App.Spd.Now * App.Spd.Fil + App.Spd.Temp * (1-App.Spd.Fil);
		}else if(App.Spd.En_Dir ==CCW){			//反
			App.Spd.Temp = App.Spd.En_Cnt_Abs*60.0f*1000/ENCODER_STEPS/App.Spd.Spd_Time;
			App.Spd.Now	 =( App.Spd.Now * App.Spd.Fil - App.Spd.Temp * (1-App.Spd.Fil));		
		}
		App.Spd.En_Cnt_Last = App.Spd.En_Cnt;		//旧值传递
		App.Spd.EnTime_Cnt  = 1;								//清计时
		App.Spd.En_Cnt_Abs = 0;
		
		text_Spd = App.Spd.Now;
	}
	if(App.Spd.Now >0){
		App.Spd.Now_FilTemp=App.Spd.Now;
	}else{
		App.Spd.Now_FilTemp=-App.Spd.Now;
	}
	App.Spd.Now_Fil = App.Spd.Now_Fil * 0.4f + App.Spd.Now_FilTemp	* 0.6f;			//计算速度滤波
}


/*
 * 函数功能：把相电压绝对值换算成TIM1比较值，并在转换前完成有限值和ARR上限保护。
 * 输入参数：phase_voltage为相电压，supply_voltage为换算母线电压。
 * 返回参数：返回0到TIM1 ARR之间的安全比较值。
 */
static uint32_t Motor_CalcCompare(float phase_voltage, float supply_voltage)
{
	float magnitude;                         //保存相电压绝对值，避免负数直接转换成无符号数。
	float compare;                           //在浮点域完成完整换算和限幅。
	uint32_t compare_max = PWM_STEPS - 1U;   //TIM1 ARR为624，CCR不得写入625。

	if((supply_voltage <= 0.0f) || (phase_voltage != phase_voltage) || (supply_voltage != supply_voltage)){
		return 0U;                             //母线无效或出现NaN时立即输出零占空比。
	}
	magnitude = (phase_voltage >= 0.0f) ? phase_voltage : -phase_voltage; //只使用非负电压幅值参与换算。
	compare = ((float)PWM_STEPS * magnitude / supply_voltage) + (float)PWM_STEPS_OFFSET;
	if(compare != compare){
		return 0U;                             //运算中产生NaN时保持输出关闭。
	}
	if(compare <= 0.0f){
		return 0U;                             //零或负比较值统一写零。
	}
	if(compare >= (float)compare_max){
		return compare_max;                    //过大值和正无穷统一限制到ARR。
	}
	return (uint32_t)compare;                //完成范围校验后再转换为无符号寄存器值。
}

/*
 * 函数功能：按A相电压方向更新互斥的两路PWM比较值。
 * 输入参数：v_a为A相目标电压，v_supply为换算母线电压。
 * 返回参数：无。
 */
void Set_OutPut_A(float v_a , float v_supply)
{
	uint32_t compare = Motor_CalcCompare(v_a, v_supply); //先完成有限值检查和ARR限幅。
	if(v_a > 0.0f){
		TIM1->CCR3 = 0U;                       //正向只允许CCR2输出，反向桥臂保持关闭。
		TIM1->CCR2 = compare;
	}
	else if(v_a < 0.0f){
		TIM1->CCR2 = 0U;                       //反向只允许CCR3输出，正向桥臂保持关闭。
		TIM1->CCR3 = compare;
	}
	else{
		TIM1->CCR2 = 0U;                       //零电压同时关闭A相两路输出。
		TIM1->CCR3 = 0U;
	}
}

/*
 * 函数功能：按B相电压方向更新互斥的两路PWM比较值。
 * 输入参数：v_b为B相目标电压，v_supply为换算母线电压。
 * 返回参数：无。
 */
void Set_OutPut_B(float v_b , float v_supply)
{
	uint32_t compare = Motor_CalcCompare(v_b, v_supply); //先完成有限值检查和ARR限幅。
	if(v_b > 0.0f){
		TIM1->CCR4 = 0U;                       //正向只允许CCR1输出，反向桥臂保持关闭。
		TIM1->CCR1 = compare;
	}
	else if(v_b < 0.0f){
		TIM1->CCR1 = 0U;                       //反向只允许CCR4输出，正向桥臂保持关闭。
		TIM1->CCR4 = compare;
	}
	else{
		TIM1->CCR1 = 0U;                       //零电压同时关闭B相两路输出。
		TIM1->CCR4 = 0U;
	}
}


//更新计算爬坡时间
void Update_PhaseStep(void){
//	if(MCPara[6] == 0){			//正转
		App.Openloop.Set_Step_Unit =(float) App.Log.Set_Speed* ENCODER_STEPS/60.0f/PWM_FRE/2;
//	}
//	else if(MCPara[6] == 1){//反转
//		App.Openloop.Set_Step_Unit =-(float) App.Log.Set_Speed* ENCODER_STEPS	/60.0f/PWM_FRE;
//	}
	if((App.Openloop.Set_Step_Unit - App.Openloop.Step_Unit)>App.Openloop.StepClimb){				//差值比爬坡值大-->增加
		App.Openloop.Step_Unit +=App.Openloop.StepClimb;			
	}
	else if((App.Openloop.Set_Step_Unit - App.Openloop.Step_Unit)<-App.Openloop.StepClimb){				//反差值比爬坡值大-->减小
		App.Openloop.Step_Unit -=App.Openloop.StepClimb;	
	}else{
		App.Openloop.Step_Unit = App.Openloop.Set_Step_Unit;																	//在爬坡值内
	}
}
//计算开环变化力量的爬坡
void Update_PowerStep(void){
	if(MCPara[6] ==0){
		App.Openloop.Power_Goal = App.Openloop.k*App.Log.Set_Speed + App.Openloop.b;			//根据逻辑速度去计算目标力量
	}else if(MCPara[6] ==1){
		App.Openloop.Power_Goal = App.Openloop.k*App.Log.Set_Speed - App.Openloop.b;			//根据逻辑速度去计算目标力量
	}
	if((App.Openloop.Power_Goal - App.Openloop.power)>App.Openloop.PowerClimb){					//差值比爬坡值大-->增加
			App.Openloop.power += App.Openloop.PowerClimb;
	}
	else if ((App.Openloop.Power_Goal - App.Openloop.power)< -App.Openloop.PowerClimb){	//反差值比爬坡值大-->减小
			App.Openloop.power -= App.Openloop.PowerClimb;	
	}
	else{
		App.Openloop.power = App.Openloop.Power_Goal;																			//在爬坡值内
	}
}

void Update_Spd(void){
	App.Spd.RampUpDown_Cycle++;
	
	if(App.Spd.RampUpDown_Cycle>MCPara[50]){			//按照周期去计算一次爬坡，每周期最少1ms
		App.Spd.RampUpDown_Cycle = 0;
	
		if(App.Log.Set_Speed * App.Spd.Set_SpdTemp < 0){
			if(App.Log.Set_Speed >0 ){
				App.Spd.Set_SpdTemp += App.Spd.Spd_RampDown;
			}else{
				App.Spd.Set_SpdTemp -= App.Spd.Spd_RampDown;
			}
		}else{			
			if(App.Log.Set_Speed > 0){
				if((App.Log.Set_Speed - App.Spd.Set_SpdTemp)>App.Spd.Spd_RampUp){							//差值比爬坡值大-->增加
					App.Spd.Set_SpdTemp += App.Spd.Spd_RampUp;
				}		
				else if((App.Log.Set_Speed - App.Spd.Set_SpdTemp)<-App.Spd.Spd_RampDown){				//反差值比爬坡值大-->减小
					App.Spd.Set_SpdTemp -= App.Spd.Spd_RampDown;
				}
				else{
					App.Spd.Set_SpdTemp = App.Log.Set_Speed;																				//在爬坡值内
				}
			}else{
				if((App.Log.Set_Speed - App.Spd.Set_SpdTemp)<-App.Spd.Spd_RampUp){							//差值比爬坡值大-->增加
					App.Spd.Set_SpdTemp -= App.Spd.Spd_RampUp;
				}		
				else if((App.Log.Set_Speed - App.Spd.Set_SpdTemp)>App.Spd.Spd_RampDown){				//反差值比爬坡值大-->减小
					App.Spd.Set_SpdTemp += App.Spd.Spd_RampDown;
				}
				else{
					App.Spd.Set_SpdTemp = App.Log.Set_Speed;																				//在爬坡值内
				}
			}
		}
	}
}


void mcLib_CalcPI1( mcParam_PIController *pParam)
{
    float U;
    float Exc;
    
    pParam->qErr  = pParam->qInRef - pParam->qInMeas; 
		
    U  = pParam->qdSum + pParam->qKp * pParam->qErr;
   
    if( U > pParam->qOutMax )
    {
        pParam->qOut = pParam->qOutMax;
    }    
    else if( U < pParam->qOutMin )
    {
        pParam->qOut = pParam->qOutMin;
    }
    else        
    {
        pParam->qOut = U;  
    }
     
    Exc = U - pParam->qOut;
    pParam->qdSum = pParam->qdSum + pParam->qKi * pParam->qErr - pParam->qKc * Exc;
		//积分限幅
		if(pParam->qdSum >3*pParam->qI_Max){
			pParam->qdSum = 3*pParam->qI_Max;							//1.5倍最大输出值
		}
		else if(pParam->qdSum < 3*pParam->qI_Min){
			pParam->qdSum = 3*pParam->qI_Min;							//1.5倍最小输出值
		}
}
