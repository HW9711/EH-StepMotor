#include "protect.h"
#include "bujing.h"
#include "logic.h"


/*
 * 函数功能：判断当前物理条件是否允许合法零速帧清除已锁存故障。
 * 输入参数：fault_code为当前锁存故障码。
 * 返回参数：故障源已经恢复安全返回1，否则返回0。
 */
uint8_t Protect_CanClearFault(uint8_t fault_code)
{
	switch(fault_code){
		case E_NONE0:
		case E_COMM5:
			return 1U;                         //正常状态和通信故障可由一帧合法零速命令完成重新授权。
		case E_UV2:
			return (App.Cal.voltage >= App.Prot.Allow_Voltage) ? 1U : 0U; //欠压恢复必须重新达到允许启动电压。
		case E_OV3:
			return (App.Cal.voltage <= (App.Prot.OverVoltage - 1.0f)) ? 1U : 0U; //过压下降1V形成清故障回差。
		case E_OC1:
			return (App.Cal.CurrentT <= (App.Prot.OverCurrent * 0.8f)) ? 1U : 0U; //电流降到阈值80%以下才允许清除。
		case E_Hardware:
			return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) ? 1U : 0U; //BREAK为高有效，输入回低才允许重启。
		default:
			return 0U;                         //未知故障保持锁存，避免误恢复功率输出。
	}
}

/*
 * 函数功能：合法零速重新授权时清除保护累计量，后续保护从当前物理状态重新计时。
 * 输入参数：无。
 * 返回参数：无。
 */
void Protect_ResetCounters(void)
{
	App.Prot.UnderVoltage_Cnt = 0;          //清除欠压累计时间。
	App.Prot.OverVoltage_Cnt = 0;           //清除过压累计时间。
	App.Prot.OverCurrent_Cnt = 0;           //清除过流累计时间。
	App.Log.ErrReturnCnt = 0U;               //清除旧自动恢复计数，当前版本不再据此复转。
	App.Log.ErrClearCnt = 0U;                //清除历史恢复次数，仅保留结构兼容。
}

/*
 * 函数功能：按固定周期检测欠压、过压和过流，达到门槛后立即硬停并锁存故障。
 * 输入参数：无。
 * 返回参数：无。
 */
void Protect(void)
{
	if((App.Prot.Protect_1msFg == 1U) && (App.Err == E_NONE0)){
		App.Prot.Protect_1msFg = 0U;          //每个保护周期只处理一次采样结果。

		if(App.Cal.voltage < App.Prot.UnderVoltage){
			App.Prot.UnderVoltage_Cnt++;        //欠压持续存在时累计保护时间。
			if(App.Prot.UnderVoltage_Cnt > App.Prot.UnderVoltage_Tim){
				App.Prot.UnderVoltage_Cnt = 0;
				Ctl_HardStop(E_UV2);              //欠压达到门槛后立即关功率并锁存故障。
				return;
			}
		}
		else if(App.Prot.UnderVoltage_Cnt > 0){
			App.Prot.UnderVoltage_Cnt--;        //电压恢复时逐步消除未达到门槛的瞬态计数。
		}

		if(App.Cal.voltage > App.Prot.OverVoltage){
			App.Prot.OverVoltage_Cnt++;         //过压持续存在时累计保护时间。
			if(App.Prot.OverVoltage_Cnt > App.Prot.OverVoltage_Tim){
				App.Prot.OverVoltage_Cnt = 0;
				Ctl_HardStop(E_OV3);              //过压达到门槛后立即关功率并锁存故障。
				return;
			}
		}
		else if(App.Prot.OverVoltage_Cnt > 0){
			App.Prot.OverVoltage_Cnt--;         //母线恢复正常时消除瞬态计数。
		}

		if(App.Cal.CurrentT > App.Prot.OverCurrent){
			App.Prot.OverCurrent_Cnt++;         //过流持续存在时累计保护时间。
			if(App.Prot.OverCurrent_Cnt > App.Prot.OverCurrent_Tim){
				App.Prot.OverCurrent_Cnt = 0;
				Ctl_HardStop(E_OC1);              //过流达到门槛后立即关功率并锁存故障。
				return;
			}
		}
		else if(App.Prot.OverCurrent_Cnt > 0){
			App.Prot.OverCurrent_Cnt--;         //电流恢复正常时消除瞬态计数。
		}
	}
}
