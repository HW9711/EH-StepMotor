#ifndef __LOGIC_H_
#define __LOGIC_H_

#include "bujing.h"
#include "motor.h"

#define UART_COMMAND_FRAME_SIZE       6U    //控制帧固定为帧头、方向、速度高低字节和CRC低高字节。
#define UART_COMMAND_TIMEOUT_MS       100U  //主控每25ms发送一帧，连续4帧缺失后驱动必须硬停。
#define UART_COMMAND_MAX_SPEED_RPM    630U  //当前主控合法最大值为抽吸泵15档乘42，超出值一律拒绝。


typedef enum{
    CTLS_WAIT=0,
    CTLS_STOP,
    CTLS_INIT,
    CTLS_CHECK,          //检测转速状态中，目前是通过母线电压进行检测
    CTLS_OPENLOOP_RUN,
    CTLS_CLOSELOOP_RUN,
    CTLS_ERROR,          //错误
		CTLS_BRAKE,          //刹车
    CTLS_ALIGN,          //非阻塞上电定位，运行闭环不得改写该状态的PWM。
}MTSTATUS_TYPEDEF;


//----------------------
typedef struct
{
	uint8_t    R_DATA[BUFFER_SIZE];//DMA正在写入的接收区，主循环禁止直接解析。
	uint8_t    RxFrame[UART_COMMAND_FRAME_SIZE];//空闲中断封存的不可变控制帧。
	uint8_t    T_DATA[BUFFER_SIZE];//发送数据
	volatile uint8_t    RxEnd;//封存帧可读取标志，最后写入以发布完整快照。
	volatile uint16_t   RxLen;//本次空闲中断实际接收长度。
	volatile uint16_t   RxDropCnt;//主循环未取走旧快照时被新帧替换的次数。
	volatile uint16_t   RxErrorCnt;//UART错误或DMA重启失败次数。
	volatile uint16_t   RxRejectCnt;//帧长、CRC、方向或速度不合法的拒绝次数。
	volatile uint32_t   LastValidTick;//最近一帧完整合法命令的毫秒时间戳。
	volatile uint8_t    CommandSeen;//上电后是否收到过完整合法命令。
	volatile uint8_t    NeedZeroRearm;//超时或故障后必须先收到合法零速帧才能重新运行。
	uint16_t   RxCRC;//接收校验位
	uint16_t   TxCRC;//发送检验位 
	uint16_t   CalcCRC;	//计算CRC的值
	uint8_t    TxLen;//发送长度
	uint16_t   UpdateRegH;
	uint16_t   UpdateRegL;
	int16_t    UpdatePara;

}MCUART_Type;

void UartDealResponse(void);
void Ctl_Logic(void);
void Usart_Receive_Data(UART_HandleTypeDef *huart);
void UARTDealResponse(void);


void Ctl_Wait(void);
void Ctl_Init(void);
void Ctl_OpenLoop(void);
void Ctl_CloseLoop(void);
void Ctl_Brake(void);
void Ctl_Stop(void);
void Ctl_Error(void);
void Ctl_HardStop(uint8_t fault_code);
void Motor_Ctl(void);
void PI_SPD(void);
void STA_Led(uint8_t fault_num);

void Spd_Init(void);			//速度环初始化

uint16_t CRC_Calc(unsigned char *snd, unsigned char num);
#endif



