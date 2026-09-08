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
/* 定位保留参数51/52爬升和参数5慢响应预算；满足稳定与完整位移条件后可提前结束，不缩短电气保护路径。 */
#define ALIGN_RAMP_MS ((uint32_t)MCPara[51] * (uint32_t)MCPara[52]) //原单步间隔乘步数，默认1ms×500=500ms。
#define ALIGN_HOLD_MS ((uint32_t)MCPara[5]) //满幅后的慢响应观察预算，默认300ms；位移及100ms稳定合格可提前结束，不再强制等满。
#define ALIGN_PHASE_EXTRA_MS      20U   //阶段额外期限保持20ms，不能随稳定质量窗口一起延长加力。
#define ALIGN_PHASE_MAX_MS (ALIGN_RAMP_MS + ALIGN_HOLD_MS + ALIGN_PHASE_EXTRA_MS) //默认单阶段仍最多820ms，到期未稳定则失败。
#define ALIGN_STABLE_MS          100U   //满幅后必须连续稳定100ms，避免中途解除外力后短暂静止被接受。
#define ALIGN_QUALITY_SPAN_COUNTS   1   //末段窗口峰峰值最多1计数，拒绝超出该范围的摆动和累积漂移。
#define ALIGN_PHASE_COUNT          6U   //正B/正A/正B/正A/反B/正A共六阶段，全部通过才发布零点。
#define ALIGN_RELEASE_MS          50U   //合法零速、PB12低和编码器静止连续满足的时间。
#define ALIGN_RETRY_OFF_MS       100U   //两轮之间关闭功率至少100ms，避免连续硬顶管路。
#define ALIGN_MAX_ATTEMPTS         2U   //每轮授权最多两次定位，连续零速不会无限重试。
#define ALIGN_ZERO_FRAMES          2U   //至少两帧连续合法零速才能开始定位。
#define ALIGN_ENCODER_SIGN         1   //A正相向B正相过渡时编码器应增加，与现有正弦表一致。
#define ALIGN_STEP_COUNTS ((ENCODER_STEPS + 1U) / (4U * NR)) //当前4000计数、50齿，四分之一电周期为20计数。
#define ALIGN_STEP_TOLERANCE       4   //单程允许20±4计数，不能只凭编码器有变化就放行。
#define ALIGN_RETURN_TOLERANCE     3   //三个A相候选任意两点最多差3计数，不将不同侧偏差平均后放行。
#define ALIGN_STILL_COUNTS         1   //稳定窗口相对固定参考最多变化1计数，缓慢爬行会重置窗口。

typedef enum {
    ALIGN_READY = 0,                 //定位通过，等待新的零速握手和启动命令。
    ALIGN_WAIT_ZERO,                 //等待合法零速与物理安全条件。
    ALIGN_B_FIRST,                   //B相预定位，尚不建立编码器零点。
    ALIGN_A_FIRST,                   //第一次A相定位，保存候选参考。
    ALIGN_B_CHECK,                   //A向B运动，检查正向位移。
    ALIGN_A_CHECK,                   //B向A回位，检查反向位移与重复性。
    ALIGN_B_OPPOSITE,                //从A转到反B相，验证零点另一侧的负向位移。
    ALIGN_A_VERIFY,                  //从反B正向返回A，与前两次A候选交叉确认后才允许建立零点。
    ALIGN_RETRY_OFF,                 //第一次失败后的断电静止间隔。
    ALIGN_FAILED                    //本轮失败锁存，只接受独立零速重定位请求，普通启动不能重新加力。
} MotorAlignState;

/* 反馈byte4高半字节携带版本和定位状态，低半字节仍是原App.Err；两端必须配套升级。 */
#define ALIGN_FEEDBACK_READY       0x80U //已验证零点；仍需原始故障为0且零速握手完成才允许运行。
#define ALIGN_FEEDBACK_BUSY        0x90U //上电等待、有限定位或完成后待零速握手，不能等同运行故障。
#define ALIGN_FEEDBACK_FAILED      0xA0U //未建立有效零点且已锁存失败，持续零速不能自动重试。
#define ALIGN_COMMAND_RESTART     0x02U //命令byte1独立操作码；只允许速度字段为0，旧驱动会安全拒绝。
#define ALIGN_COMMAND_CANCEL      0x03U //明确全停操作码，区别于维持校准的普通零速；只允许速度为0。

/* 每一轮明确授权的首次失败记录；valid最后发布，第二次失败和普通启动拒绝不覆盖。 */
typedef struct {
    uint32_t tick_ms;                    //首次失败的HAL时刻，可与主控/采样记录对照。
    uint32_t phase_elapsed_ms;           //失败时当前相位已经加力的时间，等待阶段记0。
    uint32_t sample_tick_ms;             //最近有效相电流与电压样本的时刻，不能冒充失败瞬间实测值。
    float current_a;                    //最近有限值A相采样，单位A；采样比例仍须台架确认。
    float current_b;                    //同一更新周期的B相采样，单位A，保留正负号。
    float bus_voltage;                  //同一更新周期的母线电压，单位V。
    int16_t encoder_count;              //失败前TIM2原始计数，未写入新的编码器零点。
    int16_t delta_forward;              //A到B的测量位移，只有B校验及以后阶段有对应判据意义。
    int16_t delta_return;               //正B复查到第二A的位移；新增反B回程用失败阶段/原因区分，不能混读。
    int16_t return_error;               //前两次A位置的差值，不代表最后另一侧返回的A；完整成功位置另存六阶段快照。
    uint8_t reason;                     //1不稳定、2去程、3回程、4回位、5保护/参数、6取消、7首段、8反B去程、9反B回程。
    uint8_t state;                      //停机前的定位阶段，不记录已被覆盖的FAILED状态。
    uint8_t attempt;                    //首次失败属于第几次尝试，自动补试不清记录。
    uint8_t raw_error;                  //当次原始故障码；单纯几何判据失败记E_ALIGN6。
    uint8_t sample_valid;               //为1表示本轮有有限数值样本，不代表传感器已获台架标定。
    uint8_t valid;                      //为1表示以上字段已完整发布，直到新一轮明确定位才清除。
} MotorAlignFailureSnapshot;

/* 成功记录只在完整六阶段通过后发布；数组顺序固定为正B/正A/正B/正A/反B/正A。 */
typedef struct {
    uint32_t tick_ms;                    //成功提交时刻，与首次失败快照区分，不能把补试成功误读成首轮通过。
    uint32_t duration_ms;                //本次成功尝试的总加力时长，不包含前次失败或轮间等待。
    uint32_t sample_tick_ms[ALIGN_PHASE_COUNT]; //各阶段最近有效电流/电压样本时刻，用于识别陈旧样本。
    float current_a[ALIGN_PHASE_COUNT];   //各阶段最近有限值A相电流，单位A，仅诊断，不新增相电流跳闸阈值。
    float current_b[ALIGN_PHASE_COUNT];   //各阶段同周期B相电流，保留正负号以核对反B励磁。
    float bus_voltage[ALIGN_PHASE_COUNT]; //各阶段同周期母线电压，不能替代台架电流或转矩校准。
    int16_t encoder_count[ALIGN_PHASE_COUNT]; //各阶段清零前的原始计数，跨0/3999比较须按4000回绕。
    uint16_t stable_ms[ALIGN_PHASE_COUNT]; //各阶段最终连续稳定时长，超出16位时饱和记录。
    uint16_t swing_counts[ALIGN_PHASE_COUNT]; //各阶段稳定窗口峰峰值，成功时不超过质量阈值。
    uint8_t sample_valid_mask;           //低六位分别表示各阶段有有效样本，置位不代表传感器已获标定。
    uint8_t phase_mask;                  //低六位全部置位才允许提交，禁止用不完整候选记录建立零点。
    uint8_t attempt;                     //成功属于本轮第几次尝试，自动补试后可与首次失败同时保留。
    uint8_t valid;                       //全部字段完成后最后发布，明确新定位授权或上电时清除。
} MotorAlignSuccessSnapshot;

extern volatile MotorAlignFailureSnapshot AlignFirstFailure; //停机后可一次查看首次失败，不需要持续单步捕捉。
extern volatile MotorAlignSuccessSnapshot AlignLastSuccess; //查看成功尝试的全部候选和稳定质量，RAM记录不跨断电。
extern volatile uint16_t AlignRejectedStarts; //累计未就绪时的普通启动拒绝，不改首次失败记录。

extern volatile uint8_t AlignValid;          //只有校验通过才为1，零速清故障不能直接置位。
extern volatile uint8_t AlignState;          //当前定位阶段，便于仿真器直接观察。
extern volatile uint8_t AlignAttempt;        //当前授权轮中的尝试次数。
extern volatile uint8_t AlignFailure;        //最近定位失败原因1~9；普通启动拒绝不再伪装成定位失败。
extern volatile int16_t AlignDeltaForward;   //实测A向B的有符号计数位移。
extern volatile int16_t AlignDeltaReturn;    //实测B向A的有符号计数位移。
extern volatile int16_t AlignReturnError;    //第二次A位置相对第一次A的误差，最终另一侧A位置另见成功快照。
extern volatile float AlignPower;           //当前受限的定位电压幅值。
void Motor_AlignInit(void);                   //初始化上电校准门禁，功率保持关闭。
void Motor_AlignProcess(void);                //主循环按真实毫秒节拍推进定位。
void Motor_AlignOutput(void);                 //PWM中断在新鲜采样后更新定位输出。
void Motor_AlignOnFault(uint8_t fault_code);  //PWM已关闭后保存首次真实保护码，再撤销未完成定位。
uint8_t Motor_AlignCommand(uint16_t speed);   //返回1表示命令由校准门禁消费，不能进入普通运行处理。
uint8_t Motor_AlignRequest(void);             //失败恢复后独立申请有限重定位，不携带普通运行目标。
void Motor_AlignCancel(void);                  //明确全停取消校准，保留真实故障和首次快照；普通零速保活不调用。
uint8_t Motor_AlignFeedback(void);            //返回版本化定位状态和原始故障码，不增加回包长度。
extern volatile uint16_t MCPara[ParaNum];
extern float sin_table[4000];
extern float cos_table[4000];
void TIM_PWM_Stop(void);
void TIM_PWM_Start(void);
#endif
