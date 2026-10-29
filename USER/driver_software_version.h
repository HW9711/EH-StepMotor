#ifndef DRIVER_SOFTWARE_VERSION_H
#define DRIVER_SOFTWARE_VERSION_H
#include <stdint.h>
#define DRIVER_SW_BASE_VERSION "V1.0" /* 人工大版本，最多8个ASCII字符；升级时在这里修改。 */
#define DRIVER_SW_CODE_START 0x08000000UL /* CRC固定从固件起始地址计算。 */
#define DRIVER_SW_CODE_SIZE 0x00040000UL /* 当前芯片固件校验范围256KB，与上位机一致。 */
#define DRIVER_SW_KIND 2U /* 1手柄驱动，2泵驱动，主控拒收类型不符的记录。 */
#define DRIVER_SW_NORMALIZE_PARAMS 0U /* 1把手柄Flash页99~104按FF计入CRC，调参不改变软件版本。 */
static const char s_driver_sw_marker[] = "PPSWVER1" DRIVER_SW_BASE_VERSION; /* HEX固定标记必须和记录版本共用同一字符串。 */
static uint8_t s_driver_sw_record[32]; /* 启动时生成只读记录，查询时不再次扫描Flash或擦写参数。 */
/*
 * 函数功能：把版本记录的32位字段写成大端，保证主控和HEX工具采用相同字节顺序。
 * 输入参数：p为4字节输出位置；value为字段数值。
 * 返回参数：无。
 */
static void DriverSwVersion_Put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24); /* 保存最高字节。 */
    p[1] = (uint8_t)(value >> 16); /* 保存次高字节。 */
    p[2] = (uint8_t)(value >> 8); /* 保存次低字节。 */
    p[3] = (uint8_t)value; /* 保存最低字节。 */
}
/*
 * 函数功能：启动阶段生成软件版本及代码CRC，正常查询只复制缓存，不延迟运行控制。
 * 输入参数：无。
 * 返回参数：无。
 */
static void DriverSwVersion_Init(void)
{
    const volatile uint8_t *code = (const volatile uint8_t *)DRIVER_SW_CODE_START; /* 按实际Flash字节读取。 */
    const volatile char *version = &s_driver_sw_marker[8]; /* 保持HEX标记驻留，版本来源唯一。 */
    uint32_t crc = 0xFFFFFFFFUL; /* 标准反射CRC32初值。 */
    uint32_t offset; /* 扫描完整代码范围。 */
    uint16_t sum = 0U; /* EEPROM页前30字节累加和。 */
    uint8_t bit; /* 每字节处理8位。 */
    for (offset = 0U; offset < DRIVER_SW_CODE_SIZE; ++offset)
    {
        uint8_t value = code[offset]; /* 默认校验实际代码，不能用编译时间代替。 */
#if DRIVER_SW_NORMALIZE_PARAMS == 1U
        if (offset >= 0x31800UL && offset < 0x34800UL) value = 0xFFU; /* 独立参数页99~104不是固件内容，统一按擦除态计算。 */
#endif
        crc ^= value; /* 当前字节并入CRC。 */
        for (bit = 0U; bit < 8U; ++bit) crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1); /* 与主控和上位机采用相同多项式。 */
    }
    for (offset = 0U; offset < 32U; ++offset) s_driver_sw_record[offset] = 0U; /* 清除保留字段与版本字符串尾部。 */
    s_driver_sw_record[0] = 'S'; s_driver_sw_record[1] = 'W'; /* 记录标记前半部。 */
    s_driver_sw_record[2] = 'V'; s_driver_sw_record[3] = '1'; /* 记录标记后半部。 */
    s_driver_sw_record[4] = 1U; /* 沿用32字节记录格式。 */
    for (offset = 0U; offset < 8U && version[offset] != 0; ++offset) s_driver_sw_record[5U + offset] = (uint8_t)version[offset]; /* 复制实际大版本。 */
    s_driver_sw_record[13] = 1U; /* 算法1为CRC32。 */
    DriverSwVersion_Put32(&s_driver_sw_record[14], DRIVER_SW_CODE_START); /* 保存起始地址。 */
    DriverSwVersion_Put32(&s_driver_sw_record[18], DRIVER_SW_CODE_SIZE); /* 保存校验长度。 */
    DriverSwVersion_Put32(&s_driver_sw_record[22], crc ^ 0xFFFFFFFFUL); /* 保存最终CRC32。 */
    s_driver_sw_record[26] = DRIVER_SW_KIND; /* 板类型防止手柄和泵记录互用。 */
    s_driver_sw_record[27] = DRIVER_SW_NORMALIZE_PARAMS; /* 明确参数归一化规则，HEX比较不得猜测。 */
    for (offset = 0U; offset < 30U; ++offset) sum = (uint16_t)(sum + s_driver_sw_record[offset]); /* 整页业务字段参与页和。 */
    s_driver_sw_record[30] = (uint8_t)(sum >> 8); /* 页和高字节。 */
    s_driver_sw_record[31] = (uint8_t)sum; /* 页和低字节。 */
}
#endif
