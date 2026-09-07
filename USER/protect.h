#ifndef __PROTECT_H__
#define __PROTECT_H__

#include <stdint.h>


#define		E_NONE0		 0
#define		E_OC1			 1
#define		E_UV2			 2
#define		E_OV3			 3
#define		E_Hardware 4		
#define		E_COMM5		 5
#define E_ALIGN6       6 //定位校验未通过，主控按现有非零故障处理并保留原始0x06。


void Protect(void);
uint8_t Protect_CanClearFault(uint8_t fault_code);
void Protect_ResetCounters(void);




#endif



