
#include <stdint.h>

/* Delay ---------------------------------------------------------------------*/
void DelayMs(volatile int32_t n);
void DelayUs(volatile int32_t us);
extern uint32_t HAL_GetTick(void);
