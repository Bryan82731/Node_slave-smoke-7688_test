#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include "bcomdef.h"


void Uart_taskFxn(UArg a0, UArg a1);
extern void UART_creatTask(void);
void Util_stopClock(Clock_Struct *pClock);


