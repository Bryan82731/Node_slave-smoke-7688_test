/*******************
10/10號 Scif的Driver
*******************/

#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include "bcomdef.h"



#include <scif.h>
#include <BIOS.h>
#include <scif_driver.h>


Task_Struct my_UART_Task;
Char my_UART_TaskStack[512];
Semaphore_Struct semScTaskAlert;
int rxbuf[100];


void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback


void scTaskAlertCallback(void) {

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

    // Wait for an ALERT callback
    Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    // Echo all characters currently in the RX FIFO
    int rxFifoCount = scifUartGetRxFifoCount();

    //讀出這次buffer
    for(int a=0;a<rxFifoCount;a++)
     rxbuf[a] =(      (char) scifUartRxGetChar()      );

    // Clear the events that triggered this
    scifUartClearEvents();

    // Acknowledge the alert event
    scifAckAlertEvents();   
 
    for(int a=0;a<rxFifoCount;a++)
      rxbuf[a]=0;
} // scTaskAlertCallback

/*********************************************************************
 * @fn      Uart_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void UART_creatTask(void)
{
  Task_Params taskParams;

  Task_Params_init(&taskParams);
  taskParams.stack = my_UART_TaskStack;
  taskParams.stackSize = sizeof(my_UART_TaskStack);
  taskParams.priority = 3;
  Task_construct(&my_UART_Task, Uart_taskFxn, &taskParams, NULL);

  Semaphore_Params semParams;
  Semaphore_Params_init(&semParams);
  semParams.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&semScTaskAlert, 0, &semParams);

}

/*********************************************************************
 * @fn      Uart_taskFxn
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void Uart_taskFxn (UArg a0, UArg a1) {

    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Start the UART emulator
    scifExecuteTasksOnceNbl(BV(SCIF_UART_EMULATOR_TASK_ID));

    // Enable baud rate generation
    scifUartSetBaudRate(57600);

    // Enable RX (10 idle bit periods required before enabling start bit detection)
    scifUartSetRxFifoThr(SCIF_UART_RX_FIFO_MAX_COUNT / 2);
    scifUartSetRxTimeout(10 * 2);
    scifUartSetRxEnableReqIdleCount(10 * 2);
    scifUartRxEnable(1);

    // Enable events (half full RX FIFO or 10 bit period timeout
    scifUartSetEventMask(BV_SCIF_UART_ALERT_RX_FIFO_ABOVE_THR | BV_SCIF_UART_ALERT_RX_BYTE_TIMEOUT);


}