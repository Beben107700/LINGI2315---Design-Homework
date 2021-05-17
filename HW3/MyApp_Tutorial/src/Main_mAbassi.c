/* ------------------------------------------------------------------------------------------------ */

#include "mAbassi.h"
#include "Platform.h"								/* Everything about the target processor is here*/
#include "HWinfo.h"									/* Everything about the tagret hardware is here	*/
#include "SysCall.h"								/* System Call layer stuff						*/

#include "dw_uart.h"

/* ------------------------------------------------------------------------------------------------ */
/* App variables																					*/

int G_UartDevIn  = UART_DEV;						/* Needed by the system call layer if used		*/
int G_UartDevOut = UART_DEV;						/* Needed by the system call layer if used		*/
int G_UartDevErr = UART_DEV;						/* Needed by the system call layer if used		*/

/* ------------------------------------------------------------------------------------------------ */
/* App functions																					*/

extern void Task_HPS_Led(void);
extern void Task_FPGA_Led(void);
extern void Task_FPGA_Button(void);
extern void Task_Tutorial(void);
extern void Task_ReadSensor(void);
extern void Task_DisplaySensor(void);
/* ------------------------------------------------------------------------------------------------ */

int main(void)
{
TSK_t *Task;

	setvbuf(stdin,  NULL, _IONBF, 0);				/* By default, Newlib library flush the I/O		*/
	setvbuf(stdout, NULL, _IONBF, 0);				/* buffer when full or when new-line			*/
	setvbuf(stderr, NULL, _IONBF, 0);

/* ------------------------------------------------ */
/* Start uAbassi									*/

	OSstart();										/* Start uAbassi								*/

	TSKsetCore(TSKmyID(), 0);						/* Make sure App_0 will always run on Core #0	*/

	SysCallInit();									/* Initialize the System Call layer				*/

	OSintOn();										/* Enable the interrupts						*/

/* ------------------------------------------------ */
/* SysTick Set-up									*/
													/* Timer tick needed for Sleep() & timeouts		*/
	OSisrInstall(OS_TICK_INT, &TIMtick);			/* Install the RTOS timer tick function			*/
	TIMERinit(((((OS_TICK_SRC_FREQ/OS_TICK_PRESCL)/100000)*(OX_TIMER_US))/(10)));
	GICenable(OS_TICK_INT, 128, 1);					/* Priority == mid / Edge trigger				*/

/* ------------------------------------------------ */
/* UART set-up										*/
/* 8 bits, No Parity, 1 Stop						*/
/* Cyclone V has 2 UART                             */

	uart_init(UART_DEV, BAUDRATE, 8, 0, 10, UART_Q_SIZE_RX, UART_Q_SIZE_TX, UART_FILT_OUT_LF_CRLF);

	OSisrInstall(UART0_INT, &UARTintHndl_0);
	GICenable(-UART0_INT, 128, 0);					/* UART Must use level and not edge trigger		*/
    
	OSisrInstall(UART1_INT, &UARTintHndl_1);
	GICenable(-UART1_INT, 128, 0);					/* UART Must use level and not edge trigger		*/

/* ------------------------------------------------ */
/* Application set-up								*/
	Task = TSKcreate("Task Tutorial", 0, 8192, &Task_Tutorial, 0);
	TSKsetCore(Task, 1);							/* Create new task, will always run on core #1	*/
	TSKresume(Task);								/* when BMP (G_OS_MP_TYPE == 4 or 5)			*/


	Task = TSKcreate("Task FPGA Led", 1, 8192, &Task_FPGA_Led, 0);
	TSKsetCore(Task, 1);							/* Create new task, will always run on core #1	*/
	TSKresume(Task);								/* when BMP (G_OS_MP_TYPE == 4 or 5)			*/

	Task = TSKcreate("Task FPGA Button", 1, 8192, &Task_FPGA_Button, 0);
	TSKsetCore(Task, 1);							/* Create new task, will always run on core #1	*/
	TSKresume(Task);								/* when BMP (G_OS_MP_TYPE == 4 or 5)			*/

	Task = TSKcreate("Task read the sensor", 1, 8192, &Task_ReadSensor, 0);
	TSKsetCore(Task, 1);							/* Create new task, will always run on core #1	*/
	TSKresume(Task);

	Task = TSKcreate("Task display the sensor", 1, 8192, &Task_DisplaySensor, 0);
	TSKsetCore(Task, 0);							/* Create new task, will always run on core #1	*/
	TSKresume(Task);

#if defined(USE_SHELL)
    TSKcreate("Shell", OX_PRIO_MIN, 16384, OSshell, 1);
    TSKsleep(OS_MS_TO_TICK(100));                   /* Make sure Shell Task initialization if over  */
#endif

	TSKsetCore(TSKmyID(), 0);						/* Make sure Task_HPS_Led always run on Core #0	*/

	Task_HPS_Led();									/* Go into core #0 Task_HPS_Led code			*/

	for(;;);										/* Should never come back here					*/
}

/* ------------------------------------------------------------------------------------------------ */
/* Not used but needed to build uAbassi																*/

void TIMcallBack(void)
{
	return;
}

/* EOF */
