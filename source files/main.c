/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"
/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define Load_1_Period 10
#define Load_2_Period 100          
#define Button1_Period   50
#define Button2_Period   50
#define Transmitter_Period  100
#define Receiver_Period		20
#define CAPACITY_Load_1 5
#define CAPACITY_Load_2 12

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );



TaskHandle_t Button_1_Handler =NULL;
TaskHandle_t Button_2_Handler =NULL;
TaskHandle_t Periodic_Transmitter_Handler =NULL;
TaskHandle_t Uart_Receiver_Handler =NULL;

QueueHandle_t xQueue_Message; 

int task_1_in_time =0,task_1_out_time =0, task_1_total_time;
int task_2_in_time =0,task_2_out_time =0, task_2_total_time;
int task_3_in_time =0,task_3_out_time =0, task_3_total_time;
int task_4_in_time =0,task_4_out_time =0, task_4_total_time;
int task_5_in_time =0,task_5_out_time =0, task_5_total_time;
int task_6_in_time =0,task_6_out_time =0, task_6_total_time;
int system_time = 0;
int cpu_load = 0;


unsigned char misses_TX;
unsigned char misses_L1;
unsigned char misses_L2;
unsigned char misses_B1;
unsigned char misses_B2;
unsigned char misses_Rx;

typedef enum{
	RisingEdge,
  FallingEdge,
	Level_Low,
	Level_High
}ButtonStatus;

typedef struct{
	uint8_t* Str;
}message;

/*-----------------------------------------------------------*/
/*The IDLE Hook Function implementation*/
void vApplicationIdleHook( void )
{
		GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
	
}
/*-----------------------------------------------------------*/

/*The Task Hook Function implementation*/
void vApplicationTickHook( void )
{
	GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}
/*-----------------------------------------------------------*/

/*Task Load_1_Simulation Function Implementation*/
			void Load_1_Simulation(void* pvParameters)
			{
				
				TickType_t xLastWakeTimeA = xTaskGetTickCount();
				volatile int count = CAPACITY_Load_1;
				volatile static TickType_t A_Deadline;
				
				A_Deadline = xTaskGetTickCount();
				 vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
				for(;;)
				{
					TickType_t xTime = xTaskGetTickCount();
					TickType_t x;
				while(count!=0)
					{
						if ((x = xTaskGetTickCount())>xTime)
						{
							xTime = x;
							count -- ;
						}
			}
			count = CAPACITY_Load_1;
			A_Deadline +=Load_1_Period;	
					vTaskDelayUntil(&xLastWakeTimeA , Load_1_Period);
			if (xTime>A_Deadline)
			{
				misses_L1+=1;
			}
			}
			}
			/*Task LOAD 2 Handler*/
			TaskHandle_t Load_1_Simulation_Handler = NULL;
/*-----------------------------------------------------------*/
			/*Task Load_2_Simulation Function Implementation*/
			void Load_2_Simulation(void* pvParameters)
			{
				
				TickType_t xLastWakeTimeB = xTaskGetTickCount();
				volatile static TickType_t B_Deadline ;
				volatile int count = CAPACITY_Load_2;
				B_Deadline = xTaskGetTickCount();
				
				vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
				for(;;)
				{
					TickType_t xTime = xTaskGetTickCount();
					TickType_t x;
				while(count!=0)
					{
						if ((x = xTaskGetTickCount())>xTime)
						{
							xTime = x;
							count -- ;
						}

			}
			count = CAPACITY_Load_2;
			B_Deadline +=Load_2_Period;
					vTaskDelayUntil(&xLastWakeTimeB , Load_2_Period);
			}
			}
			/*Task LOAD2 Handler*/
			TaskHandle_t Load_2_Simulation_Handler = NULL;
/*-----------------------------------------------------------*/
			
	
/*-----------------------------------------------------------*/
		
		/*-----------------------------------------------------------*/
		/*-----------------------------------------------------------*/
		/*-----------------------------------------------------------*/
	/*-----------------------------------------------------------*/






message button1_message={(uint8_t *)"---BUTTON 1---\n"};
message button2_message={(uint8_t *)"---BUTTON 2---\n"};
message SendStr_message={(uint8_t *)"Hello World !!\n"};

message BufferdMessage;



/*-----------------------------------------------------------*/
void Button_1_Monitor(void * pvParameters)
{
	volatile static TickType_t C_Deadline;
	ButtonStatus B_1_Prev_State=Level_High;
	uint8_t B_1_Current_State;
	TickType_t xLastWakeTimeA = xTaskGetTickCount();	
	C_Deadline = xTaskGetTickCount();
	
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	
	for( ; ; ) 
	{
		TickType_t xTime = xTaskGetTickCount();
		
		B_1_Current_State=GPIO_read(PORT_1,PIN0);
		
		if(B_1_Current_State == PIN_IS_LOW)
		{
			if(B_1_Prev_State==Level_High)
			{
				/*falling edge */
				B_1_Prev_State=FallingEdge;
				button1_message.Str=(uint8_t *)"Butt_1_Falling\n";
			}
			else
			{
				/*Low Level*/
				B_1_Prev_State=Level_Low;
				button1_message.Str=(uint8_t *)"Button1 NoEdge\n";
				
			}
		}
		else /*B_1_Current_State==1*/
		{
			if(B_1_Prev_State==Level_Low)
			{
				/*rising edge*/
				B_1_Prev_State=RisingEdge;
				button1_message.Str=(uint8_t *)"Butt_1_Rising \n";
			}
			else
			{
				/*High Level*/
				B_1_Prev_State=Level_High;
				button1_message.Str=(uint8_t *)"Button1 NoEdge\n";
			}
			
		}
		
		if(B_1_Prev_State != Level_Low && B_1_Prev_State != Level_High)
			{if( xQueue_Message != 0 )xQueueSend( xQueue_Message,( void * ) &button1_message,( TickType_t ) 10 );}
     
		vTaskDelayUntil(&xLastWakeTimeA , Button1_Period);  /*periodicity : 10 ms */
			
	}
}

void Button_2_Monitor(void * pvParameters)
{
	TickType_t xLastWakeTimeA = xTaskGetTickCount();
	uint8_t B_2_Current_State;
	ButtonStatus B_2_Prev_State=Level_High;
	volatile static TickType_t D_Deadline;
				
				D_Deadline = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	
	for( ; ; ) 
	{
		TickType_t xTime = xTaskGetTickCount();
		B_2_Current_State=GPIO_read(PORT_1,PIN1);
		if(B_2_Current_State==0)
		{
			if(B_2_Prev_State==Level_High)
			{
				/*falling edge */
				B_2_Prev_State=FallingEdge;
				button2_message.Str=(uint8_t *)"Butt_2_Falling\n";
			}
			else
			{
				/*Low Level*/
				B_2_Prev_State=Level_Low;
				button2_message.Str=(uint8_t *)"Button2 NoEdge\n";
				
			}
		}
		else /*B_2_Current_State==1*/
		{
			if(B_2_Prev_State==Level_Low)
			{
				/*rising edge*/
				B_2_Prev_State=RisingEdge;
				button2_message.Str=(uint8_t *)"Butt_2_Rising \n";
			}
			else
			{
				/*High Level*/
				B_2_Prev_State=Level_High;
				button2_message.Str=(uint8_t *)"Button2 NoEdge\n";
			}
			
		}
		
		if((B_2_Prev_State !=Level_High) && (B_2_Prev_State !=Level_Low))
			{if( xQueue_Message != 0 )xQueueSend( xQueue_Message,( void * ) &button2_message,( TickType_t ) 10 );}
		
	D_Deadline +=Button2_Period;
		vTaskDelayUntil(&xLastWakeTimeA , Button2_Period);  /*periodicity : 50 ms */
				if (xTime>D_Deadline)
			{
				misses_B2+=1;
			}
	}
}



void Periodic_Transmitter(void * pvParameters)
{
	volatile static TickType_t TX_Deadline;
				
	TickType_t xLastWakeTimeA = xTaskGetTickCount();
				TX_Deadline = xTaskGetTickCount();

	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	for( ; ; ) 
	{
		TickType_t xTime = xTaskGetTickCount();
		if( xQueue_Message != 0 ){ xQueueSend( xQueue_Message,( void * ) &SendStr_message,( TickType_t ) 10 );}
		TX_Deadline += Transmitter_Period;
		vTaskDelayUntil(&xLastWakeTimeA , Transmitter_Period);  /* periodicity : 100 ms */
			if (xTime>TX_Deadline)
			{
				misses_TX+=1;
			}
	}
		
		
}

void UART_Write(void * pvParameters)
{
	volatile static TickType_t Rx_Deadline;
				
	TickType_t xLastWakeTimeA = xTaskGetTickCount();
				Rx_Deadline = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ; ; ) 
	{
		TickType_t xTime = xTaskGetTickCount();
		 if( xQueueReceive( xQueue_Message,&(BufferdMessage ),( TickType_t ) 0 ) == pdPASS )
      {
         
				vSerialPutString((const signed char *)(BufferdMessage.Str),15);
      }
		
		Rx_Deadline+=Receiver_Period;
		vTaskDelayUntil(&xLastWakeTimeA,Receiver_Period);  /* periodicity : 20 ms */
			if (xTime>Rx_Deadline)
			{
				misses_Rx+=1;
			}
	}
}
		/*-----------------------------------------------------------*/
		/*-----------------------------------------------------------*/
		/*-----------------------------------------------------------*/

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	


    /* Create Tasks here */
	xTaskPeriodicCreate(Button_1_Monitor,"Button1 Monitor",100,NULL,0,&Button_1_Handler,Button1_Period);
	xTaskPeriodicCreate(Button_2_Monitor,"Button2 Monitor ",100,NULL,0,&Button_2_Handler,Button2_Period);
	xTaskPeriodicCreate(Periodic_Transmitter,"Periodic Tx",100,NULL,0,&Periodic_Transmitter_Handler,Transmitter_Period);
	xTaskPeriodicCreate(UART_Write,"UART Rx",100,NULL,1,&Uart_Receiver_Handler,Receiver_Period);
	xTaskPeriodicCreate(Load_1_Simulation,"Load_1_Simulation",100,(void*)0,0,&Load_1_Simulation_Handler,Load_1_Period);
  xTaskPeriodicCreate(Load_2_Simulation,"Load_2_Simulation",100,(void*)0,0,&Load_2_Simulation_Handler,Load_2_Period);	
			
	xQueue_Message = xQueueCreate(3, sizeof(message) );
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


