

#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define RED_LED (1U<<1)
#define BLUE_LED (1U<<2)
#define GREEN_LED (1U<<3)
#define LED_OFF (0U)
#define PORTF (1U<<5)

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND (762*2)


void Three_LEDs_INIT(void);
void MidPriorityTask(void*ptr);
void LowPriorityTask(void*ptr);
void HighPriorityTask(void*ptr);

void delay_ms(unsigned long n);


xSemaphoreHandle xMutex;
int main()
{
	/*mutexes employ priority inheritance. This means that if a high
	  priority task blocks while attempting to obtain a mutex (token) that is
	  currently held by a lower priority task, then the priority of the task 
	  holding the token is temporarily raised to that of the blocking task. */
	
	xMutex = xSemaphoreCreateMutex(); // mutexes employ priority inheritance 
	//xMutex = xSemaphoreCreateBinary(); // Binary semaphores don't employ priority inheritance 
	
	Three_LEDs_INIT();
	
	if( xMutex != NULL )
	{
	xTaskCreate(HighPriorityTask,"HighPriorityTask",240,NULL,3,NULL);
	xTaskCreate(MidPriorityTask,"MidPriorityTask",240,NULL,2,NULL);
	xTaskCreate(LowPriorityTask,"LowPriorityTask",240,NULL,1,NULL);
	vTaskStartScheduler();
	}
	else
	{
	}
	
	for(;;);
}



void MidPriorityTask(void*ptr)
{	
	vTaskDelay(3000/portTICK_RATE_MS);
	
	int counter=0;
	
	for(;;)
	{
				GPIO_PORTF_DATA_R =0x0U;
				while(counter<2)
				{
					GPIO_PORTF_DATA_BITS_R[RED_LED] = RED_LED;
					delay_ms(1000);
					
					GPIO_PORTF_DATA_R =LED_OFF;
					delay_ms(500);
					
					counter++;
					
				}
				counter = 0;
		vTaskDelay(4000/portTICK_RATE_MS);	
	}
	
}


void LowPriorityTask(void*ptr)
{	
	int counter=0;
	
	for(;;)
	{
			xSemaphoreTake( xMutex, 0 );
			{
				GPIO_PORTF_DATA_R =0x0U;
				while(counter<10)
				{
					GPIO_PORTF_DATA_BITS_R[BLUE_LED] = BLUE_LED;
					delay_ms(1000);
					
					GPIO_PORTF_DATA_R =LED_OFF;
					delay_ms(500);
					
					counter++;
					
				}
				counter = 0;
			}
			xSemaphoreGive( xMutex );

	}
	
}
	

void HighPriorityTask(void*ptr)
{	
	vTaskDelay(1000/portTICK_RATE_MS);
	int counter=0;
	
	for(;;)
	{
			xSemaphoreTake( xMutex, portMAX_DELAY );
			{
				GPIO_PORTF_DATA_R =0x0U;
				while(counter<3)
				{
					GPIO_PORTF_DATA_BITS_R[GREEN_LED] = GREEN_LED;
					delay_ms(1000);
					
					GPIO_PORTF_DATA_R =LED_OFF;
					delay_ms(500);
					
					counter++;
				}
				counter = 0;
			}
			xSemaphoreGive( xMutex );
			
			vTaskDelay(3000/portTICK_RATE_MS);
			


	}
	
}



void Three_LEDs_INIT(void)
{
	SYSCTL_RCGCGPIO_R|= PORTF;
	while((SYSCTL_PRGPIO_R&0x00000020)==0){}
	GPIO_PORTF_LOCK_R= 0x4C4F434B;
	GPIO_PORTF_CR_R=0x1F;
	GPIO_PORTF_DIR_R|= RED_LED | GREEN_LED | BLUE_LED;
	GPIO_PORTF_DEN_R|= RED_LED | GREEN_LED | BLUE_LED;
	GPIO_PORTF_DATA_R =LED_OFF;
	
}



void delay_ms(unsigned long n)
{
    volatile unsigned long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}


void vApplicationIdleHook()
{
		
}

	