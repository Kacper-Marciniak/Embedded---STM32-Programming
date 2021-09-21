#include "MySTMLibrary.h"

void on_TIMER6_end() //This function is executed on TIMER6 OVF (interruption)
{
	LED_toggle(LED_ORANGE);
	reset_timer_TIMER6();
}

void on_TIMER7_end() //This function is executed on TIMER7 OVF (interruption)
{
	LED_toggle(LED_GREEN);
	reset_timer_TIMER7();
}

int main ()
{
	//Init RCC
	RCC_init();	
	//Init LED
	LED_init_all();
	//LED Sweep
	for(int i=0;i<5;i++) LED_sweep(100);
	//LED ORANGE - timer 6
	timer_TIMER6(1000);
	//LED GREEN - timer 7
	timer_TIMER7(100);	
	//LED BLUE - PWM
	int f = 1000;
	int cycle_time = 1000;
	while(1)
	{
		LED_on(LED_RED);
		for(int D = 0; D <= 255; D++)
		{
			PWM(f, (uint32_t) D, LED_BLUE);
			delay_SysTick((uint32_t)(cycle_time/255));
		}
		for(int D = 255; D >= 0; D--)
		{
			PWM(f, (uint32_t) D, LED_BLUE);
			delay_SysTick((uint32_t)(cycle_time/255));
		}
		LED_off(LED_RED);
		delay_SysTick((uint32_t) 1000);
	}
	return 1;
}
