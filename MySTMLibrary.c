//Biblioteka z podstawowymi funkcjami do STM32
#include "MySTMLibrary.h"

////////////////////////////
////INITIALAZING THE MCU////
////////////////////////////

void RCC_init()
{
	//***RCC deinit***
	
	//SET HSION bit
	RCC->CR |= (uint32_t) RCC_CR_HSION;
	//Reset CFGR
	RCC->CFGR = (uint32_t) 0x00;
	//Reset HSEON, CSSON, PLLON
	RCC->CR &= (uint32_t) ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
	//Reset HSEBYP
	RCC->CR &= (uint32_t) ~RCC_CR_HSEBYP;
	//Disable all interrupts
	RCC->CIR = (uint32_t) 0x00;
		
	//***RCC init***
	
	//Set HSEON
	RCC->CR |= (uint32_t) RCC_CR_HSEON;
	//wait for HSERDY flag
	while((RCC->CR&RCC_CR_HSERDY)==0)	{}
		
	//Set latency for flash memory (0 to 7 cycles)
	FLASH->ACR |= (uint32_t) FLASH_ACR_LATENCY_5WS;
	//Turn on prefetch buffer
	FLASH->ACR |= (uint32_t) FLASH_ACR_PRFTEN_Msk;
	//Set system clock on AHB magistral	
	RCC->CFGR |= (uint32_t) RCC_CFGR_HPRE_DIV1; //AHB prescaler disabled (div by 1)
	//APB2 (Max 84MHz) Prescaler set to 2
	RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE1_DIV2; //Div. set to 2 (168MHz/2)
	//APB1 (Max 42MHz) Prescaler set to 4
	RCC->CFGR |= (uint32_t) RCC_CFGR_PPRE2_DIV4; //Div. set to 4 (168MHz/4)
	//PLL config.
	RCC->PLLCFGR = (uint32_t) 0x00;
	unsigned int PLLM = 2;
	unsigned int PLLN = 84;
	unsigned int PLLP = (2>>1)-1;
	unsigned int PLLQ = 4;
	RCC->PLLCFGR |= (uint32_t)( (PLLM<<RCC_PLLCFGR_PLLM_Pos) | (PLLN<<RCC_PLLCFGR_PLLN_Pos) | (PLLP<<RCC_PLLCFGR_PLLP_Pos) | (PLLQ<<RCC_PLLCFGR_PLLQ_Pos) | (1<<RCC_PLLCFGR_PLLSRC_Pos) );
	//Set PLLON
	RCC->CR  |= (uint32_t) RCC_CR_PLLON;
	while (((RCC->CR) & RCC_CR_PLLON) == 0){}
	//Set SW possision on PLLCLK
	RCC->CFGR &= (uint32_t)(~(RCC_CFGR_SW_Msk));
  RCC->CFGR |= (uint32_t) RCC_CFGR_SW_PLL;    
	while ((RCC->CFGR & (uint32_t) RCC_CFGR_SWS) != (uint32_t) 0x08){}
}

///////////////////////////////////
////STK TIMER RELATED FUNCTIONS////
///////////////////////////////////

//Basic delay function - does nothing and waits a certain number of seconds (defined by user - "ms")
void delay_SysTick(volatile uint32_t ms)
{
		uint32_t ticks = (uint32_t)(((double)ms / (double)1000)*(double)AHB_freq_8); //Convertion to ticks
		delay_ticks_SysTick(ticks);
}
//Basic delay function - does nothing and waits a certain number of ticks (defined by user - "ticks")
void delay_ticks_SysTick(volatile uint32_t ticks)
{
		SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_COUNTFLAG_Msk | SysTick_CTRL_TICKINT_Msk); //Reset ClockSource - AHB/8, Reset CountFlag, Disable interruptions
		if(ticks <= SysTick_Max) 
		{
			SysTick->LOAD = ticks;
			SysTick->CTRL |= (uint32_t) SysTick_CTRL_ENABLE_Msk; //Enable COUNTER
			while((SysTick->CTRL & (uint32_t) SysTick_CTRL_COUNTFLAG_Msk) == 0){} //Wait and do nothing
			SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_COUNTFLAG_Msk | SysTick_CTRL_ENABLE_Msk); //Reset CountFlag, Disable COUNTER
		}
		else
		{
			SysTick->LOAD = SysTick_Max;
			SysTick->CTRL |= (uint32_t) SysTick_CTRL_ENABLE_Msk; //Enable COUNTER
			while((SysTick->CTRL & (uint32_t) SysTick_CTRL_COUNTFLAG_Msk) == 0){} //Wait and do nothing
			SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_COUNTFLAG_Msk | SysTick_CTRL_ENABLE_Msk); //Reset CountFlag, Disable COUNTER
			delay_ticks_SysTick(ticks - SysTick_Max);
		}		
}

//Basic timer - runs in a background and interrupts the CPU once the certain number of seconds passes (defined by user - "ms")
void timer_SysTick(volatile uint32_t ms)
{
		uint32_t ticks = (uint32_t)(((double)ms / (double)1000)*(double)AHB_freq_8); //Convertion to ticks
		SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_COUNTFLAG_Msk); //Reset ClockSource - AHB/8, Reset CountFlag
		SysTick->CTRL |= (uint32_t) SysTick_CTRL_TICKINT_Msk; //Set TICKINT
		SysTick->LOAD = ticks; //Load TICKS
		SysTick->CTRL |= (uint32_t) SysTick_CTRL_ENABLE_Msk; //Enable COUNTER	
}

//Stop timer
void stop_timer_SysTick()
{
		SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_COUNTFLAG_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); //Reset CountFlag, Disable COUNTER, Disable interruptions
}

//Reset timer
void reset_timer_SysTick()
{
		SysTick->CTRL &= (uint32_t) ~(SysTick_CTRL_COUNTFLAG_Msk); //Reset CountFlag
}

////////////////////////////////
////TIMER6_7 RELATED FUNCTIONS////
////////////////////////////////

//Basic delay function - does nothing and waits a certain number of seconds (defined by user - "ms")
void delay_TIMER6(volatile uint32_t ms)
{
		volatile uint32_t ticks, prescaler;
		if(ms > TIMER6_7_Max_time) //if time is bigger than MAX
		{
				prescaler = TIMER6_7_Max + 1;
				ticks = TIMER6_7_Max;
		}
		else 
		{
			prescaler = get_prescaler(ms, TIMER6_7_Max); //Calculating the prescaler
			if(prescaler > TIMER6_7_Max) prescaler = TIMER6_7_Max + 1; //if prescaler is bigger than max 16-bit number - set it to MAX value
			ticks = get_ticks(ms, prescaler, TIMER6_7_Max); //Convertion to ticks
		}	
		delay_ticks_TIMER6(prescaler, ticks);
}

void delay_TIMER7(volatile uint32_t ms)
{
		volatile uint32_t ticks, prescaler;
		if(ms > TIMER6_7_Max_time) //if time is bigger than MAX
		{
				prescaler = TIMER6_7_Max + 1;
				ticks = TIMER6_7_Max;
		}
		else 
		{
			prescaler = get_prescaler(ms, TIMER6_7_Max); //Calculating the prescaler
			if(prescaler > TIMER6_7_Max) prescaler = TIMER6_7_Max + 1; //if prescaler is bigger than max 16-bit number - set it to MAX value
			ticks = get_ticks(ms, prescaler, TIMER6_7_Max); //Convertion to ticks
		}	
		delay_ticks_TIMER7(prescaler, ticks);
}

//Basic delay function - does nothing and waits a certain number of ticks (defined by user - "ticks")
void delay_ticks_TIMER6(uint32_t prescaler, uint32_t ticks)
{
			RCC->APB1ENR |= (uint32_t) RCC_APB1ENR_TIM6EN; //Enable Clock Signal	
			TIM6->PSC = (uint16_t) (prescaler-1); //Set prescaler
			TIM6->DIER &= (uint32_t) ~TIM_DIER_UIE; //Disable interruption
			TIM6->ARR = (uint16_t) ticks; //Set value
			TIM6->CR1 |= (uint32_t) TIM_CR1_CEN; //Enable counter
			while((TIM7->SR & (uint32_t) 1) == 0){} //Wait and do nothing
			stop_timer_TIMER6();
}

void delay_ticks_TIMER7(uint32_t prescaler, uint32_t ticks)
{
			RCC->APB1ENR |= (uint32_t) RCC_APB1ENR_TIM7EN; //Enable Clock Signal	
			TIM7->PSC = (uint16_t) (prescaler-1); //Set prescaler
			TIM7->DIER &= (uint32_t) ~TIM_DIER_UIE; //Disable interruption
			TIM7->ARR = (uint16_t) ticks; //Set value
			TIM7->CR1 |= (uint32_t) TIM_CR1_CEN; //Enable counter
			while((TIM7->SR & (uint32_t) 1) == 0){} //Wait and do nothing
			stop_timer_TIMER7();
}

//Basic timer - runs in a background and interrupts the CPU once the certain number of seconds passes (defined by user - "ms")
void timer_TIMER6(volatile uint32_t ms)
{
		volatile uint32_t ticks, prescaler;
		if(ms > TIMER6_7_Max_time) //if time is bigger than MAX
		{
				prescaler = TIMER6_7_Max + 1;
				ticks = TIMER6_7_Max;
		}
		else
		{
			prescaler = get_prescaler(ms, TIMER6_7_Max); //Calculating the prescaler
			ticks = get_ticks(ms, prescaler, TIMER6_7_Max); //Convertion to ticks
		}
			RCC->APB1ENR |= (uint32_t) RCC_APB1ENR_TIM6EN; //Enable Clock Signal
			TIM6->PSC = (uint16_t) (prescaler-1); //Set prescaler
			TIM6->DIER |= (uint32_t) TIM_DIER_UIE; //Enable interruption
			TIM6->ARR = (uint16_t) ticks; //Set value
			NVIC_EnableIRQ(TIM6_DAC_IRQn); //Enable Global Interrupt TIM6 i DAC
			TIM6->CR1 |= (uint32_t) TIM_CR1_CEN; //Enable counter

}

void timer_TIMER7(volatile uint32_t ms)
{
		volatile uint32_t ticks, prescaler;
		if(ms > TIMER6_7_Max_time) //if time is bigger than MAX
		{
				prescaler = TIMER6_7_Max + 1;
				ticks = TIMER6_7_Max;
		}
		else
		{
			prescaler = get_prescaler(ms, TIMER6_7_Max); //Calculating the prescaler
			ticks = get_ticks(ms, prescaler, TIMER6_7_Max); //Convertion to ticks
		}
			RCC->APB1ENR |= (uint32_t) RCC_APB1ENR_TIM7EN; //Enable Clock Signal
			TIM7->PSC = (uint16_t) (prescaler-1); //Set prescaler
			TIM7->DIER |= (uint32_t) TIM_DIER_UIE; //Enable interruption
			TIM7->ARR = (uint16_t) ticks; //Set value
			NVIC_EnableIRQ(TIM7_IRQn); //Enable Global Interrupt TIM7
			TIM7->CR1 |= (uint32_t) TIM_CR1_CEN; //Enable counter	
}

void reset_timer_TIMER6(void)
{
		TIM6->SR = (uint32_t) 0; //Reset Status Register
}

void reset_timer_TIMER7(void)
{
		TIM7->SR = (uint32_t) 0; //Reset Status Register

}

void stop_timer_TIMER6(void)
{
		TIM6->SR = (uint32_t) 0; //Reset Status Register
		TIM6->CR1 &= (uint32_t) ~TIM_CR1_CEN; //Disable counter	
}

void stop_timer_TIMER7(void)
{
		TIM7->SR = (uint32_t) 0; //Reset Status Register
		TIM7->CR1 &= (uint32_t) ~TIM_CR1_CEN; //Disable counter	
}

/////////////////////////
////IO, LED FUNCTIONS////
/////////////////////////

//Initializes the LED chosen by the user ("LED" - 12,13,14 or 15)
void LED_init(int LED)
{
		switch(LED)
		{
			case 12:
				RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;
				GPIOD->MODER |= (1<<GPIO_MODER_MODE12_Pos);
				GPIOD->OSPEEDR |= (3U<<GPIO_OSPEEDR_OSPEED12_Pos);				
				break;
			case 13:
				RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;
				GPIOD->MODER |= (1<<GPIO_MODER_MODE13_Pos);
				GPIOD->OSPEEDR |= (3U<<GPIO_OSPEEDR_OSPEED13_Pos);
				break;
			case 14:
				RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;
				GPIOD->MODER |= (1<<GPIO_MODER_MODE14_Pos);
				GPIOD->OSPEEDR |= (3U<<GPIO_OSPEEDR_OSPEED14_Pos);				
				break;
			case 15:
				RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;
				GPIOD->MODER |= (1<<GPIO_MODER_MODE15_Pos);
				GPIOD->OSPEEDR |= (3U<<GPIO_OSPEEDR_OSPEED15_Pos);				
				break;
		}
}

//Initializes all LEDs
void LED_init_all()
{
		LED_init(LED_GREEN);
		LED_init(LED_RED);
		LED_init(LED_BLUE);
		LED_init(LED_ORANGE);
}

//Turns on the LED chosen by the user ("LED" - 12,13,14 or 15)
void LED_on(int LED)
{
		switch(LED)
		{
			case 12:
				GPIOD->ODR |= (uint32_t) (GPIO_ODR_OD12); 	
				break;
			case 13:
				GPIOD->ODR |= (uint32_t) (GPIO_ODR_OD13); 	
				break;
			case 14:
				GPIOD->ODR |= (uint32_t) (GPIO_ODR_OD14); 				
				break;
			case 15:
				GPIOD->ODR |= (uint32_t) (GPIO_ODR_OD15); 
				break;
		}
}

//Turns off the LED chosen by the user ("LED" - 12,13,14 or 15)
void LED_off(int LED)
{
		switch(LED)
		{
			case 12:
				GPIOD->ODR &= ~(uint32_t) (GPIO_ODR_OD12); 			
				break;
			case 13:
				GPIOD->ODR &= ~(uint32_t) (GPIO_ODR_OD13);	
				break;
			case 14:
				GPIOD->ODR &= ~(uint32_t) (GPIO_ODR_OD14);
				break;
			case 15:
				GPIOD->ODR &= ~(uint32_t) (GPIO_ODR_OD15);
				break;
		}
}

//Toggles the LED chosen by the user ("LED" - 12,13,14 or 15)
void LED_toggle(int LED)
{
		switch(LED)
		{
			case 12:
				GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD12);
				break;
			case 13:
				GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD13);
				break;
			case 14:
				GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD14);			
				break;
			case 15:
				GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD15);
				break;
		}
}

void LED_toggle_all()
{
		GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD12);
		GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD13);
		GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD14);
		GPIOD->ODR ^= (uint32_t) (GPIO_ODR_OD15);
}

//Turns on all LEDs
void LED_on_all()
{
		LED_on(LED_ORANGE);
		LED_on(LED_RED);
		LED_on(LED_BLUE);
		LED_on(LED_GREEN);
}

//Turns off all LEDs
void LED_off_all()
{
		
		LED_off(LED_ORANGE);
		LED_off(LED_RED);
		LED_off(LED_BLUE);
		LED_off(LED_GREEN);
}

//"LED sweep" with blink time defined by the user (ms)
void LED_sweep(uint32_t ms)
{
		LED_on(LED_GREEN);
		delay_SysTick(ms);
		LED_off(LED_GREEN);
		LED_on(LED_ORANGE);
		delay_SysTick(ms);
		LED_off(LED_ORANGE);
		LED_on(LED_RED);
		delay_SysTick(ms);
		LED_off(LED_RED);
		LED_on(LED_BLUE);
		delay_SysTick(ms);
		LED_off(LED_BLUE);
}

/////////////////////////
////////// PWM //////////
/////////////////////////

void PWM(volatile double f, volatile unsigned int D, int LED) 
{
		volatile uint32_t ticks, prescaler;
		prescaler = get_prescaler((uint32_t)(1000.0/(double)f), Timer2_5_Max); //Calculating the prescaler
		ticks = get_ticks((uint32_t)(1000.0/(double)f),(uint32_t)prescaler, Timer2_5_Max); //Convertion to ticks
		
		RCC->APB1ENR &= (uint32_t) ~RCC_APB1ENR_TIM4EN;
		RCC->APB1ENR |= (uint32_t) RCC_APB1ENR_TIM4EN; //Initialize TIM4 RCC
		RCC->AHB1ENR &= (uint32_t) ~RCC_AHB1ENR_GPIODEN;
		RCC->AHB1ENR |= (uint32_t) RCC_AHB1ENR_GPIODEN;
		
	
		switch(LED)
		{
			case 12:
				GPIOD->MODER &= (uint32_t)~(GPIO_MODER_MODE12_Msk);			
				GPIOD->MODER |= (2U<<GPIO_MODER_MODE12_Pos);
				GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED12_Pos);
				GPIOD->AFR[1] |= (uint32_t)(2U << GPIO_AFRH_AFSEL12_Pos);
				break;
			case 13:
				GPIOD->MODER &= (uint32_t)~(GPIO_MODER_MODE13_Msk);	
				GPIOD->MODER |= (2U<<GPIO_MODER_MODE13_Pos);
				GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED13_Pos);
				GPIOD->AFR[1] |= (uint32_t)(2U << GPIO_AFRH_AFSEL13_Pos);
				break;
			case 14:
				GPIOD->MODER &= (uint32_t)~(GPIO_MODER_MODE14_Msk);	
				GPIOD->MODER |= (2U<<GPIO_MODER_MODE14_Pos);
				GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED14_Pos);
				GPIOD->AFR[1] |= (uint32_t)(2U << GPIO_AFRH_AFSEL14_Pos);		
				break;
			case 15:
				GPIOD->MODER &= (uint32_t)~(GPIO_MODER_MODE15_Msk);	
				GPIOD->MODER |= (2U<<GPIO_MODER_MODE15_Pos);
				GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED15_Pos);
				GPIOD->AFR[1] |= (uint32_t)(2U << GPIO_AFRH_AFSEL15_Pos);
				break;
		}		
		
		TIM4->PSC = (uint32_t) prescaler-1;	//Prescaler
		TIM4->ARR = (uint32_t) ticks;	//Time		

		TIM4->CCER |= (uint32_t) TIM_CCER_CC4E;
		TIM4->CCMR2 |= (uint32_t) (6U <<TIM_CCMR2_OC4M_Pos);
		
		TIM4->CCR4 = (uint32_t) ((double)ticks * (double)D / 255.0); //Duty Cycle		
		
		TIM4->CR1 |= (uint32_t) TIM_CR1_CEN;
}

/////////////////////////
////INTERRUPT HANDLER////
/////////////////////////

void SysTick_Handler()
{
		on_SysTick_end();
}

void TIM6_DAC_IRQHandler()
{
		reset_timer_TIMER6();
		on_TIMER6_end();
}

void TIM7_IRQHandler()
{
		reset_timer_TIMER7();
		on_TIMER7_end();
}

__attribute__((weak)) void on_SysTick_end(){} //Allows the function to be overwritten (GCC Compiler)
	
__attribute__((weak)) void on_TIMER6_end(){} //Allows the function to be overwritten (GCC Compiler)
	
__attribute__((weak)) void on_TIMER7_end(){} //Allows the function to be overwritten (GCC Compiler)

uint32_t get_prescaler(volatile uint32_t ms, uint32_t MAX_VALUE)
{
		uint32_t prescaler = (uint32_t) ceil((double) ms / 1000.0 * ((double)APB1_freq * 2.0) / (double)MAX_VALUE); //Calculating the prescaler
		if(prescaler > TIMER6_7_Max + 1) prescaler = MAX_VALUE + 1; //if prescaler is bigger than max 16-bit number - set it to MAX value
		return prescaler;
}

uint32_t get_ticks(volatile uint32_t ms, uint32_t prescaler, uint32_t MAX_VALUE)
{
		uint32_t ticks = (uint32_t)((double)ms / 1000.0 * ((double)APB1_freq * 2.0) / (double)prescaler); //Convertion to ticks
		if(ticks > MAX_VALUE) ticks = MAX_VALUE; //if prescaler is bigger than max 16-bit number - set it to MAX value
		return ticks;
}
