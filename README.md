# Embedded---STM32-Programming
Custom STM32 library with basic I/O functions.

## RCC Initialization

- void RCC_init(void);

*RCC initialization*

## SysTick

- void delay_SysTick(volatile uint32_t ms);

*Delay in ms using SysTick*
- inline void delay_ticks_SysTick(volatile uint32_t ticks);

*Delay with ticks*
- void timer_SysTick(volatile uint32_t ticks);

*Timer (interruptions) using SysTick - use with "on_SysTick_end()*
- inline void reset_timer_SysTick(void);

*RESET SysTick*
- inline void stop_timer_SysTick(void);

*STOP SysTick*

## TIMER6&7

- void delay_TIMER6(volatile uint32_t ms);

*Delay in ms using TIMER6*
- void delay_TIMER7(volatile uint32_t ms);

*Delay in ms using TIMER7*
- inline void delay_ticks_TIMER6(uint32_t prescaler, uint32_t ticks);

*Delay with ticks (TIM6)*
- inline void delay_ticks_TIMER7(uint32_t prescaler, uint32_t ticks);

*Delay with ticks (TIM7)*
- void timer_TIMER6(volatile uint32_t ms);

*Timer (interruptions) using TIMER6 - use with "on_TIMER6_end()"*
- void timer_TIMER7(volatile uint32_t ms);

*Timer (interruptions) using TIMER7 - use with "on_TIMER7_end()"*
- inline void reset_timer_TIMER6(void);

*RESET TIMER6*
- inline void reset_timer_TIMER7(void);

*RESET TIMER7*
- inline void stop_timer_TIMER6(void);

*STOP TIMER6*
- inline void stop_timer_TIMER7(void);

*STOP TIMER7*

## LED
- inline void LED_init(int LED);

*Initialize chosen LED*
- void LED_init_all(void);

*Initialize all LEDs*
- inline void LED_on(int LED); 

*Turn on chosen LED*
- inline void LED_off(int LED);

*Turn off chosen LED*
- inline void LED_toggle(int LED);

*Toggle chosen LED*
- inline void LED_on_all(void);

*Turn on all LED*
- inline void LED_off_all(void);

*Turn off all LED*
- inline void LED_toggle_all(void);

*Toggle all LEDs*
- inline void LED_sweep(uint32_t ms);

*LED Sweep - 1 cycle*

## PWM

- void PWM(volatile double f, volatile unsigned int D, int LED);

*PWM function where: f - frequency, D - Duty Cycle: 0-255, LED - chosen LED diode*

## INTERRUPT HANDLERS

- extern void on_SysTick_end(void);
- extern void on_TIMER6_end(void);
- extern void on_TIMER7_end(void);
- extern void SysTick_Handler(void);
- extern void TIM6_DAC_IRQHandler(void);
- extern void TIM7_IRQHandler(void);

## ADDITIONAL FUNCTIONS

- uint32_t get_prescaler(volatile uint32_t ms, uint32_t MAX_VALUE);
- uint32_t get_ticks(volatile uint32_t ms, uint32_t prescaler, uint32_t MAX_VALUE);
