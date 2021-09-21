//Biblioteka z podstawowymi funkcjami do STM32

#include <stdint.h>
#include "stm32f407xx.h"
//DEFINE - LEDs
#define	LED_GREEN							(int) 12
#define	LED_ORANGE						(int) 13
#define LED_RED								(int) 14
#define LED_BLUE							(int) 15
#define GPIO_ODR_OD_GREEN			GPIO_ODR_OD12
#define GPIO_ODR_OD_ORANGE		GPIO_ODR_OD13
#define GPIO_ODR_OD_RED				GPIO_ODR_OD14
#define GPIO_ODR_OD_BLUE			GPIO_ODR_OD15
//DEFINE - CLOCKS AND TIMERS
#define AHB_freq							(uint32_t) 168000000
#define APB1_freq							(uint32_t)  84000000
#define APB2_freq							(uint32_t)  42000000
#define AHB_freq_8						(uint32_t)  21000000
#define SysTick_Max						(uint32_t)  16777215
#define TIMER6_7_Max					(uint32_t)     65535
#define TIMER6_7_Max_time			(uint32_t)     51129
#define Timer2_5_Max					(uint32_t)     65535
#define Timer2_5_Max_time			(uint32_t)     51129
//DEFINE - SPI
#define PCLK_2								(uint32_t)         0
#define PCLK_4								(uint32_t)         1
#define PCLK_8								(uint32_t)         2
#define PCLK_16								(uint32_t)         3
#define PCLK_32								(uint32_t)         4
#define PCLK_64								(uint32_t)         5
#define PCLK_128							(uint32_t)         6
#define PCLK_256							(uint32_t)         7

//RCC initialization

void RCC_init(void);

//SysTick

void delay_SysTick(volatile uint32_t ms);																					//Delay using SysTick
inline void delay_ticks_SysTick(volatile uint32_t ticks);													//Delay with ticks
void timer_SysTick(volatile uint32_t ticks);																			//Timer (interruptions) using SysTick - use with "on_SysTick_end()"
inline void reset_timer_SysTick(void);																						//RESET SysTick
inline void stop_timer_SysTick(void);																							//STOP SysTick

//TIMER6&7

void delay_TIMER6(volatile uint32_t ms);																					//Delay using TIMER6
void delay_TIMER7(volatile uint32_t ms);																					//Delay using TIMER7
inline void delay_ticks_TIMER6(uint32_t prescaler, uint32_t ticks);								//Delay with ticks
inline void delay_ticks_TIMER7(uint32_t prescaler, uint32_t ticks);								//Delay with ticks
void timer_TIMER6(volatile uint32_t ms);																					//Timer (interruptions) using TIMER6 - use with "on_TIMER6_end()"
void timer_TIMER7(volatile uint32_t ms);																					//Timer (interruptions) using TIMER7 - use with "on_TIMER7_end()"
inline void reset_timer_TIMER6(void);																							//RESET TIMER6
inline void reset_timer_TIMER7(void);																							//RESET TIMER7
inline void stop_timer_TIMER6(void);																							//STOP TIMER6
inline void stop_timer_TIMER7(void);																							//STOP TIMER7

//LED
inline void LED_init(int LED);																										//Initialize chosen LED
void LED_init_all(void);																													//Initialize all LEDs
inline void LED_on(int LED);																											//Turn on chosen LED
inline void LED_off(int LED);																											//Turn off chosen LED
inline void LED_toggle(int LED);																									//Toggle chosen LED
inline void LED_on_all(void);																											//Turn on all LED
inline void LED_off_all(void);																										//Turn off all LED
inline void LED_toggle_all(void);																									//Toggle all LEDs
inline void LED_sweep(uint32_t ms);																								//LED Sweep - 1 cycle

//PWM

void PWM(volatile double f, volatile unsigned int D, int LED); 										//f - frequency D - Duty Cycle: 0-255

//INTERRUPT HANDLERS

extern void on_SysTick_end(void);
extern void on_TIMER6_end(void);
extern void on_TIMER7_end(void);
extern void SysTick_Handler(void);
extern void TIM6_DAC_IRQHandler(void);
extern void TIM7_IRQHandler(void);

//ADDITIONAL

uint32_t get_prescaler(volatile uint32_t ms, uint32_t MAX_VALUE);
uint32_t get_ticks(volatile uint32_t ms, uint32_t prescaler, uint32_t MAX_VALUE);
