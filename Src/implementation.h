#ifndef IMPLEMENTATION_H_
#define IMPLEMENTATION_H_

#include "stm32g071xx.h"

//*************************************//
//****** User-adjustable defines ******//
//*************************************//

/*
	@brief 	Contains desired SYSCLK and HCLK values in Hz.

	Used in all time related things: SYSCLK setup, general purpose timers setup, PWM setup, time calculation and so forth.

	@note 	Only frequencies multiple to 2Mhz are allowed and PLLN is has a minimum value of 8.
	 	 	Allowed frequencies are: 16000000, 18000000 ... 62000000, 64000000 Hz.

 */
#define SYSCLK_FREQUENCY 			48000000 // Hz = 24 Mhz

/*
	@brief 	PWM frequency for motor control in Hz and related PWM precision

	@Calculations	PWM precision can be calculated as:

					 SYSCLK_FREQUENCY
	PWM_precision = ------------------
 	 	 	 	 	  PWM_FREQUENCY
 */
#define PWM_FREQUENCY 				20000	// Hz = 20 Khz
#define PWM_PRECISION				(SYSCLK_FREQUENCY / PWM_FREQUENCY - 1)	// -1 is needed for proper timers setup. Shouldn't be changed

/*
	@brief	Amount of times system clock interrupt occurs in one second

	Determines control system outer loop frequency
 */
#define SYSTICK_FREQUENCY			1000		// Hz

//****** End of User-adjustable defines ******//


//****************************//
//****** User functions ******//
//****************************//
// @brief	Can be called in the code by the programmer while developing applications.

// @brief	Sets up SYSCLK to SYSCLK_FREQUENCY with taking into account problems with different sources
uint32_t system_clock_setup(void);

// @brief	Sets up all desired device peripherals
uint32_t full_device_setup(void);



//********************************//
//****** Non-User functions ******//
//********************************//
//	@brief	Called only by the system when needed. Should not be used in the application code

// @brief	Sets up PLL with respect to input source and SYSCLK_FREQUENCY
uint32_t pll_setup(uint32_t is_HSE_clock_source);

// @brief	Non-maskable interrupt handler. Called when HSE fails. Tries to start SYSCLK with PLL with HSI as a clock source.
void NMI_Handler();

// @brief	Sets up GPIO for all needed on device functions
void gpio_setup(void);

// @brief	Sets up all used on the board timers and enables desired interrupts
void timers_setup(void);


uint32_t adc_setup(void);

uint32_t intrfaces_setup(void);

uint32_t safe_setup(void);

// TODO: Если необходимо настроить прерывание для какой либо периферии, которая в теории может быть уже включена. Необходимо убедиться, что данная периферия
// 	Сначала будет выключена
uint32_t interrupt_setup(void);





//*** Blinks led if

void blink(void);

void delay_in_milliseconds(const uint32_t * time_in_millisecond);

void delay(const uint32_t time_in_milliseconds);


/*** Non user-adjustable defines  ***/
/*
	@brief All defines that should not be changed contains here. Also all checks for #define mistakes happen here redefines happen here so they will be at top of any listing and won't distract programmers.
 */

/* Check for correct system clock define  */
#if (SYSCLK_FREQUENCY > 64000000)
	#error Clock speed define is higher than maximum value of 64 Mhz

#elif (SYSCLK_FREQUENCY < 16000000)
	#error Clock speed define is less then minimum value of 16 Mhz

#endif


//*** GPIO setup defines ***//
#define GPIO_MODER_MSK			(3U) 	//0x11
#define GPIO_ANALOG_IN 			(3U)	//0x11
#define GPIO_ALTERNATE			(2U)	//0x10
#define GPIO_DIGITAL_OUT		(1U)	//0x01
#define GPIO_DIGITAL_IN			(0U)	//0x00

#define GPIO_OSPEED_VERY_LOW	(0U)	//0x00
#define GPIO_OSPEED_LOW			(1U)	//0x01
#define GPIO_OSPEED_HIGH		(2U)	//0x10
#define GPIO_OSPEED_VERY_HIGH	(3U)	//0x11

#define ALTERNATE_FUNCTION_1	(1U)	//0x01
#define ALTERNATE_FUNCTION_2	(2U)	//0x10

//*** System clock prescalers ***//
#define	PLLN_VALUE  			SYSCLK_FREQUENCY/2000000
#define PLLR_VALUE				4
#define PLL_OFFSET				1					// Offset  for PLLM and PLLR
#define PLLM_VALUE_WITH_HSI		2

//*** HSE state defines ***//
#define HSE_IS_OK 		1
#define HSE_IS_NOT_OK 	0

/* Completely random value to determine the waiting-state length */
#define DUMMY_DELAY_VALUE 10000


#endif /* IMPLEMENTATION_H_ */
