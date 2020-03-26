#ifndef IMPLEMENTATION_H_
#define IMPLEMENTATION_H_

#include "stm32g071xx.h"

//*************************************//
//****** User-adjustable defines ******//
//*************************************//

/*
	@brief 	Contains desired SYSCLK and HCLK values in Hz

	Used in all time related things: SYSCLK setup, general purpose timers setup, PWM setup, time calculation and so forth.

	@note 	Only frequencies multiple to 2Mhz are allowed and PLLN is has a minimum value of 8.
	 	 	Allowed frequencies are: 16000000, 18000000 ... 62000000, 64000000 Hz.

 */
#define CLOCK_SPEED 				48000000 // Hz = 24 Mhz


/*
	@

 */

#define MOTORS_PWM_FREQUENSY		20000	// Hz
#define MAX_PWM_WIDTH	 			1199

//****** End of User-adjustable defines ******//


//****************************//
//****** User functions ******//
//****************************//
// 	@brief	Can be called in the code by the programmer while developing applications

/*
	@brief	Sets up SYSCLK to CLOCK_SPEED with taking into account problems with different sources
 */
uint32_t system_clock_setup(void);

/*
	@brief	Sets up GPIO for all needed on device functions
 */
void gpio_setup(void);

/*
	@brief	Sets up all used on the board timers and enables desired interrupts
 */
void timers_setup(void);

uint32_t full_device_setup(void);

//********************************//
//****** Non-User functions ******//
//********************************//
//	@brief	Called only by the system when needed. Should not be used in the application code

/*
	@brief	sets up SYSCLK to CLOCK_SPEED with taking into account problems with different sources
 */
uint32_t pll_setup(uint32_t is_HSE_clock_source);

void NMI_Handler(); //!!!!!!

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
#if (CLOCK_SPEED > 64000000)
	#error Clock speed define is higher than maximum value of 64 Mhz

#elif CLOCK_SPEED < 16000000
	#error Clock speed define is less then minimum value of 16 Mhz

#endif

/* System clock prescalers*/
#define	PLLN_VALUE  			CLOCK_SPEED/2000000
#define PLLR_VALUE				4
#define PLL_OFFSET				1					// Offset  for PLLM and PLLR
#define PLLM_VALUE_WITH_HSI		2

/* HSE state */
#define HSE_IS_OK 		1
#define HSE_IS_NOT_OK 	0

/* Completely random value to determine the waiting-state length */
#define DUMMY_DELAY_VALUE 10000


#endif /* IMPLEMENTATION_H_ */
