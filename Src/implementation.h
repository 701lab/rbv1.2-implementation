#ifndef IMPLEMENTATION_H_
#define IMPLEMENTATION_H_

#include "stm32g071xx.h"


//****** User-adjustable defines ******
	/*
	 * @brief	Those defines used in device initialization thus are very important.
	 *
	 * For proper device work all values should be changed before developing a new project based on this file.
	 */

/*
	@brief contains SYSCLK value AFTER clock setup in Hz

	This value isn't used to setup SYSCLK. The main goal of this value is to be a reference for all time-based setups and clock setup error handling.

	@note This value should be set BY HANDS AFTER system clock setup!

	???
	@recommendation choose value  multiple to 8 Mhz multiple: 8, 16, 24, 32, 40, 48, 56, 64Mhz for reliability improvment - if Pll
 */
#define CLOCK_SPEED 				24000000 // Hz = 24 Mhz


/*
	@

 */

#define MOTORS_PWM_FREQUENSY		20000	// Hz
#define MAX_PWM_WIDTH	 			1199

//****** End of User-adjustable defines ******




uint32_t clock_setup(void);





uint32_t gpio_setup(void);

uint32_t adc_setup(void);

uint32_t intrfaces_setup(void);

uint32_t safe_setup(void);

// TODO: Если необходимо настроить прерывание для какой либо периферии, которая в теории может быть уже включена. Необходимо убедиться, что данная периферия
// 	Сначала будет выключена
uint32_t interrupt_setup(void);

uint32_t full_device_setup(void);




//*** Blinks led if

void blink(void);

void delay_in_milliseconds(const uint32_t * time_in_millisecond);

void delay(const uint32_t time_in_milliseconds);




//*** Non user-adjustable defines  ***
/*
	@brief All defines that should not be changed contains here. Also all checks for #define mistakes happen here redefines happen here so they will be at top of any listing and won't distract programmers.
 */

/* Check for correct system clock define  */
#if (CLOCK_SPEED > 64000000)
	#error "Clock speed define is higher than maximum value of 64 Mhz"

#elif CLOCK_SPEED < 8000000
	#error "Clock speed defin is less then minimum value of 8 Mhz"

#endif


#endif /* IMPLEMENTATION_H_ */
