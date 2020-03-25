#ifndef IMPLEMENTATION_H_
#define IMPLEMENTATION_H_

#include "stm32g071xx.h"

// ?? Do i really need it??
/* NULL pointer definition */
#ifndef NULL
#define NULL ((void*)0)
#endif


#define CLOCK_SPEED 				24000000 // 24 Mhz
#define MAX_PWM_WIDTH	 			1199	 //

uint32_t device_setup(void);

//*** Blinks led if

void blink(void);

void delay_in_milliseconds(const uint32_t * time_in_millisecond);

void delay(const uint32_t time_in_milliseconds);



#endif /* IMPLEMENTATION_H_ */
