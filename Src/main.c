
#include "implementation.h"


uint32_t iterator = 0;


int main(void)
{

	device_setup();

	for(;;){

		GPIOD->ODR ^= 0x03;
		for(iterator = 0; iterator < 5000; iterator++){
			__NOP();
		}
//		blink();
	}
}
