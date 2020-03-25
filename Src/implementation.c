/*
 * implementation.c
 *
 *  Created on: Mar 24, 2020
 *      Author: Андрей
 */


#include "implementation.h"


/******** MCU connections overview ********

	LEDs:
		PD0-PD3 (digital outputs).

	Motor Controller ( DRV8848 ):
		PC6 - nSleep (digital output) - Logic high to enable device, logic low to enter low-power sleep mode;
		PC7 - nFault (digital input with interrupt) - Pulled logic low with fault condition;
		PA8-PA9 - AIN1-2 (TIM1 PWM CH1-2, alternate function 2) - motor 2 speed control inputs;
		PA10-PA11 - BIN1-2 (TIM1 PWM CH3-4, alternate function 4) - motor 1 speed control inputs.

	Radio module ( NRF24l01+ ):
		PA5-PA7 - SCK, MISO, MOSI respectively (SPI1, alternate function 0);
		PB0 - CE (digital output) - chip enable, logic high to turn on device;
		PB1 - CSN (digital output) - SPI chip select;
		PB2 - IRQ (interrupt input) - Pulled logic low with interrupt condition.

	Motor 1 encoder:
		PB3 - encoder input 2 (TIM2_CH2, alternate function 2),
		PA15 - encoder input 1 (TIM2_CH1, alternate function 2).

	Motor 2 encoder:
		PB4-5 encoder input 1-2 (TIM3 CH1-2, alternate function 1).

	IMU ( ICM-20600 ):
		PB13-PB15 (SPI2 SCK, MISO, MOSI, alternate function 0), PB10-PB11 (INT2, INT1, respectively, external interrupt), PB12 (CS, digital output).

	Voltage control:
		PA0 - input voltage ADC (ADC1 IN0, analog input).

	USART:
		PB6-7 - TX and RX respectively (USART1, alternate function 0).

	TIM14 - high speed counter for time calculations;
	TODO: нужно изучить, как запускаются часы реального времени и наверное использовать для вычисления времени их. Хотят тут станет вопрос, что такие часы могут
	быть не совсем надежными и все же будет оправданно использовать один из таймеров вместо часов реального времени.

	TIM15 - high speed counter for precise speed calculations.

	TIM16 - timer for proper delay implementation

*/


	/*
		@brief system clock half smart setup.

		@Documentation:
		> STM32G0x1 reference manual chapter 5.
	 */


uint32_t clock_setup(void){

	#if	( CLOCK_SPEED > 48000000 )
		FLASH->ACR |= 0x02; // To make clock speed more than 48 MHz flash access time should be 2 cycles
	#endif

		FLASH->ACR |= 0x02; // To make clock speed more than 48 MHz flash access time should be 2 cycles

	RCC->CR |= RCC_CR_HSEON;
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= 0x70000C03; // Set PLLR divider to 4 and enable PLLR output, set PLLN to 12, set HSE as a clock source
	while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY){}

	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY){}	// Wait until PLL starts

	RCC->CFGR &= ~RCC_CFGR_SW_Msk; 	// Clear sw bits
	RCC->CFGR |= RCC_CFGR_SW_1; 	// PLL as clock source
	while((RCC->CFGR & RCC_CFGR_SWS_1) != RCC_CFGR_SWS_1){}
}





uint32_t full_device_setup(void){



	//*** Enable all needed peripherals ***//
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN;
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN | RCC_APBENR1_SPI2EN;
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN |RCC_APBENR2_SPI1EN | RCC_APBENR2_USART1EN | RCC_APBENR2_TIM14EN | RCC_APBENR2_TIM15EN | RCC_APBENR2_ADCEN; //	ADC??

	//*** Port A full GPIO setup ***//
	GPIOA->MODER &=~0xC0FFFC03;
	GPIOA->MODER |= 0x80AAA803;
	GPIOA->OSPEEDR |= 0x00AAA800; 	// High speed for PWM and SPI outputs for better transient
	GPIOA->AFR[1] |= 0x20002222;

	//*** Port B full GPIO setup ***//
	GPIOB->MODER &= ~0xFFF0FFFF;
	GPIOB->MODER |= 0xA900AA85;
	GPIOB->OSPEEDR |= 0xA800A000; 	// High speed for SPI and UART outputs
	GPIOB->AFR[0] |= 0x00112000;

	//*** Port C full GPIO setup ***//
	GPIOC->MODER &=~0xF000;
	GPIOC->MODER |= 0x1000;

	//*** Port D full GPIO setup ***//
	GPIOD->MODER &= ~0x000000FF;
	GPIOD->MODER |= 0x00000055;

	//*** TIM1 PWM setup ***//
	TIM1->PSC = 0; //24Mhz
	TIM1->ARR = MAX_PWM_WIDTH; // 20.000 HZ PWM
	TIM1->CCMR1 |= 0x6868; //PWM mode
	TIM1->CCMR2 |= 0x6868;
	TIM1->CCER |= 0x1111;
	TIM1->CR1 |= 0x80;
	TIM1->BDTR  |=  TIM_BDTR_MOE;
	TIM1->EGR |= 0x01; // update event
	TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
	TIM1->CCR1 = MAX_PWM_WIDTH;
	TIM1->CCR2 = MAX_PWM_WIDTH;
	TIM1->CCR3 = MAX_PWM_WIDTH;
	TIM1->CCR4 = MAX_PWM_WIDTH;

	//*** Timer2 encoder setup ***//
	TIM2->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
	TIM2->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
	TIM2->SMCR |= 0x03;		// Encoder mode setup
	TIM2->CNT = 0;			// Clear counter before start
	TIM2->CR1 |= TIM_CR1_CEN; //TIM_CR1_CEN;

	//*** Timer3 encoder setup ***//
	TIM3->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
//	TIM3->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
	TIM3->SMCR |= 0x03;		// Encoder mode setup
	TIM3->CNT = 0;			// Clear counter before start
	TIM3->CR1 |= TIM_CR1_CEN;

	//*** TIM14 test setup ***//
	// Used to count program time for mistakes log
	TIM14->PSC |= 39999; 	// 24 Mhz clock -> 600 pulses per second into CNT
	TIM14->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so with prescaler it will be around 1 min 47 seconds of time stamps
	TIM14->CNT = 0;			// Clear counter before start
	TIM14->CR1 |= TIM_CR1_CEN;

	//*** TIM15 setup ***//
	// Used to count millisecond for speed calculations
	TIM15->PSC |= 47; 	// 24 Mhz clock -> 500.000 pulses per second into CNT
	TIM15->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so program should have minimum 16 speed calculations per second not to loose data
	TIM15->CNT = 0;			// Clear counter before start
	TIM15->CR1 |= TIM_CR1_CEN;

	//*** USART1 setup ***//
	USART1->BRR = 24000000/19200; //baud rate is 19200
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	//*** SPI1 setup ***//
	SPI1->CR1 |= 0x0314; // set as SPI-master, disable NSS-pin
	SPI1->CR2 |= 0x1000; //
	SPI1->CR1 |= 0x0040; // turn on SPI


	// SPI2 setup //
	SPI2->CR1 |= 0x0314; // set as SPI-master, disable NSS-pin
	SPI2->CR2 |= 0x1000; //
	SPI2->CR1 |= 0x0040; // turn on SPI

	//*** ADC setup ***//
//	ADC->CCR |= 0x00040000;			// Divide clock by 2

	ADC1->CR |= ADC_CR_ADVREGEN;	// Power up ADC

	uint32_t timrrrr = 200;
	delay_in_milliseconds(&timrrrr); // maybe more maybe less
//	ADC1->CR |= ADC_CR_ADCAL;		// Calibrate ADC
//	while(ADC1->CR & ADC_CR_ADCAL){}
	ADC1->CFGR1 |= ADC_CFGR1_CHSELRMOD;
	ADC1->CHSELR |= 0xF0;

	ADC1->CR |= ADC_CR_ADEN;		// Enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY)) {}

//
//	//*** System timer setup ***//
//	SysTick->LOAD = 14999; // 200 times per second at 24 MHZ
//	SysTick->VAL = 0;
//	NVIC_EnableIRQ(SysTick_IRQn);
//	SysTick->CTRL |= 0x03; // Starts SysTick, enables interrupts

	return 0;
}



void blink(void){

	uint32_t temp_time = 1000000;

	GPIOD->ODR ^= 0x0F;
	delay_in_milliseconds(&temp_time);
	GPIOD->ODR ^= 0x0F;
	delay_in_milliseconds(&temp_time);

}

void delay_in_milliseconds(const uint32_t *time_in_millisecond){
	// Temporary implementation
	for(uint32_t iterator = 0; iterator < *time_in_millisecond; ++iterator);
}



