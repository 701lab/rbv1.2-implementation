#include "implementation.h"


/******** MCU connections overview ********

	LEDs:
		PD0-PD3 (digital outputs).

	Motor Controller ( DRV8848 ):
		PC6 - nSleep (digital output) - Logic high to enable device, logic low to enter low-power sleep mode;
		PC7 - nFault (digital input with interrupt) - Pulled logic low with fault condition;
		PA8-PA9 - AIN1-2 (TIM1 PWM CH1-2, alternate function 2) - motor 2 speed control inputs;
		PA10-PA11 - BIN2-1 (TIM1 PWM CH3-4, alternate function 4) - motor 1 speed control inputs.

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

/*!
	@brief 	sets up SYSCLK to CLOCK_SPEED with taking into account problems with different sources

	Tries to start up HSE. Calls PLL startup functions with HSE as a source if it started correctly and HSI otherwise.

	@return	mistake code. If HSE fails and PLL starts - 3, if PLL fails with any source - PLL fail code.

	@Documentation:
	> STM32G0x1 reference manual chapter 3 (Flash) - flash access latency (3.3.4);
	> STM32G0x1 reference manual chapter 5 (RCC) - all information about clock setup.

	@function completeness = 60% |    ######|

	TODO:
 */
uint32_t system_clock_setup(void)
{

	/* Flash read access latency from Clock_speed. See RM chapter 3.3.4 */
	#if	( CLOCK_SPEED > 48000000 )

		FLASH->ACR |= FLASH_ACR_LATENCY_1; // 0x02 - 2 clock cycles latency

	#elif ( CLOCK_SPEED > 24000000 )

		FLASH->ACR |= FLASH_ACR_LATENCY_0;	// 0x01 - 1 clock cycle latency

	#endif

	RCC->CR |= RCC_CR_HSEON;

	for ( uint32_t i = 0; i < DUMMY_DELAY_VALUE; ++i )
	{
		if ( (RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY )
		{
			// Enable clock security system
			RCC->CR |= RCC_CR_CSSON;

			const uint32_t pll_setup_return_value = pll_setup(HSE_IS_OK);

			if ( pll_setup_return_value == 0 )
			{
				// Everything is ok. HSI will be stopped for so it won't consume energy
				RCC->CR &= ~RCC_CR_HSION_Msk;
				return 0;
			}
			else
			{
				RCC->CR &= ~RCC_CR_HSEON;		// Stop HSE so it won't consume power.
				return pll_setup_return_value;	// PLL fails, so HSI is a clock source. PLL error code either 1 or 2
			}
		}
	}

	RCC->CR &= ~RCC_CR_HSEON;	// Stop HSE so it won't consume power.

	const uint32_t pll_setup_return_value = pll_setup(HSE_IS_NOT_OK);

	if ( pll_setup_return_value )
	{
		return pll_setup_return_value;
	}

	return 3; // HSE fail code
}

/*!
	@brief	starts PLL. Takes into account HSE status

	Tries to start PLL with given CLOCK_SPEED.

	@param[in] is_HSE_clock_source - status of HSE. If HSE is ok uses it as a clock source, uses HSI otherwise.

	@return		Mistake code: 0 - if everything s ok, 1 - if PLL start fail, 2 - if PLL fails to start as a clock source

	@limitations 	For the sake of simplifying usage of the code only one multiplier of the frequency calculation equation should be variable.
					Only frequencies multiple to 2Mhz and starting from 16Mhz are allowed: 16, 18, 20 ... 62, 64Mhz. Because values of PLLN lower then 8 and higher then 43 are not allowed.
					This limitation allows switching frequency source between PLL on HSI and HSE without troubles.

	@Documentation:
		> STM32G0x1 reference manual chapter 5 (RCC) - all information about clock setup.

	@Calculations
	> When using 8 Mhz HSE (external oscillator) as a clock sours for PLL (default setup).

		We take:

		PLLR = 4
		PLLM = 1
		PLLN = CLOCK_SPEED / 2000000

					oscillator_frequensy * PLLN       8 * PLLN
		SYSCLK =  ------------------------------- = ------------
						   PLLM * PLLR				   1 * 4

		PLLN can be changed from 8 to 86, but only 8 to 43 are allowed, so we can have any frequency from 2 to 64 Mhz with a step of 2 Mhz and change in only one variable

	> When using 16 Mhz HSI (internal oscillator) as a clock sours for PLL (critical - when HSE can't be enabled)

		We take:

		PLLR = 4
		PLLM = 2
		PLLN = CLOCK_SPEED / 2000000

					oscillator_frequensy * PLLN       16 * PLLN
		SYSCLK =  ------------------------------- = ------------
						   PLLM * PLLR				   2 * 4

		PLLN can be changed from 8 to 86, but only 8 to 43 are allowed, so we can have any frequency from 2 to 64 Mhz with a step of 2 Mhz and change in only one variable

	@note 	If PLL is not working (i don't know if it is really possible) HSI is used as a clock source no matter of HSE status.

	@note 	AHB prescaler always equals to 1, so HCLK = SYSCLK

	@note 	APB prescaler always equals to 1
 */
uint32_t pll_setup(uint32_t is_HSE_clock_source)
{
	RCC->PLLCFGR = 0;

	if ( is_HSE_clock_source == HSE_IS_OK )
	{
		// No need for PLLM
		RCC->PLLCFGR |= ((PLLR_VALUE - PLL_OFFSET) << RCC_PLLCFGR_PLLR_Pos)
				| RCC_PLLCFGR_PLLREN | (PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos)
				| RCC_PLLCFGR_PLLSRC_HSE;
	}
	else
	{
		RCC->PLLCFGR |= ((PLLR_VALUE - PLL_OFFSET) << RCC_PLLCFGR_PLLR_Pos)
				| RCC_PLLCFGR_PLLREN | (PLLN_VALUE << RCC_PLLCFGR_PLLN_Pos)
				| ((PLLM_VALUE_WITH_HSI - PLL_OFFSET) << RCC_PLLCFGR_PLLM_Pos)
				| RCC_PLLCFGR_PLLSRC_HSI;
	}

	RCC->CR |= RCC_CR_PLLON;

	uint32_t safety_delay_counter = 0;

	while ( (RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY )
	{
		++safety_delay_counter;
		if ( safety_delay_counter > DUMMY_DELAY_VALUE )
		{
			RCC->CR &= ~RCC_CR_PLLON;	// Stop PLL so it won't consume power.
			return 1;					// PLL startup fail code
		}
	}

	// At that point PLL is on and we can use it as a SYSCLK source
	RCC->CFGR &= ~RCC_CFGR_SW_Msk;
	RCC->CFGR |= RCC_CFGR_SW_1;

	safety_delay_counter = 0;

	while ( (RCC->CFGR & RCC_CFGR_SWS_1) != RCC_CFGR_SWS_1 )
	{
		++safety_delay_counter;
		if ( safety_delay_counter > DUMMY_DELAY_VALUE )
		{
			RCC->CFGR &= ~RCC_CFGR_SW_Msk;	// Use HSI as a clock source
			return 2;						// PLL as clock source startup fail code
		}
	}

	return 0;	// Everything is fine and online
}

/*
	@brief	Interrupt handler. Called when HSE fails. Tries to start PLL with HSI as a source

	@Documentation:
		> STM32G0x1 reference manual chapter 5 (RCC) - clock security system (5.2.8).

	Right now I don't know if this interrupt can be called by any other system.
	Because of it it doesn't check for call reason but can be changed in the future.
 */
void NMI_Handler()
{
	// Clear the clock security system interrupt flag
	RCC->CICR |= RCC_CICR_CSSC;

	// Wait until PLL is fully stopped
	while ( (RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY ){}

	// Isn't trying to use HSA, uses HSI instead. Tries to start PLL.
	const uint32_t pll_setup_return_value = pll_setup(HSE_IS_NOT_OK);

	if (pll_setup_return_value)
	{
		// PLL setup fail code handling
	}
		// Else PLL started correctly

	// The place for error handling capabilities. The program should log that HSE problem occurred

}

/*
	@brief	Sets up GPIO for all needed on device functions

	@Documentation
		> STM32G0x1 reference manual chapter 5 (RCC) - all information about peripheral locations and enabling.
		> STM32G0x1 reference manual chapter 6 (GPIO) - information about GPIO setup.
		> STM32G071x8/xb datasheet chapter 4 (Pinouts, pin description and alternate functions) - information about alternate function on the pins.

	@note	In stm32g0 all GPIO are in analog mode by default (0b11)
 */
void gpio_setup(void)
{
	//*** Enable GPIO ***//
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN;

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
}



/*
	@brief	Sets up all used on the board timers and enables desired interrupts

	@Documentation:
		> STM32G0x1 reference manual chapter 20 (TIM1) - TIM1 setup information;
		> STM32G0x1 reference manual chapter 21 (TIM2/TIM3) - TIM2 and TIM3 setup information;
		> STM32G0x1 reference manual chapter 21 (TIM2/TIM3) - TIM2 and TIM3 setup information;

	Timer channels allocations with respect to function.

	Timer 			| function
	---------------	| ----------------------------------------------------
	TIM1			| generates PWM for motor control on all 4 channels
	TIM1 CH1		| DRV8848 AIN1
	TIM1 CH2 		| DRV8848 AIN2
	TIM1 CH3		| DRV8848 BIN2
	TIM1 CH4		| DRV8848 BIN1
	--------------- | ----------------------------------------------------
	TIM2			| Motor1 encoder pulses counter
	TIM2 CH1		| Motor1 encoder phase A counter
	TIM2 CH2		| Motor1 encoder phase B counter
	--------------- | ----------------------------------------------------
	TIM3			| Motor2 encoder pulses counter
	TIM3 CH1		| Motor2 encoder phase A counter
	TIM3 CH2		| Motor2 encoder phase B counter
	--------------- | ----------------------------------------------------
	TIM14			| ??
	--------------- | ----------------------------------------------------
	TIM15 			| ??

 */
void timers_setup(void)
{
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN;
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
//	RCC->APBENR2 |= RCC_APBENR2_TIM1EN | RCC_APBENR2_TIM14EN | RCC_APBENR2_TIM15EN;

	//*** TIM1 PWM setup ***//
	TIM1->PSC = 0; 				// Timer speed = bass speed
	TIM1->ARR = MAX_PWM_WIDTH; 	//
	TIM1->CCMR1 |= 0x6868; //PWM mode
	TIM1->CCMR2 |= 0x6868;
	TIM1->CCER |= 0x1111;
	TIM1->CR1 |= 0x80;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->EGR |= 0x01; // update event
	TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
//	TIM1->CCR1 = MAX_PWM_WIDTH/2;
//	TIM1->CCR2 = MAX_PWM_WIDTH;
//	TIM1->CCR3 = MAX_PWM_WIDTH;
	TIM1->CCR4 = MAX_PWM_WIDTH/2;

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

//		//*** TIM14 test setup ***//
//		// Used to count program time for mistakes log
//		TIM14->PSC |= 39999; 	// 24 Mhz clock -> 600 pulses per second into CNT
//		TIM14->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so with prescaler it will be around 1 min 47 seconds of time stamps
//		TIM14->CNT = 0;			// Clear counter before start
//		TIM14->CR1 |= TIM_CR1_CEN;
//
//		//*** TIM15 setup ***//
//		// Used to count millisecond for speed calculations
//		TIM15->PSC |= 47; 	// 24 Mhz clock -> 500.000 pulses per second into CNT
//		TIM15->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so program should have minimum 16 speed calculations per second not to loose data
//		TIM15->CNT = 0;			// Clear counter before start
//		TIM15->CR1 |= TIM_CR1_CEN;

//		//*** System timer setup ***//
//		SysTick->LOAD = 14999; // 200 times per second at 24 MHZ
//		SysTick->VAL = 0;
//		NVIC_EnableIRQ(SysTick_IRQn);
//		SysTick->CTRL |= 0x03; // Starts SysTick, enables interrupts
}





uint32_t full_device_setup(void){



	//*** Enable all needed peripherals ***//
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN;
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN | RCC_APBENR1_SPI2EN;
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN |RCC_APBENR2_SPI1EN | RCC_APBENR2_USART1EN | RCC_APBENR2_TIM14EN | RCC_APBENR2_TIM15EN | RCC_APBENR2_ADCEN; //	ADC??

//	//*** Port A full GPIO setup ***//
//	GPIOA->MODER &=~0xC0FFFC03;
//	GPIOA->MODER |= 0x80AAA803;
//	GPIOA->OSPEEDR |= 0x00AAA800; 	// High speed for PWM and SPI outputs for better transient
//	GPIOA->AFR[1] |= 0x20002222;
//
//	//*** Port B full GPIO setup ***//
//	GPIOB->MODER &= ~0xFFF0FFFF;
//	GPIOB->MODER |= 0xA900AA85;
//	GPIOB->OSPEEDR |= 0xA800A000; 	// High speed for SPI and UART outputs
//	GPIOB->AFR[0] |= 0x00112000;
//
//	//*** Port C full GPIO setup ***//
//	GPIOC->MODER &=~0xF000;
//	GPIOC->MODER |= 0x1000;
//
//	//*** Port D full GPIO setup ***//
//	GPIOD->MODER &= ~0x000000FF;
//	GPIOD->MODER |= 0x00000055;

//	//*** TIM1 PWM setup ***//
//	TIM1->PSC = 0; //24Mhz
//	TIM1->ARR = MAX_PWM_WIDTH; // 20.000 HZ PWM
//	TIM1->CCMR1 |= 0x6868; //PWM mode
//	TIM1->CCMR2 |= 0x6868;
//	TIM1->CCER |= 0x1111;
//	TIM1->CR1 |= 0x80;
//	TIM1->BDTR  |=  TIM_BDTR_MOE;
//	TIM1->EGR |= 0x01; // update event
//	TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
//	TIM1->CCR1 = MAX_PWM_WIDTH;
//	TIM1->CCR2 = MAX_PWM_WIDTH;
//	TIM1->CCR3 = MAX_PWM_WIDTH;
//	TIM1->CCR4 = MAX_PWM_WIDTH;
//
//	//*** Timer2 encoder setup ***//
//	TIM2->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
//	TIM2->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
//	TIM2->SMCR |= 0x03;		// Encoder mode setup
//	TIM2->CNT = 0;			// Clear counter before start
//	TIM2->CR1 |= TIM_CR1_CEN; //TIM_CR1_CEN;
//
//	//*** Timer3 encoder setup ***//
//	TIM3->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
////	TIM3->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
//	TIM3->SMCR |= 0x03;		// Encoder mode setup
//	TIM3->CNT = 0;			// Clear counter before start
//	TIM3->CR1 |= TIM_CR1_CEN;
//
//	//*** TIM14 test setup ***//
//	// Used to count program time for mistakes log
//	TIM14->PSC |= 39999; 	// 24 Mhz clock -> 600 pulses per second into CNT
//	TIM14->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so with prescaler it will be around 1 min 47 seconds of time stamps
//	TIM14->CNT = 0;			// Clear counter before start
//	TIM14->CR1 |= TIM_CR1_CEN;
//
//	//*** TIM15 setup ***//
//	// Used to count millisecond for speed calculations
//	TIM15->PSC |= 47; 	// 24 Mhz clock -> 500.000 pulses per second into CNT
//	TIM15->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so program should have minimum 16 speed calculations per second not to loose data
//	TIM15->CNT = 0;			// Clear counter before start
//	TIM15->CR1 |= TIM_CR1_CEN;

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



