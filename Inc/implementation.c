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

	TIM14 - counts time with 0.1 ms precision for mistakes log

	TIM15 - high speed counter for precise speed calculations.

	TIM16 - timer for proper delay implementation

*/

/*
	@brief Makes log entry with given mistake code at current time.
		If mistake code is equal to 0 doens't log it

	@param[in] mistake_code - mistakes code to be written to log.
 */
void add_to_mistakes_log(uint32_t mistake_code)
{
	if(mistake_code == 0)
	{
		return;
	}

	mistakes_log[mistakes_log_pointer].mistake_code = mistake_code;
	mistakes_log[mistakes_log_pointer].mistake_time_in_minuts = time_from_log_enable_in_minutes;
	mistakes_log[mistakes_log_pointer].mistake_time_in_seconds = TIM14->CNT;

	++mistakes_log_pointer;
	if (mistakes_log_pointer == MISTAKES_LOG_SIZE)
	{
		mistakes_log_pointer = 0;
	}
}

/*
	@brief TIM14 overflow interrupt handler - counts minutes from enable of mistakes log.

	@note If device have EEprom - saves mistake log into it every minute.
 */
void TIM14_IRQHandler()
{
	TIM14->SR &= ~TIM_SR_UIF;

	++time_from_log_enable_in_minutes;

	// Place to put code to write into EEPROM (for development of future devices)
}



/*!
	@brief 	sets up SYSCLK to SYSCLK_FREQUENCY with taking into account problems with different sources

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

	/* Flash read access latency from SYSCLK_FREQUENCY. See RM chapter 3.3.4 */
	#if	( SYSCLK_FREQUENCY > 48000000 )
		FLASH->ACR |= FLASH_ACR_LATENCY_1; 	// 0x02 - 2 clock cycles latency

	#elif ( SYSCLK_FREQUENCY > 24000000 )
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
				RCC->CR &= ~RCC_CR_HSEON;				// Stop HSE so it won't consume power.
				FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;	// Reset FLASH latency because HSI is 16 Mhz
				return PLL_FAILED;						// PLL fails, so HSI is a clock source
			}
		}
	}

	RCC->CR &= ~RCC_CR_HSEON;	// Stop HSE so it won't consume power.
	add_to_mistakes_log(HSE_FAILED_TO_START);

	const uint32_t pll_setup_return_value = pll_setup(HSE_IS_NOT_OK);

	if ( pll_setup_return_value )
	{
		return pll_setup_return_value;
	}

	return 3; // HSE fail code
}

/*!
	@brief	starts PLL. Takes into account HSE status

	@param[in] is_HSE_clock_source - status of HSE. If HSE is ok uses it as PLL clock source, uses HSI otherwise.

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
		PLLN = SYSCLK_FREQUENCY / 2000000

					oscillator_frequensy * PLLN       8 * PLLN
		SYSCLK =  ------------------------------- = ------------
						   PLLM * PLLR				   1 * 4

		PLLN can be changed from 8 to 86, but only 8 to 43 are allowed, so we can have any frequency from 16 to 64 Mhz with a step of 2 Mhz and change in only one variable

	> When using 16 Mhz HSI (internal oscillator) as a clock sours for PLL (critical - when HSE can't be enabled)

		We take:

		PLLR = 4
		PLLM = 2
		PLLN = SYSCLK_FREQUENCY / 2000000

					oscillator_frequensy * PLLN       16 * PLLN
		SYSCLK =  ------------------------------- = ------------
						   PLLM * PLLR				   2 * 4

		PLLN can be changed from 8 to 86, but only 8 to 43 are allowed, so we can have any frequency from 16 to 64 Mhz with a step of 2 Mhz and change in only one variable

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
			add_to_mistakes_log(PLL_FAILED);
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
			FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;	// Reset FLASH latency because HSI is 16 Mhz
			RCC->CFGR &= ~RCC_CFGR_SW_Msk;			// Use HSI as a clock source
			add_to_mistakes_log(PLL_FAILED);
			return 1;								// PLL as clock source startup fail code
		}
	}

	return 0;	// Everything is fine and online
}

/*
	@brief	Non-maskable interrupt handler. Called when HSE fails. Tries to start PLL with HSI as a source

	@Documentation:
		> STM32G0x1 reference manual chapter 5 (RCC) - clock security system (5.2.8).

	Right now I don't know if this interrupt can be called by any other system.
	Because of it it doesn't check for call reason but can be changed in the future.
 */
void NMI_Handler()
{
	// Clear the clock security system interrupt flag
	RCC->CICR |= RCC_CICR_CSSC;

	add_to_mistakes_log(HSE_FAILED_WHILE_RUNNING);

	// Wait until PLL is fully stopped
	while ( (RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY ){}

	// Isn't trying to use HSA, uses HSI instead. Tries to start PLL.
	const uint32_t pll_setup_return_value = pll_setup(HSE_IS_NOT_OK);

	if (pll_setup_return_value)
	{
		add_to_mistakes_log(PLL_FAILED);
	}
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
	//*** GPIO peripheral clock enable ***//
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIODEN;

	//*** Port A full GPIO setup ***//
	GPIOA->MODER &= ~((GPIO_MODER_MSK << GPIO_MODER_MODE0_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE5_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE6_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE7_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE8_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE9_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE10_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE11_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE15_Pos));		// Equal to GPIOA->MODER &= ~0xC0FFFC03;

	GPIOA->MODER |= (GPIO_ANALOG_IN << GPIO_MODER_MODE0_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE5_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE6_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE7_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE8_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE9_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE10_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE11_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE15_Pos);		// Equal to GPIOA->MODER |= 0x80AAA803;

	GPIOA->OSPEEDR |= (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED5_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED6_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED7_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED8_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED9_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED10_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED11_Pos);	// Equal to GPIOA->OSPEEDR |= 0x00AAA800;

	GPIOA->AFR[1] |= (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL8_Pos)
			| (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL9_Pos)
			| (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL10_Pos)
			| (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL11_Pos)
			| (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL15_Pos);	// Equal to GPIOA->AFR[1] |= 0x20002222

	//*** Port B full GPIO setup ***//
	GPIOB->MODER &= ~((GPIO_MODER_MSK << GPIO_MODER_MODE0_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE1_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE2_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE3_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE4_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE5_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE6_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE7_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE10_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE11_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE12_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE13_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE14_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE15_Pos));		// GPIOB->MODER &= ~0xFFF0FFFF;

	GPIOB->MODER |= (GPIO_DIGITAL_OUT << GPIO_MODER_MODE0_Pos)
			| (GPIO_DIGITAL_OUT << GPIO_MODER_MODE1_Pos)
			| (GPIO_DIGITAL_IN << GPIO_MODER_MODE2_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE3_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE4_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE5_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE6_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE7_Pos)
			| (GPIO_DIGITAL_IN<< GPIO_MODER_MODE10_Pos)
			| (GPIO_DIGITAL_IN << GPIO_MODER_MODE11_Pos)
			| (GPIO_DIGITAL_OUT << GPIO_MODER_MODE12_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE13_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE14_Pos)
			| (GPIO_ALTERNATE << GPIO_MODER_MODE15_Pos);		// Equals to GPIOB->MODER |= 0xA900AA85;


	GPIOB->OSPEEDR |= (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED6_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED7_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED13_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED14_Pos)
			| (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED15_Pos);	// Equals to GPIOB->OSPEEDR |= 0xA800A000;

	GPIOB->AFR[0] |= (ALTERNATE_FUNCTION_2 << GPIO_AFRH_AFSEL11_Pos)
			| (ALTERNATE_FUNCTION_1 << GPIO_AFRH_AFSEL12_Pos)
			| (ALTERNATE_FUNCTION_1 << GPIO_AFRH_AFSEL13_Pos);	// Equals to GPIOB->AFR[0] |= 0x00112000;

	// Temporary interrupt initialization
	EXTI->FTSR1 |= 0x04; // enable pb2 interrupt
	NVIC_EnableIRQ(EXTI2_3_IRQn);


	//*** Port C full GPIO setup ***//
	GPIOC->MODER &= ~((GPIO_MODER_MSK << GPIO_MODER_MODE6_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE7_Pos));		// Equal to GPIOC->MODER &=~0xF000;

	GPIOC->MODER |= (GPIO_DIGITAL_OUT << GPIO_MODER_MODE6_Pos)
			| (GPIO_DIGITAL_IN << GPIO_MODER_MODE7_Pos);		// Equal to GPIOC->MODER |= 0x1000;

	//*** Port D full GPIO setup ***//
	GPIOD->MODER &= ~((GPIO_MODER_MSK << GPIO_MODER_MODE0_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE1_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE2_Pos)
			| (GPIO_MODER_MSK << GPIO_MODER_MODE3_Pos));		// Equal to GPIOD->MODER &= ~0x000000FF;

	GPIOD->MODER |= (GPIO_DIGITAL_OUT << GPIO_MODER_MODE0_Pos)
			| (GPIO_DIGITAL_OUT << GPIO_MODER_MODE1_Pos)
			| (GPIO_DIGITAL_OUT << GPIO_MODER_MODE2_Pos)
			| (GPIO_DIGITAL_OUT << GPIO_MODER_MODE3_Pos);		// Equal to GPIOD->MODER |= 0x00000055;
}

/*!
	@brief	Sets up all used on the board timers and enables desired interrupts

	@Documentation:
		> STM32G0x1 reference manual chapter 5 (RCC) - all information about peripheral locations and enabling;
		> STM32G0x1 reference manual chapter 20 (TIM1) - TIM1 setup information;
		> STM32G0x1 reference manual chapter 21 (TIM2/TIM3) - TIM2 and TIM3 setup information;
		> STM32G0x1 reference manual chapter 23 (TIM14) - TIM14 setup information;
	 	> Cortex-M0+ programming manual for stm32 chapter 4 - SysTick timer (STK)(4.4).

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
	TIM14			| Counts time in 0.125 millisecond steps for debug log.
					| Resets exactly every minute
					|

	@calculation

	TIM14_prescaler = -----------------
 */
void timers_setup(void)
{
	//*** Timers peripheral clock enable ***//
	RCC->APBENR1 |= RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN;
	RCC->APBENR2 |= RCC_APBENR2_TIM1EN | RCC_APBENR2_TIM14EN | RCC_APBENR2_TIM16EN;
//	RCC->APBENR2 |= RCC_APBENR2_TIM1EN | RCC_APBENR2_TIM14EN | RCC_APBENR2_TIM15EN;

	//*** TIM1 PWM setup ***//
	TIM1->PSC = 0; 				// Timer speed = bus speed
	TIM1->ARR = PWM_PRECISION;
	TIM1->CCMR1 |= 0x6868; 	// PWM mode 1 enable and output compare preload enable on channels 1 and 2
	TIM1->CCMR2 |= 0x6868;	// PWM mode 1 enable and output compare preload enable on channels 3 and 4
	TIM1->CCER |= 0x1111;	// Enable CH1-4
	TIM1->CR1 |= TIM_CR1_ARPE;	// Enable Auto reload preload
	TIM1->BDTR |= TIM_BDTR_MOE;	// Main output enable
	TIM1->CR1 |= TIM_CR1_CEN; // Enable timer
	TIM1->CCR1 = PWM_PRECISION;
	TIM1->CCR2 = PWM_PRECISION;
	TIM1->CCR3 = PWM_PRECISION;
	TIM1->CCR4 = PWM_PRECISION;

	//*** Timer2 encoder setup ***//
	TIM2->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
//	TIM2->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
	TIM2->SMCR |= 0x03;		// Encoder mode setup
	TIM2->CNT = 0;			// Clear counter before start
	TIM2->CR1 |= TIM_CR1_CEN;

	//*** Timer3 encoder setup ***//
	TIM3->ARR = 65535; 		// 2^16-1 - maximum value for this timer. No prescaler, so timer is working with max speed
	TIM3->CCER |= 0x02;		// Should be uncommented if encoder direction reversal is needed
	TIM3->SMCR |= 0x03;		// Encoder mode setup
	TIM3->CNT = 0;			// Clear counter before start
	TIM3->CR1 |= TIM_CR1_CEN;

	//*** TIM14 test setup ***//
	TIM14->PSC |= (uint32_t)(SYSCLK_FREQUENCY / 28800 - 1); //
	TIM14->ARR = 35999; 	// 60 second * 60 millisecond * 10 - 1 to get 0.1 milliseconds step
	TIM14->CNT = 0;			// Clear counter before start
	TIM14->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM14_IRQn);
	TIM14->CR1 |= TIM_CR1_CEN;
//
//		//*** TIM15 setup ***//
//		// Used to count millisecond for speed calculations
//		TIM15->PSC |= 47; 	// 24 Mhz clock -> 500.000 pulses per second into CNT
//		TIM15->ARR = 65535; 	// 2^16-1 - maximum value for this timer, so program should have minimum 16 speed calculations per second not to loose data
//		TIM15->CNT = 0;			// Clear counter before start
//		TIM15->CR1 |= TIM_CR1_CEN;

	//*** TIM16 setup ***//
	// Used to count millisecond for speed calculations
	TIM16->PSC |= (uint32_t)(SYSCLK_FREQUENCY / 1000 - 1); 	// One millisecond step
	TIM16->CNT = 0;			// Clear counter before start
	TIM16->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM16_IRQn);
	TIM16->CR1 |= TIM_CR1_OPM;	// One pulse mode. Counter don't need to be started

	//*** System timer setup ***//
	SysTick->LOAD = SYSCLK_FREQUENCY / (8 * SYSTICK_FREQUENCY) - 1;
	SysTick->VAL = 0;
	NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->CTRL |= 0x03; // Start SysTick, enable interrupt
}

/*!
	@brief	Enable UART transmission and reception with given baud rate

	@param[in] transmission_speed_in_bauds UART desired baud rate

	Leaves all other parameters unchanged (the most basic UART configuration). So UART parameters are:
	> transmission speed = @param[in] transmission_speed_in_bauds;
	> 8 Data bits
	> 1 stop bit
	> parity control disabled
	> no interrupts are enbabled
	> no DMA
	> FIFO disabled
	> etc (not advanced features at all)

	@Doucementation
		> STM32G0x1 reference manual chapter 5 (RCC) - all information about peripheral locations and enabling;
		> STM32G0x1 reference manual chapter 32 (USART/UART) - UART setup information.
 */
void basic_uart1_setup(const uint32_t transmission_speed_in_bauds)
{
	RCC->APBENR2 |= RCC_APBENR2_USART1EN;

	USART1->BRR = SYSCLK_FREQUENCY/transmission_speed_in_bauds;
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
}

// @brief Sends given byte when TX buffer is empty
void uart1_send_byte(const uint8_t message_byte)
{
	while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC){}
	while((USART1->ISR & USART_ISR_TXE_TXFNF) != USART_ISR_TXE_TXFNF) {}
	USART1->TDR = message_byte;
}

/*
	@brief Enable SPI1 transmission with respect to given SPI speed

	SPI speed is set by dividing bus frequency by 2^(SPI_CR1_BR) so in general, it is impossible to set exact SPI speed and the closest speed smaller then input is used.

	@param[in] transmittion_speed_in_hz frequency witch will be used as a reference for set up (rounding down)

	@documentation
		> STM32G0x1 reference manual chapter 5 (RCC) - all information about peripheral locations and enabling;
		> STM32G0x1 reference manual chapter 34 (SPI/I2S) - SPI setup information.
 */
void basic_spi1_setup(uint32_t transmittion_speed_in_hz)
{
	// 10 Mhz is standard high SPI speed, so in general, we should not use higher speeds. If input speed is greater than 10Mhz set it to 5Mhz and write input mistake to log
	if(transmittion_speed_in_hz > 10000000)
	{
		add_to_mistakes_log(WRONG_SPI1_FREQUENCY_INPUT);
		transmittion_speed_in_hz = 5000000;
	}

	// Enable SPI clocking
	RCC->APBENR2 |= RCC_APBENR2_SPI1EN;

	// SPI SPI_CR1_BR value determination
	uint32_t baud_rate_devider = 2;
	uint32_t baud_rate = 0;

	for(int i = 0; i < 8; ++i)
	{
		if(SYSCLK_FREQUENCY/baud_rate_devider < transmittion_speed_in_hz)
		{
			break;
		}
		++baud_rate;
		baud_rate_devider *= 2;
	}

	// SPI setup
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (baud_rate << SPI_CR1_BR_Pos) | SPI_CR1_MSTR; // Equal to SPI1->CR1 |= 0x0314;
	SPI1->CR2 |= SPI_CR2_FRXTH;
	SPI1->CR1 |= SPI_CR1_SPE;
}

/*
	@brief Transmit and receive single byte with SPI 1

	@param[in] byte_to_be_sent Byte that will be sent by the SPI.

	@return	Byte returned by SPI

	@documentation
		> STM32G0x1 reference manual chapter 34 (SPI/I2S) - SPI information.
 */
uint8_t spi1_write_single_byte(const uint8_t byte_to_be_sent)
{
	uint32_t safety_delay_counter = 0;

	// Wait until transmit buffer is empty
	while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
	{
		++safety_delay_counter;
		if ( safety_delay_counter > DUMMY_DELAY_VALUE )
		{
			add_to_mistakes_log(SPI1_TRANSMISSION_FAIL);
			return 0;
		}
	}

	// Write single byte into the Data Register with single byte access
	*((volatile uint8_t *)&SPI1->DR) = byte_to_be_sent;

	// Wait until answer will appear in RX buffer
	while(((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE)){}
//	while(((SPI->SR & 0x81) == 0x80)){}

	// Return value from RX buffer
	return SPI1->DR;
}



//	@brief Same deal as with basic_spi1_setup function
void basic_spi2_setup(uint32_t transmittion_speed_in_hz)
{
	// 10 Mhz is standard high SPI speed, so in general, we should not use higher speeds. If input speed is greater than 10Mhz set it to 5Mhz and write input mistake to log
	if(transmittion_speed_in_hz > 10000000)
	{
		add_to_mistakes_log(WRONG_SPI2_FREQUENCY_INPUT);
		transmittion_speed_in_hz = 5000000;
	}

	// Enable SPI clocking
	RCC->APBENR1 |= RCC_APBENR1_SPI2EN;

	// SPI SPI_CR1_BR value determination
	uint32_t baud_rate_devider = 2;
	uint32_t baud_rate = 0;

	for(int i = 0; i < 8; ++i)
	{
		if(SYSCLK_FREQUENCY/baud_rate_devider < transmittion_speed_in_hz)
		{
			break;
		}
		++baud_rate;
		baud_rate_devider *= 2;
	}

	// SPI setup
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (baud_rate << SPI_CR1_BR_Pos) | SPI_CR1_MSTR; // Equal to SPI2->CR1 |= 0x0314;
	SPI2->CR2 |= SPI_CR2_FRXTH;
	SPI2->CR1 |= SPI_CR1_SPE;
}

//	@brief Same deal as with spi1_write_single_byte function
uint8_t spi2_write_single_byte(const uint8_t byte_to_be_sent)
{
	uint32_t safety_delay_counter = 0;

	// Wait until transmit buffer is empty
	while ( (SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE )
	{
		++safety_delay_counter;
		if ( safety_delay_counter > DUMMY_DELAY_VALUE )
		{
			add_to_mistakes_log(SPI2_TRANSMISSION_FAIL);
			return 0;
		}
	}

	// Write single byte into the Data Register with single byte access
	*((volatile uint8_t *)&SPI2->DR) = byte_to_be_sent;

	safety_delay_counter = 0;

	// Wait until answer will appear in RX buffer
	while ( ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE) ){}

	// Return value from RX buffer
	return SPI2->DR;
}

/*
	@brief Sets up all interfaces with default speed values
 */
void intrfaces_setup(void)
{

	basic_spi1_setup(5000000);

	basic_spi2_setup(5000000);

	basic_uart1_setup(19200);

}



void full_device_setup(uint32_t should_inclued_interfaces)
{

	system_clock_setup();

	gpio_setup();

	timers_setup();

	if(should_inclued_interfaces == yes)
	{
		intrfaces_setup();
	}

	return;
}




//void TIM15_IRQHandler()
//{
//
//}
//
//void TIM16_IRQHandler()
//{
//
//}
//
//void TIM17_IRQHandler()
//{
//
//}

void gpioc6_high(void)
{
	GPIOC->BSRR |= GPIO_BSRR_BS6;
}

void gpioc6_low(void)
{
	GPIOC->BSRR |= GPIO_BSRR_BR6;
}

void gpiob12_high(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BS12;
}

void gpiob12_low(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BR12;
}

void gpiob1_high(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BS1;
}

void gpiob1_low(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BR1;
}

void gpiob0_high(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BS0;
}

void gpiob0_low(void)
{
	GPIOB->BSRR |= GPIO_BSRR_BR0;
}

uint32_t set_motor1_pwm(const int32_t required_duty_cycle_coefficient)
{
	uint32_t max_duty_cycle = PWM_PRECISION;

	if (required_duty_cycle_coefficient < 0)
	{
		if(required_duty_cycle_coefficient < - max_duty_cycle) 	// PWM task negative but higher than maximum -> set maximum PWM in reverse direction
		{
			TIM1->CCR3 = max_duty_cycle;
			TIM1->CCR4 = 0;

			return M1_PWM_TASK_LOWER_THAN_MINIMUM;
		}
		else {						// PWM task is negative and less than maximum -> set task PWM in reverse direction
			TIM1->CCR3 = max_duty_cycle;
			TIM1->CCR4 = max_duty_cycle + required_duty_cycle_coefficient;
		}
	}
	else
	{ /* required_duty_cycle_coefficient >= 0 */
		if(required_duty_cycle_coefficient > max_duty_cycle)	// PWM task is positive but higher than maximum -> set maximum PWM in forward direction
		{
			TIM1->CCR4 = max_duty_cycle;
			TIM1->CCR3 = 0;

			return M1_PWM_TASK_HIGHER_THAN_MAXIMUM;
		}
		else						// PWM task is positive and less than maximum -> set maximum PWM in forward direction
		{
			TIM1->CCR4 = max_duty_cycle;
			TIM1->CCR3 = max_duty_cycle - required_duty_cycle_coefficient;
		}
	} /* required_duty_cycle_coefficient >= 0 */

	return 0;
}

uint32_t set_motor2_pwm(const int32_t required_duty_cycle_coefficient)
{
	uint32_t max_duty_cycle = PWM_PRECISION;

	if (required_duty_cycle_coefficient < 0)
	{
		if(required_duty_cycle_coefficient < - max_duty_cycle) 	// PWM task negative but higher than maximum -> set maximum PWM in reverse direction
		{
			TIM1->CCR2 = max_duty_cycle;
			TIM1->CCR1 = 0;

			return M2_PWM_TASK_LOWER_THAN_MINIMUM;
		}
		else {						// PWM task is negative and less than maximum -> set task PWM in reverse direction
			TIM1->CCR2 = max_duty_cycle;
			TIM1->CCR1 = max_duty_cycle + required_duty_cycle_coefficient;
		}
	}
	else
	{ /* required_duty_cycle_coefficient >= 0 */
		if(required_duty_cycle_coefficient > max_duty_cycle)	// PWM task is positive but higher than maximum -> set maximum PWM in forward direction
		{
			TIM1->CCR1 = max_duty_cycle;
			TIM1->CCR2 = 0;

			return M2_PWM_TASK_HIGHER_THAN_MAXIMUM;
		}
		else						// PWM task is positive and less than maximum -> set maximum PWM in forward direction
		{
			TIM1->CCR1 = max_duty_cycle;
			TIM1->CCR2 = max_duty_cycle - required_duty_cycle_coefficient;
		}
	} /* required_duty_cycle_coefficient >= 0 */

	return 0;
}


int16_t get_motor1_encoder_value(void)
{
	return TIM2->CNT;
}

int16_t get_motor2_encoder_value(void)
{
	return TIM3->CNT;
}


void TIM16_IRQHandler()
{
	TIM16->SR &= ~TIM_SR_UIF;
	delay_is_finished = yes;
}



void delay_in_milliseconds(const uint16_t time_in_millisecond){
//	for(uint32_t iterator = 0; iterator < time_in_millisecond; ++iterator);
	TIM16->ARR = time_in_millisecond-1;
	TIM16->CR1 |= TIM_CR1_CEN;
	delay_is_finished = 0;
	while(delay_is_finished == no){}

}

