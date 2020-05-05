#ifndef IMPLEMENTATION_H_
#define IMPLEMENTATION_H_

#include "stm32g071xx.h"
#include "mistakes_codes.h"

//*************************************//
//****** User-adjustable defines ******//
//*************************************//

/*
	@brief 	Contains desired SYSCLK and HCLK values in Hz.

	Used in all time related things: SYSCLK setup, general purpose timers setup, PWM setup, time calculation and so forth.

	@note 	Only frequencies multiple to 2Mhz are allowed and PLLN is has a minimum value of 8.
	 	 	Allowed frequencies are: 16000000, 18000000 ... 62000000, 64000000 Hz.
 */
#define SYSCLK_FREQUENCY 				48000000 // Hz = 24 Mhz

/*
	@brief 	PWM frequency for motor control in Hz and related PWM precision

	@Calculations	PWM precision can be calculated as:

					 SYSCLK_FREQUENCY
	PWM_precision = ------------------
 	 	 	 	 	  PWM_FREQUENCY
 */
#define PWM_FREQUENCY 					20000	// Hz = 20 Khz
#define PWM_PRECISION					(SYSCLK_FREQUENCY / PWM_FREQUENCY - 1)	// -1 is needed for proper timers setup. Equation shouldn't be changed

/*
	@brief	Amount of times system clock interrupt occurs in one second

	Determines control system outer loop frequency
 */
#define SYSTICK_FREQUENCY				40		// Hz

/*
	@brief Mistakes log is used to collect mistakes codes during runtime

	Log can be sent by any available interface during runtime to debug the system.

	TODO: @idea	It will be good to have EEPROM on every device so we can store mistakes log there

	TODO: It will be good to implement some type of black box: way to permanently store log with system parameters for the last couple of hours at least: EEPROM won't suffice
 */
#define MISTAKES_LOG_SIZE				50

// *** Libraries based defines *** //
#define NRF24L01P_MISTAKES_OFFSET 		100
#define ICM_20600_MISTAKES_OFFSET		200

//****** End of User-adjustable defines ******//


//****************************//
//****** User functions ******//
//****************************//
// @brief Can be called in the code by the programmer while developing applications.


/*
	@brief Makes log entry with given mistake code at current time
 */
uint32_t add_to_mistakes_log(uint32_t mistake_code);

/*
	@brief Sets up all desired device peripherals
 */
void full_device_setup(uint32_t should_inclued_interfaces, uint32_t should_setup_interrupts);

/*
	@brief	Enables UART 1 with a given baud rate with TX and RX enable and default setting in everything else
 */
void basic_uart1_setup(const uint32_t transmission_speed_in_bauds);

/*
	 @brief	Sends given byte when TX buffer is empty
 */
void uart1_send_byte(const uint8_t message_byte);

/*
	@brief Enable SPI1 transmission with respect to given SPI speed
 */
void basic_spi1_setup(uint32_t transmittion_speed_in_hz);

/*
	@brief Transmit and receive single byte with SPI 1
 */
uint8_t spi1_write_single_byte(const uint8_t byte_to_be_sent);

/*
	@brief Enable SPI2 transmission with respect to given SPI speed
 */
void basic_spi2_setup(uint32_t transmittion_speed_in_hz);

/*
	@brief Transmit and receive single byte with SPI 2
 */
uint8_t spi2_write_single_byte(const uint8_t byte_to_be_sent);


//********************************//
//****** Non-User functions ******//
//********************************//
/*
	@brief	Called only by the system when needed. Should not be used in the application code
 */

/*
	@brief	Sets up SYSCLK to SYSCLK_FREQUENCY with taking into account problems with different sources
 */
uint32_t system_clock_setup(void);

/*
	@brief	Sets up PLL with respect to input source and SYSCLK_FREQUENCY
 */
uint32_t pll_setup(uint32_t is_HSE_clock_source);

/*
	@brief Sets up GPIO for all needed on device functions
 */
void gpio_setup(void);

/*
	@brief Sets up all used on the board timers and enables desired interrupts
 */
void timers_setup(void);

/*
	@brief Sets all interfaces with default transmission rates
 */
void intrfaces_setup(void);

/*
	@brief Set up all external interrupts. ICM-20600, and NRF24L01 interrupt
 */
void interrupts_setup(void);

/*
	@brief TIM14 overflow interrupt handler - counts minutes from enable of mistakes log.
 */
void TIM14_IRQHandler();

/*
	@brief Makes program stop for given duration
 */
void delay_in_milliseconds(const uint16_t time_in_millisecond);

/*
	@brief Resets program blocking value after overflow
 */
void TIM16_IRQHandler();

//****** DRV8848 ******//
/*
	GPIOC6 - nSleep pin of motor driver
 */
void gpioc6_low(void);
void gpioc6_high(void);

/*
	@brief Sets PWM duty cycle for motor 1 or motor 2 PWM channels
 */
uint32_t set_motor1_pwm(const int32_t required_duty_cycle_coefficient);
uint32_t set_motor2_pwm(const int32_t required_duty_cycle_coefficient);

/*
	@brief Returns motor-related encoder counter value. TIM2 for motor1 and TIM3 for motor 2
 */
int16_t get_motor1_encoder_value(void);
int16_t get_motor2_encoder_value(void);

//****** NRF24L01+ ******//
#define NRF24_CSN_HIGH 			GPIOB->BSRR	|=	GPIO_BSRR_BS1;
#define NRF24_CSN_LOW  			GPIOB->BSRR |=	GPIO_BSRR_BR1;
#define NRF24_CE_HIGH	 		GPIOB->BSRR	|=	GPIO_BSRR_BS0;
#define NRF24_CE_LOW 	 		GPIOB->BSRR	|=	GPIO_BSRR_BR0;
#define nrf24_spi_write			spi1_write_single_byte

void gpiob1_high(void);
void gpiob1_low(void);
void gpiob0_high(void);
void gpiob0_low(void);

//****** ICM-20600 ******//
/*
	GPIOB12 - chip enable
	For icm-20600 library ICM_THROUGH_SPI or ICM_THROUGH_I2C should be defined
 */
void gpiob12_high(void);
void gpiob12_low(void);

#define ICM_THROUGH_SPI
//#define ICM_THROUGH_I2C


//****************************************//
//****** Non user-adjustable defines ******//
//****************************************//
/*
	@brief All defines that should not be changed contains here. Also all checks for #define mistakes happen here redefines happen here so they will be at top of any listing and won't distract programmers.
 */

/* Check for correct system clock define  */
#if (SYSCLK_FREQUENCY > 64000000)
	#error Clock speed define is higher than maximum value of 64 Mhz

#elif (SYSCLK_FREQUENCY < 16000000)
	#error Clock speed define is less then minimum value of 16 Mhz

#endif


// @brief Mistakes log entry form
typedef struct{
	uint32_t mistake_code;
	uint32_t mistake_time_in_milliseconds;
	uint32_t mistake_time_in_seconds;
} mistake;

/*** Global variables declaration ***/
/*
	@brief Declares global variables in all files that include this one.

	 If there is VAR_DECLS define in file - declares and initialize variables (should be only one such file).
	 If there is no VAR_DECLS define in file - declares all variables as extern and doesn't initialize them.
 */
#ifndef VAR_DEFS
#define VAR_DEFS 1		// Make sure this file is included only once for each .c file

#ifndef VAR_DECLS		// There should be one file with defined VAR_DECLS. So there will be only one file with variables initialization.
# define _DECL extern	// If the .c file doesn't contains VAR_DECLS definition, all global variables will be declared with extern prefix and without initialization
# define _INIT(x)
#else
# define _DECL			// If there is VAR_DECLS definition in .c file, all global variables will be declared without extern prefix
# define _INIT(x)  = x	// and will be initialized with predefined values x
#endif /* VAR_DECLS */

/* Global variables should be declared as follows: "_DECL [standard variable declaration] _INIT(x);", where x is initialization value.
 * If there is no need for variables initialization, declaration can look as: "_DECL [standard variable declaration];" */

///*** Mistakes log variables ***//
//	@brief This log is used to accumulate mistakes, that occurred during runtime, so it will be possible to debug system with debugger.
_DECL mistake mistakes_log[MISTAKES_LOG_SIZE];
_DECL uint32_t mistakes_log_pointer _INIT(0);
_DECL uint32_t critical_mistake_detected _INIT(0);		/* will be 1 - if there were a critical mistake. System should be stopped. All performance critical objects should be returned to safe state and be deactivated */
_DECL uint32_t time_from_log_enable_in_seconds _INIT(0);

_DECL uint32_t delay_is_finished _INIT(0);

#endif /* VAR_DEFS */
/******************************************/

enum question_answers
{
	no = 0,
	yes = 1
};

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

/*** Completely random value to determine the waiting-state length ***/
#define DUMMY_DELAY_VALUE 10000

#endif /* IMPLEMENTATION_H_ */
