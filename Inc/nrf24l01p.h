#ifndef NRF24L01P_H_
#define NRF24L01P_H_

// File that your project should provide for the library to work properly.
#include "implementation.h"

//Files related to the library.
#include "nrf24l01p_registers.h"
#include "nrf24l01p_mistakes.h"

//****** typedefs required for the library ******//

// @brief New data type for safe nrf24l01+ data rate setup
typedef enum nrf24_data_rate
{
	nrf24_250_kbps = 0x20,
	nrf24_1_mbps = 0x00,
	nrf24_2_mbps = 0x08
} nrf24_data_rate;

// @brief New data type for safe nrf24l01+ power output setup
typedef enum nrf24_pa_contol
{
	nrf24_pa_min = 0x06,	// 0 dbm RF output power, 11.3 mA dc current consumption
	nrf24_pa_low = 0x04,	// -6 dbm RF output power, 9.0 mA dc current consumption
	nrf24_pa_high = 0x02,	// -12 dbm RF output power, 7.5 mA dc current consumption
	nrf24_pa_max = 0x00		// -18 dbm RF output power, 7.0 mA dc current consumption
} nrf24_pa_contol;

// @brief New data type for nrf24 safe nrf24l01+ auto retransmit delay setup
typedef enum nrf24_auto_retransmit_delay
{
	nrf24_wait_250_us = 0x00,
	nrf24_wait_500_us = 0x10,
	nrf24_wait_750_us = 0x20,
	nrf24_wait_1000_us = 0x30,
	nrf24_wait_1250_us = 0x40,
	nrf24_wait_1500_us = 0x50,
	nrf24_wait_1750_us = 0x60,
	nrf24_wait_2000_us = 0x70,
	nrf24_wait_2250_us = 0x80,
	nrf24_wait_2500_us = 0x90,
	nrf24_wait_2750_us = 0xA0,
	nrf24_wait_3000_us = 0xB0,
	nrf24_wait_3250_us = 0xC0,
	nrf24_wait_3500_us = 0xD0,
	nrf24_wait_3750_us = 0xE0,
	nrf24_wait_4000_us = 0xF0
} nrf24_auto_retransmit_delay;

/*
 	 @brief Structure which contains all parameters that should be the same on both transmitter and receiver for easier and more consistent setups.
 	 	This structure makes it possible to use multiple devices at the same time. Each device should have its own object.
		It is possible to use multiple devices with single spi_write_byte_function if devices have at least different csn pins (CE pins can but not required to be different).
 */
typedef struct
{

	//	@brief Chip select high. Used in SPI communication to indicate the end of the transition.
	//		Should set CSN pin of this particular NRF24L01+ instance into logic high.
	void (*csn_high)(void);

	//	@brief Chip select low. Used in SPI communication to indicate the start of the transition.
	//		Should set CSN pin of this particular NRF24L01+ instance into logic low.
	void (*csn_low)(void);

	//	@brief Chip enable high. Enables chip from standby-1 mode.
	//		Should set CE pin of this particular NRF24L01+ instance into logic high.
	void (*ce_high)(void);

	//	@brief Chip enable low. Disables chip from either RX or TX  mode into the standby-1.
	//		Should set CE pin of this particular NRF24L01+ instance into logic low.
	void (*ce_low)(void);

	//	@brief Sends single byte through specified SPI. Returns recieved with this byte answer.
	//		Should send input byte by the NRF24L01+ instance-specific SPI in single-byte mode. Should return byte received by SPI during transmission.
	uint8_t (*spi_write_byte)(uint8_t byte_to_be_written);

	// Payload size for all pipes. Should be between 0 and 32.
	uint32_t payload_size_in_bytes;

	// Should be in range of 1 to 124.
	uint32_t frequency_channel;

	// Should be set up with nrf24_pa_contol type value.
	nrf24_pa_contol power_output;

	// Should be set up with nrf24_data_rate type value.
	nrf24_data_rate data_rate;

	// Flag that shows that device was already initialized. Needed for internal library error checking optimization. Should be set to 0 at init.
	uint32_t device_was_initialized;

} nrf24l01p;

//****** Functions implemented in the library ******//
/*
	@name nrf24_basic_init
	@brief Sets up nrf24l01+ with parameters specified in the structure of particular instance.
		Doesn't power up device. Doesn't enable any mode. Should be called at least one time for every nrf24l01+ instance.
 */
uint32_t nrf24_basic_init(nrf24l01p * nrf24_instance);

/*
	@name nrf24_check_if_alive
	@brief Checks if particular devices responses (connected), if not returns mistake code.
 */
uint32_t nrf24_check_if_alive(nrf24l01p * nrf24_instance);

/*
	@name nrf24_check_declarations
	@brief Checks if all function pointers of particular nrf24l01+ instance struct were initialized. If not returns mistake code.
		Those mistakes should be handled as critical, so any function calling this one should immediately return mistake code.
 */
uint32_t nrf24_check_declarations(nrf24l01p * nrf24_instance);

/*
	@name nrf24_tx_mode
	@brief Changes device setup to TX, powers device up. Device is ready to transmit data.
 */
uint32_t nrf24_tx_mode(nrf24l01p * nrf24_instance);

/*
	@name nrf24_rx_mode
	@brief Changes device settings to RX, powers up the device, start device. Device is ready to receive data.
 */
uint32_t nrf24_rx_mode(nrf24l01p * nrf24_instance);

/*
	@name nrf24_set_tx_address
	@brief Sets new TX and RX pipe 0 addresses. Address should be 5 bytes long. Powers device down, So after this function either nrf24_rx_mode or nrf24_tx_mode should be called.
 */
uint32_t nrf24_set_tx_address(nrf24l01p * nrf24_instance, const uint8_t new_tx_address[5]);

/*
	@name nrf24_enable_pipe1
	@brief Sets new RX address for pipe 1. Address should be 5 bytes long. 4 first bytes are used as base for addresses of pipes 2 to 5.
		Powers device down, So after this function either nrf24_rx_mode or nrf24_tx_mode should be called.
 */
uint32_t nrf24_enable_pipe1(nrf24l01p * nrf24_instance, uint8_t pipe_address[]);

/*
	@name nrf24_enable_pipe2_4
	@brief Sets new RX address for pipe from 2 to 5 and enables corresponding pipe. Use base from pipe 4, so only one byte changes.
		Powers device down, So after this function either nrf24_rx_mode or nrf24_tx_mode should be called.
 */
uint32_t nrf24_enable_pipe2_5(nrf24l01p * nrf24_instance, uint32_t pipe_number, uint8_t pipe_address_last_byte);

/*
	@name nrf24_send_message
	@brief Sends message with either acknowledgment or not.
 */
uint32_t nrf24_send_message(nrf24l01p * nrf24_instance, void *payload, uint32_t payload_size, int32_t send_ac);

/*
	@name nrf24_is_new_data_availiable
	@brief Checks if new data is available. If so return the number of the pipe which received message.
 */
uint8_t nrf24_is_new_data_availiable(nrf24l01p * nrf24_instance);

/*
	@name nrf24_read_message
	@brief Saves last received message into the input array.
 */
uint32_t nrf24_read_message(nrf24l01p * nrf24_instance, void * payload_storage, uint32_t payload_size_in_bytes);

/*
	@name nrf24_enable_interrupts
	@brief Enables interrupts with 1 in related input parameters, disables interrupts with 0. So to disable all interrupts call function with all 0 as inputs.
 */
uint32_t nrf24_enable_interrupts(nrf24l01p * nrf24_instance, uint32_t enable_rx_dr, uint32_t enable_tx_ds, uint32_t enable_max_rt);

/*
	@name nrf24_get_interrupts_status
	@brief Returns interrupt flags in raw state. Clears interrupts flags.
 */
uint8_t nrf24_get_interrupts_status(nrf24l01p * nrf24_instance);

/*
	@name nrf24_update_retransmission_params
	@brief Sets new values for retransmit delay and count of retransmissions for particular nrf24l01+ device.
		If retransmit count = 0 no retransmissions are produced.
 */
uint32_t nrf24_update_retransmission_params(nrf24l01p * nrf24_instance, nrf24_auto_retransmit_delay new_retransmit_delay, uint32_t new_retransmit_count);

#endif /* NRF24L01P_H_ */
