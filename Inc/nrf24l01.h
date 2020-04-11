#ifndef NRF24L01_PLUS_H_
#define NRF24L01_PLUS_H_

#include "implementation.h"
#include "nrf24l01_registers.h"
#include "nrf24l01p_mistakes.h"

/*
	General library overwiew


	What is not implemented and won't be implemented
	- received power detection (RPD) reading and handling;
	- only addresses with a length of 5 bytes are allowed. Addresses with 3 and 4 bytes are not allowed;

 */

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
 */
typedef struct
{
	/*
		@brief Chip select high. Used in SPI communication to indicate the end of the transition.
			Should set CSN pin of this particular NRF24L01+ instance into logic high.
	 */
	void (*csn_high)(void);

	/*
		@brief Chip select low. Used in SPI communication to indicate the start of the transition.
			Should set CSN pin of this particular NRF24L01+ instance into logic low.
	 */
	void (*csn_low)(void);

	/*
		@brief Chip enable high. Enables chip from standby-1 mode.
			Should set CE pin of this particular NRF24L01+ instance into logic high.
	 */
	void (*ce_high)(void);

	/*
		@brief Chip enable low. Disables chip from either RX or TX  mode into the standby-1.
			Should set CE pin of this particular NRF24L01+ instance into logic low.
	 */
	void (*ce_low)(void);

	/*
		@brief Sends single byte through specified SPI. Returns recieved with this byte answer.
			Should send input byte by the NRF24L01+ instance-specific SPI in single-byte mode. Should return byte received by SPI during transmission.
	 */
	uint8_t (*spi_write_byte)(uint8_t byte_to_be_written);

	// Used as default payload size for channel 1 and 0.
	uint32_t payload_size_in_bytes;

	uint32_t frequency_channel;
	nrf24_pa_contol power_output;
	nrf24_data_rate data_rate;

	// Flag that shows that device was already initialized. Needed for error checking optimiztion.
	uint32_t device_was_initialized;
} nrf24l01p;


/*
	@brief Sets up nrf24l01+ in such a way that it doen't use anything additional to what is needed to transmit and receive data in
 */
uint32_t nrf24_basic_init(nrf24l01p * nrf24_instance);

/*
	@brief Read RF_CH register (which is never equals 0). If response data equals 0 - device is not connected, returns mistake code.
 */
uint32_t nrf24_check_if_alive(nrf24l01p * nrf24_instance);

/*
	@brief Checks if all function pointers were initialized. If not returns mistake code.
 */
uint32_t nrf24_check_declarations(nrf24l01p * nrf24_instance);

/*
	@brief
 */
uint32_t nrf24_send_message(nrf24l01p * nrf24_instance, void *payload, uint32_t payload_size, int32_t send_ac);

/*
	@brief change device mode to TX, but not pulls CE logic high for power efficiency - if powered up stays in standby-1.
 */
uint32_t nrf24_tx_mode(nrf24l01p * nrf24_instance);

/*
	@brief Changes device settings to RX, powers up the device, sets CE in logic high - start device.
 */
uint32_t nrf24_rx_mode(nrf24l01p * nrf24_instance);

/*
	@brief Sets new values for retransmit delay and count of retransmissions for particular nrf24l01+ device.
 */
uint32_t nrf24_update_retransmission_params(nrf24l01p * nrf24_instance, nrf24_auto_retransmit_delay new_retransmit_delay, uint32_t new_retransmit_count);

/*
	@brief Sets new TX and RX pipe 0 addresses.
 */
uint32_t nrf24_set_tx_address(nrf24l01p * nrf24_instance, const uint8_t new_tx_address[5]);

/*
	@brief Enables interrupts with 1 in related input parameters, disables interrupts with 0. So to disable all interrupts call function with all 0 as inputs.
 */
uint32_t nrf24_enable_interrupts(nrf24l01p * nrf24_instance, uint32_t enable_rx_dr, uint32_t enable_tx_ds, uint32_t enable_max_rt);

/*
	@brief Reads NRF24_STATUS register, clears all interrupts and returns only interrupt flags states.
 */
uint32_t nrf24_get_interrupts_status(nrf24l01p * nrf24_instance);

/*
	@brief Sets new TX address for pipe 1.
 */
uint32_t nrf24_enable_pipe1(nrf24l01p * nrf24_instance, uint8_t pipe_address[]);

uint32_t nrf24_enable_pipe2_4(nrf24l01p * nrf24_instance, uint32_t pipe_number, uint8_t pipe_address_last_byte);

uint8_t nrf24_is_new_data_availiable(nrf24l01p * nrf24_instance);

uint32_t nrf24_read_message(nrf24l01p * nrf24_instance, void * payload_storage, uint32_t pyaload_size);


#endif /* NRF24L01_PLUS_H_ */
