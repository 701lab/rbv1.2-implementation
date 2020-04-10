#ifndef NRF24L01_PLUS_H_
#define NRF24L01_PLUS_H_

#include "implementation.h"
#include "nrf24l01_registers.h"

//** Place for development of new library which will fully replace the old one. Work in progress so far. **//

// рекомендуемое поведение устрйоства: выход в активный режим только тогда, когда необходимо отправлять данные и возвращаться в standby после для ожидания новой партии (ниже энергопотребление) Надо посмотреть,
// 	работает ли ныне существующая функция по такой логике и если нет, сделать два ваохможных варианта работы: динамический (экономит энергию постоянно включаясь и выключаясь) и статический (постоянно включен)

/*
	General library overwiew


	What is not implemented and won't be implemented
	- received power detection (RPD) reading and handling;
	- only addresses with a length of 5 bytes are allowed. Addresses with 3 and 4 bytes are not allowed;

 */


/*
	@brief Main structure which fully describes nrf24l01+ instance.
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

	uint32_t frequency_channel;
	nrf24_pa_contol power_output;
	nrf24_data_rate data_rate;
	nrf24_auto_retransmit_delay auto_retransmit_delay;
} nrf24l01p;

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
	nrf24_wait_250_us = (0U),
	nrf24_wait_500_us = (1U),
	nrf24_wait_750_us = (2U),
	nrf24_wait_1000_us = (3U),
	nrf24_wait_1250_us = (4U),
	nrf24_wait_1500_us = (5U),
	nrf24_wait_1750_us = (6U),
	nrf24_wait_2000_us = (7U),
	nrf24_wait_2250_us = (8U),
	nrf24_wait_2500_us = (9U),
	nrf24_wait_2750_us = (10U),
	nrf24_wait_3000_us = (11U),
	nrf24_wait_3250_us = (12U),
	nrf24_wait_3500_us = (13U),
	nrf24_wait_3750_us = (14U),
	nrf24_wait_4000_us = (15U)
} nrf24_auto_retransmit_delay;


//*** Basic building blocks of library ***//





// @brief Can be called almost immediately afters board startup, but nrf24 need 100ms to start up into power-down mode, so check should be produced. In power-down all registers are available through SPI.
uint32_t nrf24_power_up(nrf24l01p * nrf24_instance);

// @brief Goes to power down mode directly from any device state. Not recommended but possible behavior.
uint32_t nrf24_fast_power_down(nrf24l01p * nrf24_instance);

// @brief Going to power down through standby-1 state even though it is possible to do from any other state directly. Recommended behavior.
uint32_t nrf24_safe_power_down(nrf24l01p * nrf24_instance);

// @brief Reads *** register which always contains some non 0 data to check if device is connected.
uint32_t nrf24_check_if_alive(void);

// @brief Enables interrupts with 1 in related input parameters, disables interrupts with 0. So to disable all interrupts call function with all 0 as inputs.
void nrf24_enable_interrupts(uint32_t enable_rx_dr, uint32_t enable_tx_ds, uint32_t enable_max_rt);

//*********************************************************************************************************//



//hardware based defines: 
/*
	In this section happaens all the define magic, so that NRF24 can work properly. Also you need not to forgot to setup needed 
	periperal. You can do this just in your code and then call initNRF_SPI() function. Or you can jange the initSystemSPI() function and call it first.
*/


//**********************//

// RF_SETUP values end result = RF_POWER|DATA_RATE
#define RF_POWER 0x04  					// 0x00 = -18dBm // 0x02 = -12dBm // 0x04 = -6dBm // 0x06 = 0dBm//
#define DATA_RATE 0x00 					// 0x00 = 1Mbps // 0x08 = 2Mbps // 0x20 = 250kbps //

// RF_CH values
#define RF_CH_VAL 45 						// frequency channel nRF24L01+ operates on [0,127]

// EN_RXADDR values // reset state 0x03
// enables data pipes, bit for pipe [0,5]
#define EN_RXADDR_VAL = 0x03 		// use of pipe 0 and 1

// NRF_CONFIG values
// Enable CRC | CRC encoding scheme (0 - 1 byte, 1 - 2 bytes) | PWR_UP!! | PRIM_RX
#define CONFIG_VAL 0x0E 				// 0x0E - typical transmitter setup // 0x0F - typical receiver setup (PRIM_RX = 1) //

// NRF_STATUS values
#define NRF_STATUS_RESET 0x70 	// reset all interupts flags

// Transmiter and reciever payload lenght
#define PAYLOAD_LENGTH 12 			// number of bytes to be transmited and recieved. Roght now all pipes use the same value
																// can be updated. so each pype will have it own pyaload length. Values [1, 32]

void nrf24_basic_init(void);    		// consist NRF registers setup



void setTxAddress(uint8_t transiverAddress[5]);

void tx_mode(void); // goes to tx mode
void rx_mode(void); // goes to rx mode

void enablePype(uint8_t pypeAddress[5],uint32_t pipeNum);

void nrf24_send_data(const void *data, uint32_t PayloadSize, int no_ack);

uint8_t dataAvailiable(void);//-
void readData(void* data, uint32_t PayloadSize);

#endif /* NRF24L01_PLUS_H_ */
