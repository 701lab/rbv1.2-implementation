#ifndef NRF24L01_PLUS_H_
#define NRF24L01_PLUS_H_

#include "implementation.h"
#include "nrf24l01_registers.h"

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

uint32_t nrf24_check_if_alive(void);


void setTxAddress(uint8_t transiverAddress[5]);

void tx_mode(void); // goes to tx mode
void rx_mode(void); // goes to rx mode

void enablePype(uint8_t pypeAddress[5],uint32_t pipeNum);

void nrf24_send_data(const void *data, uint32_t PayloadSize, int no_ack);

uint8_t dataAvailiable(void);//-
void readData(void* data, uint32_t PayloadSize);

#endif /* NRF24L01_PLUS_H_ */
