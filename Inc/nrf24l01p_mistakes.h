#ifndef NRF24L01P_MISTAKES_H_
#define NRF24L01P_MISTAKES_H_

/*
	@file nrf24l01p_mistakes.h
	@brief Contains defines with all mistakes codes that nrf24l01p library uses
 */

// Can be changed by the user to get mistakes in any desired range
#define NRF24L01P_MISTAAKES_OFFSET			0

#define NRF24_WRONG_CHANNEL_FREQUENCY		1 + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_WRONG_PAYLOAD_SIZE			2 + NRF24L01P_MISTAAKES_OFFSET
//#define NRF24_WRONG_PAYLOAD_SIZE			2 + NRF24L01P_MISTAAKES_OFFSET



#endif /* NRF24L01P_MISTAKES_H_ */
