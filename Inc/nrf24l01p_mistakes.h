#ifndef NRF24L01P_MISTAKES_H_
#define NRF24L01P_MISTAKES_H_

/*
	@file nrf24l01p_mistakes.h
	@brief Contains defines with all mistakes codes that nrf24l01p library uses
 */

// Can be changed by the user to get mistakes in any desired range
#define NRF24L01P_MISTAAKES_OFFSET					(0U)

#define NRF24_WRONG_CHANNEL_FREQUENCY				(1U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_WRONG_PAYLOAD_SIZE					(2U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_DEVICE_IS_NOT_CONNECTED				(3U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_CE_HIGH_FUNCTION_IS_MISSING			(4U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_CE_LOW_FUNCTION_IS_MISSING			(5U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_CSN_HIGH_FUNCTION_IS_MISSING			(6U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_CSN_LOW_FUNCTION_IS_MISSING			(7U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_SPI_WRITE_FUNCTION_IS_MISSING			(8U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_INSTANCE_WAS_NOT_INITIALIZED			(9U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_WRONG_RETRANSMIT_COUNT				(10U) + NRF24L01P_MISTAAKES_OFFSET
#define NRF24_WRONG_MESSAGE_SIZE					(11U) + NRF24L01P_MISTAAKES_OFFSET


#endif /* NRF24L01P_MISTAKES_H_ */
