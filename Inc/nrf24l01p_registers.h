#ifndef NRF24L01_REGISTERS_H_
#define NRF24L01_REGISTERS_H_

// Information about registers contains in nrf24l01+ datasheet
// This file is enhanced with only bits and masks useful for library implementation. So not all bits are masked here.

//*** NRF24L0+ registers ***//

// Register
#define NRF24_CONFIG  			0x00
	// Bits
	#define NRF24_MASK_RX_DR			0x40	// Data received interrupt: 1 - disable, 0 - enable
	#define NRF24_MASK_TX_DS			0x20	// Data transmitted interrupt: 1 - disable, 0 - enable
	#define NRF24_MASK_MAX_RT			0x10	// Maximum amount of retransmissions reached interrupt: 1 - disable, 0 - enable
	#define NRF24_EN_CRC				0x08	// 1 - enable CRC, 0 - disable CRC
	#define NRF24_CRCO					0x04	// 1 - 2 bytes CRC, 0 - 1 byte CRC
	#define NRF24_PWR_UP				0x02	// 1 - power up, 0 - power down
	#define NRF24_PRIM_RX				0x01	// 1 - receiver (RX), 0 - transmitter (TX)
	// Masks
	#define NRF24_INTERRUPTS_MASK		0x70	// To mask all interrupts at the same time

// Registers
#define NRF24_EN_AA       		0x01			// Not used in the library
#define NRF24_EN_RXADDR   		0x02
	// can implement pipe enable with shift 1<<pipe number
//	#define NRF24_ERX_P5			0x20		// Data pipe 5: 1 - enable, 0 - disable (disabled by default)
//	#define NRF24_ERX_P4			0x10		// Data pipe 4: 1 - enable, 0 - disable (disabled by default)
//	#define NRF24_ERX_P3			0x08		// Data pipe 3: 1 - enable, 0 - disable (disabled by default)
//	#define NRF24_ERX_P2			0x04		// Data pipe 2: 1 - enable, 0 - disable (disabled by default)
//	#define NRF24_ERX_P1			0x02		// Data pipe 1: 1 - enable, 0 - disable (enabled by default)
//	#define NRF24_ERX_P0			0x01		// Data pipe 0: 1 - enable, 0 - disable (enabled by default)

// Registers
#define NRF24_SETUP_AW    		0x03			// Not used in the library
#define NRF24_SETUP_RETR  		0x04
	// Fields positions
	#define NRF24_ARD_pos			(4U)		// Shift for auto retransmit delay setup field
	#define NRF24_ARC_pos			(0U)		// SHift for auto retransmit count setup field

// Registers
#define NRF24_RF_CH       		0x05
#define NRF24_RF_SETUP    		0x06
#define NRF24_STATUS  			0x07
#define NRF24_OBSERVE_TX  		0x08
#define NRF24_CD          		0x09

#define NRF24_RX_ADDR_P0 		0x0A
#define NRF24_RX_ADDR_P1  		0x0B
#define NRF24_RX_ADDR_P2  		0x0C
#define NRF24_RX_ADDR_P3 		0x0D
#define NRF24_RX_ADDR_P4  		0x0E
#define NRF24_RX_ADDR_P5  		0x0F
#define NRF24_TX_ADDR     		0x10

#define NRF24_RX_PW_P0    		0x11
#define NRF24_RX_PW_P1    		0x12
#define NRF24_RX_PW_P2    		0x13
#define NRF24_RX_PW_P3    		0x14
#define NRF24_RX_PW_P4    		0x15
#define NRF24_RX_PW_P5    		0x16
#define NRF24_FIFO_STATUS 		0x17

#define NRF24_DYNPD	    		0x1C
#define NRF24_FEATURE	    	0x1D

//*** NRF24L01+ commands ***//
#define NRF24_R_REGISTER    	0x00
#define NRF24_W_REGISTER    	0x20
#define NRF24_REGISTER_MASK 	0x1F

#define NRF24_ACTIVATE     		0x50
#define NRF24_R_RX_PL_WID  		0x60
#define NRF24_R_RX_PAYLOAD  	0x61
#define NRF24_W_TX_PAYLOAD  	0xA0
#define NRF24_W_ACK_PAYLOAD 	0xA8

#define NRF24_FLUSH_TX      	0xE1
#define NRF24_FLUSH_RX      	0xE2
#define NRF24_REUSE_TX_PL   	0xE3
#define NRF24_NOP           	0xFF

#define W_TX_PAYLOAD_NO_ACK		0xB0

#endif /* NRF24L01_REGISTERS_H_ */
