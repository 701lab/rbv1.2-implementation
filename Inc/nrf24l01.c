#include "nrf24l01.h"


//***********NRF24l01+ main setup***********// 
// NRF is seted up as transmiter with address 0xAABBCCDD11
// 1 MBS, -6dBm, 32 payload length, 45 channel
//******************************************//

void nrf24_basic_init(void){

	NRF24_CSN_HIGH


	//Main nrf setup: bit 0 - RX-mod=1, TX-mod=0; bit 1 - Power up=1, Power down=0; bit 2 - 1 byte CRC=0, 2 byte CRC=1, bit 3 - Enable CRC
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_CONFIG);//0x20
	nrf24_spi_write(0x0E); // Transmiter mod
	NRF24_CSN_HIGH

	//Next to registers - really advanced setup.
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_FEATURE);		 //0x3D
	nrf24_spi_write(0x00);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_DYNPD);	 		 //0x3C
	nrf24_spi_write(0x00);
	NRF24_CSN_HIGH
	
	//Clear tx and rx flags to communicate 
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_STATUS); //0x27
	nrf24_spi_write(0x70);
	NRF24_CSN_HIGH

	//Setup of signal power and data rate
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_RF_SETUP);   //0x26
	nrf24_spi_write(RF_POWER | DATA_RATE);
	NRF24_CSN_HIGH

	//Setup channel from 0 to 127
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_RF_CH);			 //0x25
	nrf24_spi_write(RF_CH_VAL);
	NRF24_CSN_HIGH

	//Receive 0 address. Should be the same as TX address. Default state 0xAABBCCDD11
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_RX_ADDR_P0); //0x2A
	nrf24_spi_write(0x11);
	nrf24_spi_write(0xDD);
	nrf24_spi_write(0xCC);
	nrf24_spi_write(0xBB);
	nrf24_spi_write(0xAA);
	NRF24_CSN_HIGH

	//TX address. One of RX n(where n from 1 to 5) channels on the transiver should have the same address. Default state 0xAABBCCDD11
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_TX_ADDR);		 //0x30
	nrf24_spi_write(0x11);
	nrf24_spi_write(0xDD);
	nrf24_spi_write(0xCC);
	nrf24_spi_write(0xBB);
	nrf24_spi_write(0xAA);
	NRF24_CSN_HIGH

	//RX_0 payload length
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_RX_PW_P0);   //0x31
	nrf24_spi_write(PAYLOAD_LENGTH);
	NRF24_CSN_HIGH

	//Clear TX payload
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_TX);
	NRF24_CSN_HIGH
	
	//Clear RX payload
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_RX);
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
}


//***********Set TX address***********// 
// chainging RX_P0 and TX adresses
//************************************//

void setTxAddress(uint8_t transiverAddress[5])
{
	NRF24_CE_LOW // stop module before changing of setups

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_RX_ADDR_P0); //we need to change Rx address as well
	for(int i = 0; i < 5; ++i){
		nrf24_spi_write(transiverAddress[4-i]);
	}
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_TX_ADDR); //we need to change Rx address as well
	for(int i = 0; i < 5; ++i){
		nrf24_spi_write(transiverAddress[4-i]);
	}
	NRF24_CSN_HIGH

	NRF24_CE_HIGH// start module
}


//***********Enable pype 1***********// 
/*
	Important to remember, that all adresses for pipe1 - 5 should be ecual despite last byte. So here
	it is implemented in the way that we believe that person will not mistake in call of this function.
	If pipe = 1 just write it fully. if pipe != 1 write only last byte. I hope, that if pipe != 1 pipe one is already addressed
*/
//***********************************//

void enablePype(uint8_t pypeAddress[5], uint32_t pipeNum)
{
	NRF24_CE_LOW
	
	NRF24_CSN_LOW
	if(pipeNum > 5 || pipeNum == 0){
		return;
	}
	else{
		if(pipeNum == 1){
				nrf24_spi_write(NRF24_W_REGISTER | NRF24_RX_ADDR_P1); //0x2B
				for(int i = 0; i < 5; ++i){
					nrf24_spi_write(pypeAddress[4-i]);
				}
				NRF24_CSN_HIGH
		}
		else{
			nrf24_spi_write(NRF24_W_REGISTER | (NRF24_RX_ADDR_P0 + pipeNum));
			nrf24_spi_write(pypeAddress[4]);
			NRF24_CSN_HIGH

			NRF24_CSN_LOW																					// mind trick to enable pipe other then 0 or 1
			nrf24_spi_write(NRF24_R_REGISTER | NRF24_EN_RXADDR);
			uint8_t register_state = nrf24_spi_write(0xFF);
			NRF24_CSN_HIGH
			
			NRF24_CSN_LOW
			nrf24_spi_write(NRF24_W_REGISTER | NRF24_EN_RXADDR);
			nrf24_spi_write(register_state | 1<<pipeNum);
			NRF24_CSN_HIGH
		}
	}

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | (NRF24_RX_PW_P0 + pipeNum)); //0x32
	nrf24_spi_write(PAYLOAD_LENGTH);
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
}


//***********TX mode***********// 
// Turn on TX mode 
//*****************************//

void tx_mode(void)
{
	NRF24_CE_LOW
	
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_CONFIG);//0x20
	nrf24_spi_write(0x0E); // Transmiter mod
	NRF24_CSN_HIGH
	
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_TX);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_STATUS); //0x27
	nrf24_spi_write(0x70);
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
}


//***********TX mode***********// 
// Turn on RX mode 
//*****************************//

void rx_mode(void)
{
	NRF24_CE_LOW

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_CONFIG);//0x20
	nrf24_spi_write(0x0F); // Receiver mod
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_RX);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_STATUS); //0x27
	nrf24_spi_write(0x70);
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
}


//***************Send data**************// 
/*
Send data by splitting it into single bytes. Weird method, but really beautiful and resources friendly.
*/
//**************************************//

void nrf24_send_data(const void* data, uint32_t PayloadSize, int no_ack)
{
	NRF24_CE_LOW

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_TX);
	NRF24_CSN_HIGH

	uint8_t* current = data;
	int tmp =  PAYLOAD_LENGTH - PayloadSize;

	NRF24_CSN_LOW

	nrf24_spi_write(no_ack == 0 ? NRF24_W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK);

	for(int i = 0; i < PayloadSize; ++i){
		nrf24_spi_write(*current);
		current++;
	}
	for(int i = 0; i < tmp; ++i){
		nrf24_spi_write(0x00);
	}

	NRF24_CSN_HIGH //end of sending procedure

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_STATUS); //0x27
	nrf24_spi_write(0x70); // reseting status flags
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
}


//***********Check if data is availiable***********// 
/* 
check FIFO status to see if data is availiable
not returning size of payload becouse of the weird delay in work when i try
*/
//*************************************************//

uint8_t dataAvailiable(void)
{
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FIFO_STATUS);
	uint8_t tmp = nrf24_spi_write(0xFF);
	NRF24_CSN_HIGH
	
	if(((tmp & 0x01) != 0x01)){
		return 1;//(tmp & 0x0E)>>1; - to return pype number
	}
	return 0;
}


//************read data from NRF24l01+*************// 
/*
 If i right, all data srores in memory byte by byte, so, no matter what length we send we will be able to reconstruct data by
 fulling any array with needed data. If it works, same aproach can work with transmition, which will be greate.
*/
//*************************************************//
void readData(void *data, uint32_t PayloadSize)
{

	uint8_t *current = data;

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_R_RX_PAYLOAD);
	for ( int i = 0; i < PayloadSize; ++i )
	{
		*current++ = nrf24_spi_write(0xFF); //
	}

	int tmp = PAYLOAD_LENGTH - PayloadSize;
	for ( int i = 0; i < tmp; ++i )
	{
		nrf24_spi_write(0xFF);
	}

	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_RX);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_NRF_STATUS);
	nrf24_spi_write(NRF_STATUS_RESET);
	NRF24_CSN_HIGH
}


//************read data from NRF24l01+*************//
/*
 If i right, all data srores in memory byte by byte, so, no matter what length we send we will be able to reconstruct data by
 fulling any array with needed data. If it works, same aproach can work with transmition, which will be greate.
*/
//*************************************************//
uint32_t nrf24_check_if_alive(void)
{
	NRF24_CSN_LOW

	nrf24_spi_write(NRF24_RF_CH);

	if ( nrf24_spi_write(0xFF) )
	{
		NRF24_CSN_HIGH
		return 0;
	}

	return 1;
	NRF24_CSN_HIGH
}





