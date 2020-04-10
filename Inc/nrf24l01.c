#include "nrf24l01.h"

uint32_t nrf24_basic_init(nrf24l01p *nrf24_instance)
{
	// Check if the particular device was initialized properly
	uint32_t declaration_mistake_code = nrf24_check_declarations(nrf24_instance);
	if ( declaration_mistake_code )
	{
		// Function is immediately stopped
		return declaration_mistake_code;
	}

	// Check if the particular device is at least in power-down mode. It takes 100ms for the device to load into the power-down from the power on.
	// So if the function is called immediately device could not be ready yet
	uint32_t checks = 0;
	while ( nrf24_check_if_alive(nrf24_instance) )
	{
		// Small dummy delay. Efficiency will depend on MCU clock speed
		for (uint32_t dummy_delay = 0; dummy_delay < 1000; dummy_delay++){}

		checks += 1;

		// If after 10 checks device is still not responding return mistake code
		if ( checks == 10 )
		{
			return NRF24_DEVICE_IS_NOT_CONNECTED;
		}
	}

	// To start SPI transmission high to low front should be detected. So we should prepare lone setting it high
	nrf24_instance->csn_high();

	// Go to standby-1 mode in case init is called to re-setup device
	nrf24_instance->ce_low();

	// This function will return mistake code if mistake occures. If more then one mistake occures only code of the last one will be returned
	uint32_t mistake_code = 0;

	// Default config setup - disable all interrupts, enable CRC with 1 byte encoding, TX mode with NO power up
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_CONFIG);
	nrf24_instance->spi_write_byte(NRF24_INTERRUPTS_MASK | NRF24_EN_CRC | NRF24_CRCO);
	nrf24_instance->csn_high();

	// Checking frequency channel. Change to valid value if wrong and throw mistake. Then write value to NRF
	if ( nrf24_instance->frequency_channel < 1 )
	{
		nrf24_instance->frequency_channel = 1;
		mistake_code = NRF24_WRONG_CHANNEL_FREQUENCY;
	}
	else if ( nrf24_instance->frequency_channel > 124 )
	{
		nrf24_instance->frequency_channel = 124;
		mistake_code = NRF24_WRONG_CHANNEL_FREQUENCY;
	}
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_RF_CH);
	nrf24_instance->spi_write_byte(nrf24_instance->frequency_channel);
	nrf24_instance->csn_high();

	// Set up device data rate and power output
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_RF_SETUP);
	nrf24_instance->spi_write_byte(nrf24_instance->data_rate | nrf24_instance->power_output);
	nrf24_instance->csn_high();

	// Reset all interrupt flags in case setup is called more than one time to dynamically change parameters of nrf24l01+ instance
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_STATUS);
	nrf24_instance->spi_write_byte(NRF24_INTERRUPTS_MASK);
	nrf24_instance->csn_high();

	// Check if payload size is correct. If needed change value and throw mistake. Then write value for pipe 0
	if ( nrf24_instance->payload_size_in_bytes < 0 )
	{
		nrf24_instance->payload_size_in_bytes = 0;
		mistake_code = NRF24_WRONG_PAYLOAD_SIZE;
	}
	else if ( nrf24_instance->payload_size_in_bytes > 32 )
	{
		nrf24_instance->payload_size_in_bytes = 32;
		mistake_code = NRF24_WRONG_PAYLOAD_SIZE;
	}
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_RX_PW_P0);
	nrf24_instance->spi_write_byte(nrf24_instance->payload_size_in_bytes);
	nrf24_instance->csn_high();

	// Write same payload size for pipe 1
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_RX_PW_P1);
	nrf24_instance->spi_write_byte(nrf24_instance->payload_size_in_bytes);
	nrf24_instance->csn_high();

	// Device was initialized
	nrf24_instance->device_was_initialized = 1;

	return mistake_code;
}


//************ check if particular nrf24l01+ instance connected *************//
/*
	@brief Read RF_CH register (which is never equals 0). If response data equals 0 - device is not connected, returns mistake code

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return mistake code or 0 if no mistakes were found
*/
//*************************************************//
uint32_t nrf24_check_if_alive(nrf24l01p *nrf24_instance)
{
	nrf24_instance->csn_low();

	nrf24_instance->spi_write_byte(NRF24_RF_CH);

	if ( nrf24_instance->spi_write_byte(NRF24_NOP) )
	{
		nrf24_instance->csn_high();
		return 0;
	}

	nrf24_instance->csn_high();
	return NRF24_DEVICE_IS_NOT_CONNECTED;
}

//********************************************//
/*
	@brief Checks if all function pointers were initialized. If not returns mistake code.
		Those mistakes should be handled as critical, so any function calling this one should immediately return mistake code.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
//********************************************//
// This function takes nrf to standby-1 but does not put CE high, so it should be done elsewhere
uint32_t nrf24_power_up(nrf24l01p *nrf24_instance)
{
	// Check if device was initialized
	if(nrf24_instance->device_was_initialized == 0)
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	// Read current config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_R_REGISTER | NRF24_CONFIG);
	uint8_t current_register_state = nrf24_instance->spi_write_byte(NRF24_NOP);
	nrf24_instance->csn_high();

	// Add power up to current config
	current_register_state |= NRF24_PWR_UP;

	// Write new config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_CONFIG);
	nrf24_instance->spi_write_byte(current_register_state);
	nrf24_instance->csn_high();

	return 0;
}

//********************************************//
/*
	@brief Checks if all function pointers were initialized. If not returns mistake code.
		Those mistakes should be handled as critical, so any function calling this one should immediately return mistake code.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
//********************************************//
uint32_t nrf24_check_declarations(nrf24l01p *nrf24_instance)
{
	if ( nrf24_instance->ce_high == 0 )
	{
		return NRF24_CE_HIGH_FUNCTION_IS_MISSING;
	}

	if ( nrf24_instance->ce_low == 0 )
	{
		return NRF24_CE_LOW_FUNCTION_IS_MISSING;
	}

	if ( nrf24_instance->csn_high == 0 )
	{
		return NRF24_CSN_HIGH_FUNCTION_IS_MISSING;
	}

	if ( nrf24_instance->csn_low == 0 )
	{
		return NRF24_CSN_LOW_FUNCTION_IS_MISSING;
	}

	if ( nrf24_instance->spi_write_byte == 0 )
	{
		return NRF24_SPI_WRITE_FUNCTION_IS_MISSING;
	}

	return 0;
}



//***********NRF24l01+ main setup***********// 
// NRF is seted up as transmiter with address 0xAABBCCDD11
// 1 MBS, -6dBm, 32 payload length, 45 channel
//******************************************//

void nrf24_basic_init_old(void){

	NRF24_CSN_HIGH


	//Main nrf setup: bit 0 - RX-mod=1, TX-mod=0; bit 1 - Power up=1, Power down=0; bit 2 - 1 byte CRC=0, 2 byte CRC=1, bit 3 - Enable CRC
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_CONFIG);//0x20
	nrf24_spi_write(0x0E); // Transmiter mod
	NRF24_CSN_HIGH

//	//Next to registers - really advanced setup.
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_W_REGISTER | NRF24_FEATURE);		 //0x3D
//	nrf24_spi_write(0x00);
//	NRF24_CSN_HIGH
//
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_W_REGISTER | NRF24_DYNPD);	 		 //0x3C
//	nrf24_spi_write(0x00);
//	NRF24_CSN_HIGH
	
	//Clear tx and rx flags to communicate 
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS); //0x27
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

//	//Clear TX payload
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_FLUSH_TX);
//	NRF24_CSN_HIGH
//
//	//Clear RX payload
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_FLUSH_RX);
//	NRF24_CSN_HIGH

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
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_CONFIG);//0x20
	nrf24_spi_write(0x0E); // Transmiter mod
	NRF24_CSN_HIGH
	
	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_TX);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS); //0x27
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
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_CONFIG);//0x20
	nrf24_spi_write(0x0F); // Receiver mod
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_FLUSH_RX);
	NRF24_CSN_HIGH

	NRF24_CSN_LOW
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS); //0x27
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
//	NRF24_CE_LOW

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
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS); //0x27
	nrf24_spi_write(0x70); // reseting status flags
	NRF24_CSN_HIGH

	NRF24_CE_HIGH
	for (int i = 0; i < 1000; ++i){}
	NRF24_CE_LOW

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
	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS);
	nrf24_spi_write(NRF_STATUS_RESET);
	NRF24_CSN_HIGH
}

