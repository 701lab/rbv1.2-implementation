#include "nrf24l01.h"

// ******************* Function ******************* // (V)
/*
	@brief Read RF_CH register (which is never equals 0). If response data equals 0 - device is not connected, returns mistake code

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return mistake code or 0 if no mistakes were found
*/
// ************************************************ //
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

	// This function will return mistake code if mistake occurs. If more then one mistake occurs only code of the last one will be returned
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


// ******************* Function ******************* // (V)
/*
	@brief Read RF_CH register (which is never equals 0). If response data equals 0 - device is not connected, returns mistake code

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return mistake code or 0 if no mistakes were found
*/
// ************************************************ //
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

// ******************* Function ******************* // (V)
/*
	@brief Add PWR_UP bit to NRF24l01+ CONFIG register.
		Doesn't touch CE pin. So id CE = 0 device will enter standby-1 mode. IF CE = 1 depending on PRIM_RX and RX FIFO can enter either RX, TX or standby-2 mode.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
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

// ******************* Function ******************* // (V)
/*
	@brief Checks if all function pointers were initialized. If not returns mistake code.
		Those mistakes should be handled as critical, so any function calling this one should immediately return mistake code.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
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

// ******************* Function ******************* //
/*
	@brief change device mode to TX, but not pulls CE logic high for power efficiency - if powered up stays in standby-1.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
uint32_t nrf24_tx_mode(nrf24l01p *nrf24_instance)
{
	// Check if device was initialized
	if ( nrf24_instance->device_was_initialized == 0 )
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	// Go to standby-1 mode if CE = 1 to safely switch modes
	nrf24_instance->ce_low();

	// Read current config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_R_REGISTER | NRF24_CONFIG);
	uint8_t current_register_state = nrf24_instance->spi_write_byte(NRF24_NOP);
	nrf24_instance->csn_high();

	// Add power up to current config
	current_register_state &= ~NRF24_PRIM_RX;

	// Write new config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_CONFIG);
	nrf24_instance->spi_write_byte(current_register_state);
	nrf24_instance->csn_high();

	// Clear TX FIFO if it was not empty
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_FLUSH_TX);
	nrf24_instance->csn_high();

	// Reset all interrupt flags
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_STATUS);
	nrf24_instance->spi_write_byte(NRF24_INTERRUPTS_MASK);
	nrf24_instance->csn_high();

	return 0;
}

// ******************* Function ******************* //
/*
	@brief Changes device settings to RX, if device is powered up goes to RX mode

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
uint32_t nrf24_rx_mode(nrf24l01p *nrf24_instance)
{
	// Check if device was initialized
	if ( nrf24_instance->device_was_initialized == 0 )
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	// Go to standby-1 mode
	nrf24_instance->ce_low();

	// Read current config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_R_REGISTER | NRF24_CONFIG);
	uint8_t current_register_state = nrf24_instance->spi_write_byte(NRF24_NOP);
	nrf24_instance->csn_high();

	// Add power up to current config
	current_register_state |= NRF24_PRIM_RX;

	// Write new config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_CONFIG);
	nrf24_instance->spi_write_byte(current_register_state);
	nrf24_instance->csn_high();

	// Clear RX FIFO if it was not empty
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_FLUSH_RX);
	nrf24_instance->csn_high();

	// Reset all interrupt flags
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_STATUS);
	nrf24_instance->spi_write_byte(NRF24_INTERRUPTS_MASK);
	nrf24_instance->csn_high();

	// NRF should always be in RX mode not to miss data
	nrf24_instance->ce_high();

	return 0;
}

// ******************* Function ******************* // (V)
/*
	@brief Sets new values for retransmit delay and count of retransmissions for particular nrf24l01+ device.
		If retransmit count = 0 no retransmissions are produced.
		This setup is very important if data is transmitted with ACK message.
		To have ability to retransmit 32 bytes of data with ACC, retransmit delay should be at least 500us. 250us (default value) is enough for:
		- 5 bytes payload at 1 Mbps data rate;
		- 15 bytes palyload at 2 Mbps data rate.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called
	@param [in] new_retransmit_delay - new value of retransmission delay, should be expressed as nrf24_auto_retransmit_delay enum member
	@param [in] new_retransmit_count - new retransmit count

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
uint32_t nrf24_update_retransmission_params(nrf24l01p * nrf24_instance, nrf24_auto_retransmit_delay new_retransmit_delay, uint32_t new_retransmit_count)
{
	// Check if device was initialized
	if ( nrf24_instance->device_was_initialized == 0 )
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	uint32_t mistake_code = 0;

	if(new_retransmit_count < 0)
	{
		new_retransmit_count = 0;
		mistake_code = NRF24_WRONG_RETRANSMIT_COUNT;
	}
	else if ( new_retransmit_count > 15 )
	{
		new_retransmit_count = 15;
		mistake_code = NRF24_WRONG_RETRANSMIT_COUNT;
	}

	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_SETUP_RETR);
	nrf24_instance->spi_write_byte(new_retransmit_delay | new_retransmit_count);
	nrf24_instance->csn_high();

	return mistake_code;
}

// ******************* Function ******************* // (V)
/*
	@brief Sets new TX and RX pipe 0 addresses.
		Addresses should be written if reverse order, so that they can be represented as they were stored in the array.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called
	@param [in] new_tx_address[5] - array which contains new tx address. Programmer must make sure, that array is exactly 5 elements long.

	@return first found mistake code or 0 if no mistakes were found

	@note This function doesn't put CE in logic High, so if before the function call nrf24 instance was in RX mode, it won't automatically return into it.
		It is considered, that user only needs to change TX address when using device in TX mode, so CE should stay at logic low for power efficiency.
 */
// ************************************************ //
//@note This function doesn't put CE in logic High, so if operation ws
uint32_t nrf24_set_tx_address(nrf24l01p * nrf24_instance, const uint8_t new_tx_address[5])
{
	// Check if device was initialized
	if ( nrf24_instance->device_was_initialized == 0 )
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	// if device was in RX or TX mode it should be reset to standby-1
	nrf24_instance->ce_low();

	// Change address of pipe 0 which is used for reception  of ACK
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_RX_ADDR_P0);
	for(uint32_t i = 0; i < 5; ++i)
	{
		nrf24_instance->spi_write_byte(new_tx_address[4-i]);
	}
	nrf24_instance->csn_high();

	// Change TX address
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_TX_ADDR);
	for(uint32_t i = 0; i < 5; ++i)
	{
		nrf24_instance->spi_write_byte(new_tx_address[4-i]);
	}
	nrf24_instance->csn_high();

	return 0;
}



// ******************* Function ******************* // (V)
/*
	@brief Checks if all function pointers were initialized. If not returns mistake code.
		Those mistakes should be handled as critical, so any function calling this one should immediately return mistake code.

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return first found mistake code or 0 if no mistakes were found
 */
// ************************************************ //
uint32_t nrf24_send_message(nrf24l01p * nrf24_instance,  void *payload, uint32_t payload_size_in_bytes, int32_t should_send_ack)
{
	if(nrf24_instance->device_was_initialized == 0)
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}


	uint32_t mistake_code = 0;

	if (payload_size_in_bytes > nrf24_instance->payload_size_in_bytes)
	{
		payload_size_in_bytes = nrf24_instance->payload_size_in_bytes;
		mistake_code = NRF24_WRONG_MESSAGE_SIZE;
	}

	// Data will be sent byte by byte
	uint8_t *current_byte_to_send = payload;

	// If payload input is shorter then actual payload, not specified bytes will be sent as 0.
	uint32_t amount_of_zeros_requered = nrf24_instance->payload_size_in_bytes - payload_size_in_bytes;

	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(should_send_ack == 1 ? NRF24_W_TX_PAYLOAD : W_TX_PAYLOAD_NO_ACK);

	// Send data
	for  (uint32_t i = 0; i < payload_size_in_bytes; ++i)
	{
		nrf24_instance->spi_write_byte(*current_byte_to_send);
		current_byte_to_send++;
	}

	// !!! надо попрбовать вообще без этого момента и посмотреть что будет приходить
	// Fill empty space with 0 if needed.
	for (uint32_t i = 0; i < amount_of_zeros_requered; ++i)
	{
		nrf24_instance->spi_write_byte(0);
	}
	nrf24_instance->csn_high();


	// Send device into the Tx mode to send one payload
	nrf24_instance->ce_high();
	for (int i = 0; i < 500; ++i){}
	nrf24_instance->ce_low();

	return mistake_code;
}


// ******************* Function ******************* // (V)
/*
	@brief Reads NRF24_STATUS register, clears all interrupts and returns only interrupt flags states

	@param [in] nrf24_instance - pointer to the nrf24l01p instance for which function is called

	@return NRF24_STATUS register interrupts bits
 */
// ************************************************ //
uint32_t nrf24_get_interrupts_status(nrf24l01p * nrf24_instance)
{

	nrf24_instance->csn_low();
	uint32_t interrupt_status = nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_STATUS) & 0xF0;
	nrf24_instance->spi_write_byte(NRF24_INTERRUPTS_MASK);
	nrf24_instance->csn_high();

	return interrupt_status;
}

uint32_t nrf24_enable_interrupts(nrf24l01p *nrf24_instance,	uint32_t enable_rx_dr, uint32_t enable_tx_ds, uint32_t enable_max_rt)
{
	if ( nrf24_instance->device_was_initialized == 0 )
	{
		return NRF24_INSTANCE_WAS_NOT_INITIALIZED;
	}

	uint8_t interrupts_to_be_enabled = 0;

	if ( enable_rx_dr > 0 )
	{
		interrupts_to_be_enabled |= 0x40;
	}
	if ( enable_tx_ds > 0 )
	{
		interrupts_to_be_enabled |= 0x20;
	}
	if ( enable_max_rt > 0 )
	{
		interrupts_to_be_enabled |= 0x10;
	}

	// Read current config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_R_REGISTER | NRF24_CONFIG);
	uint8_t current_register_state = nrf24_instance->spi_write_byte(NRF24_NOP);
	nrf24_instance->csn_high();

	// Remove interrupt masking from current config state
	current_register_state &= ~interrupts_to_be_enabled;

	// Write new config state
	nrf24_instance->csn_low();
	nrf24_instance->spi_write_byte(NRF24_W_REGISTER | NRF24_CONFIG);
	nrf24_instance->spi_write_byte(current_register_state);
	nrf24_instance->csn_high();

	return 0;
}





//*************************************************************



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





////***********TX mode***********//
//// Turn on RX mode
////*****************************//
//
//void rx_mode(void)
//{
//	NRF24_CE_LOW
//
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_W_REGISTER | NRF24_CONFIG);//0x20
//	nrf24_spi_write(0x0F); // Receiver mod
//	NRF24_CSN_HIGH
//
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_FLUSH_RX);
//	NRF24_CSN_HIGH
//
//	NRF24_CSN_LOW
//	nrf24_spi_write(NRF24_W_REGISTER | NRF24_STATUS); //0x27
//	nrf24_spi_write(0x70);
//	NRF24_CSN_HIGH
//
//	NRF24_CE_HIGH
//}


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

