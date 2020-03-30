
// For global variables declareation
#define VAR_DECLS

#include "implementation.h"
#include "icm-20600.h"
#include "nrf24l01.h"

int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t icm_test_data[4] = { 0, 0, 0, 0 };

float information_to_be_send[3] = { 0, 54, 275 };

//
//int32_t icm_20600_init(uint8_t *responses);
//int32_t icm_20600_getData(int16_t *data);


// 	В такой реализации, когда включено прерывание, но при этом не написаны обработчики, программа крашится. Соответственно это хорошая возможность
// 	потестировать watchdog, чтобы он мог чуть что ресетнуть контроллер при необходимости

int main(void)
{


	full_device_setup();

	delay_in_milliseconds(1000000);

	basic_spi1_setup(5000000);

	nrf24_basic_init();

//	TIM1->CCR4 = PWM_PRECISION/2;

	while(1){
//		icm_basic_init


		GPIOD->ODR ^= 0x03;
		blink();
	}
}

/*
	@brief	System counter interrupt handler - place for all control logic code
 */
void SysTick_Handler()
{
	__NOP();
}



//
//int32_t icm_20600_init(uint8_t *responses)
//{
//	ICM_CS_LOW
//
//	responses[0] = icm_spi_write(0x6B); // write to 107 (0x6B) register
//	responses[1] = icm_spi_write(0x00); // 0 to wake ICM from sleep mode
//
//	ICM_CS_HIGH
//
//	ICM_CS_LOW
//
//	responses[2] = icm_spi_write(0x70); // write to 112 (0x70) register
//	responses[3] = icm_spi_write(0x40); // disable I2C interface
//
//	ICM_CS_HIGH
//
//	return 0;
//}
//
//
//int32_t icm_20600_getData(int16_t *data)
//{
//	ICM_CS_LOW
//	icm_spi_write(0x3b | 0x80); 	// reading data from 59 (0x3b) register
//
//	data[0] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get first accel data - 59-60 register
//	data[1] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get second accel data - 61-62 register
//	data[2] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get third accel data - 63-64 register
//	icm_spi_write(0xFF); // skip temperature data low byte - 65 register
//	icm_spi_write(0xFF); // skip temperature data high byte - 66 register
//	data[3] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get first gyro data - 67-68 registers
//	data[4] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get second gyro data - 69-70 registers
//	data[5] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF); // get third gyro data - 71-72 registers
//
//	ICM_CS_HIGH
//
//	return 0;
//}


