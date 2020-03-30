
// For global variables declareation
#define VAR_DECLS

#include "implementation.h"
#include "icm-20600.h"
#include "nrf24l01.h"

//int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };
//uint8_t icm_test_data[4] = { 0, 0, 0, 0 };

uint8_t addrForRx[5] = {0xAA,0xBB,0xCC,0xEE,0x10};
//uint8_t addrForRx3[5] = {0xAA,0xBB,0xCC,0xEE,0x15};		// Green leds car
uint8_t addrForRx3[5] = {0xAA,0xBB,0xCC,0xEE,0x25};	// blue leds car
uint16_t nrfDataArray[5] = {0, 0, 0, 0, 0};

void control_robot();


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

void control_tobot(void)
{
	if(dataAvailiable()){
		readData(nrfDataArray, 10);
		GPIOD->ODR |= 0x08;	// LED that shows connection with controller

		dataHasBeenCaptured = 1;


		setPwmWidth(&motor1, 0); //stop in case there is no command sent
		setPwmWidth(&motor2, 0);


		if(nrfDataArray[2] < 1000 /*means it up*/ && nrfDataArray[1] < 3000 && nrfDataArray[1] > 1000){
			setPwmWidth(&motor1, currentSpeedTask);
			setPwmWidth(&motor2, currentSpeedTask);
//			GPIOB->ODR |= 0x90; // forward
		}
		else if(nrfDataArray[2] < 1000 /*means it up*/ && nrfDataArray[1] < 1000){
			setPwmWidth(&motor1, currentSpeedTask);
			setPwmWidth(&motor2, 299);
//			setPwmWidth(&motor2, 0);
//			GPIOB->ODR |= 0x80; // forward left
		}
		else if(nrfDataArray[2] > 1000 && nrfDataArray[2] < 3000 && nrfDataArray[1] < 1000){
			setPwmWidth(&motor1, currentSpeedTask - 200);
			setPwmWidth(&motor2, -currentSpeedTask + 200);
//			GPIOB->ODR |= 0xA0; // turn left
		}
		else if(nrfDataArray[2] > 3000 && nrfDataArray[1] < 1000){
			setPwmWidth(&motor1, -currentSpeedTask);
			setPwmWidth(&motor2, -299);
//			setPwmWidth(&motor2, 0);
//			GPIOB->ODR |= 0x40; // backward left
		}
		else if(nrfDataArray[2] > 3000  && nrfDataArray[1] < 3000 && nrfDataArray[1] > 1000){
			setPwmWidth(&motor1, -currentSpeedTask);
			setPwmWidth(&motor2, -currentSpeedTask);
//			GPIOB->ODR |= 0x60; // backward
		}
		else if(nrfDataArray[2] > 3000 && nrfDataArray[1] > 3000){
//			setPwmWidth(&motor1, 0);
			setPwmWidth(&motor1, -299);
			setPwmWidth(&motor2, -currentSpeedTask);
//			GPIOB->ODR |= 0x20; // backward right
		}
		else if(nrfDataArray[2] < 1000 && nrfDataArray[1] > 3000){
//			setPwmWidth(&motor1, 0);
			setPwmWidth(&motor1, 299);
			setPwmWidth(&motor2, currentSpeedTask);
//			GPIOB->ODR |= 0x10; // forward right
		}
		else if(nrfDataArray[2] > 1000 && nrfDataArray[2] < 3000 && nrfDataArray[1] > 3000){
			setPwmWidth(&motor1, -currentSpeedTask+200);
			setPwmWidth(&motor2, currentSpeedTask-200);
//			GPIOB->ODR |= 0x50; // turn right
		}

		if((nrfDataArray[4] & 0x14) == 0x14){ // means, that top right button is unpressed
			currentSpeedTask = 599;
		}
		else if((nrfDataArray[4] & 0x04) == 0){
			currentSpeedTask = 899;
		}
		else{
			currentSpeedTask = 1199;
		}
	}
}

