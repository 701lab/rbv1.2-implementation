/*
 * mistakes_codes.h
 *
 *  Created on: Mar 27, 2020
 *      Author: Андрей
 */

#ifndef MISTAKES_CODES_H_
#define MISTAKES_CODES_H_

//*** MCU related mistakes ***//

#define MCU_MISTAKES_OFFSET					(0U)
#define HSE_FAILED_TO_START					(1U) + MCU_MISTAKES_OFFSET
#define HSE_FAILED_WHILE_RUNNING 			(2U) + MCU_MISTAKES_OFFSET
#define	PLL_FAILED							(3U) + MCU_MISTAKES_OFFSET
#define SYSCLK_BASIC_SETUP_FAILED			(4U) + MCU_MISTAKES_OFFSET
#define WRONG_SPI1_FREQUENCY_INPUT			(5U) + MCU_MISTAKES_OFFSET
#define WRONG_SPI2_FREQUENCY_INPUT			(6U) + MCU_MISTAKES_OFFSET
#define SPI1_TRANSMISSION_FAIL				(7U) + MCU_MISTAKES_OFFSET
#define SPI2_TRANSMISSION_FAIL				(8U) + MCU_MISTAKES_OFFSET
#define WRONG_UART1_FREQUENCY_INPUT			(9U) + MCU_MISTAKES_OFFSET
#define UART1_TRANSMISSION_FAIL				(10U) + MCU_MISTAKES_OFFSET

//*** NRF24L01+ mistakes ***//
#define NRF24_MISTAKES_OFFSER				(40U)
#define NRF_IS_NOT_RESPONDING				(1U) + NRF24_MISTAKES_OFFSER

//*** ICM-20600 mistakes ***//
#define ICM_20600_MISTAKES_OFFSET			(60U)
#define ICM_IS_NOT_RESPONDONG				(1U) + ICM_20600_MISTAKES_OFFSET

//*** Motor related mistakes ***//
#define MOTORS_MISTAKES_OFFSET				(100U)
#define M1_PWM_TASK_LOWER_THAN_MINIMUM		(1U) + MOTORS_MISTAKES_OFFSET
#define M1_PWM_TASK_HIGHER_THAN_MAXIMUM		(2U) + MOTORS_MISTAKES_OFFSET
#define M2_PWM_TASK_LOWER_THAN_MINIMUM		(3U) + MOTORS_MISTAKES_OFFSET
#define M2_PWM_TASK_HIGHER_THAN_MAXIMUM		(4U) + MOTORS_MISTAKES_OFFSET
#define M1_IS_NOT_CONNECTED					(5U) + MOTORS_MISTAKES_OFFSET
#define M2_IS_NOT_CONNECTED					(6U) + MOTORS_MISTAKES_OFFSET
#define M1_DIRECTION_MISMATCH				(7U) + MOTORS_MISTAKES_OFFSET
#define M2_DIRECTION_MISMATCH				(8U) + MOTORS_MISTAKES_OFFSET




#endif /* MISTAKES_CODES_H_ */
