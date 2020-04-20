/*
	@brief This file contains all data specific to two sided balancing robot
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include "implementation.h"
#include "icm-20600.h"
#include "nrf24l01p.h"
#include "motors.h"


/*
	@brief Runs all board diagnostics tests
 */
void device_self_diagnosticks(icm_20600_instance * icm_instance, nrf24l01p *nrf24_instance, motor * first_motor_instance, motor * second_motor_instance);







#endif /* DEVICE_H_ */
