#include "device.h"


void device_self_diagnosticks(icm_20600 * icm_instance, nrf24l01p *nrf24_instance, motor * first_motor_instance, motor * second_motor_instance)
{
	// Icm-20600 diagnostics
	add_to_mistakes_log(icm_20600_check_if_alive(icm_instance));

	// NRF24 diagnostics
	add_to_mistakes_log(nrf24_check_if_alive(nrf24_instance));

	// Motor1 rotation check
	uint32_t motors_rotation_test_resalt = motors_rotation_deiraction_test(first_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M1_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M1_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}

	// Motor 2 rotation check
	motors_rotation_test_resalt = motors_rotation_deiraction_test(second_motor_instance);
	switch(motors_rotation_test_resalt){
	case 2: add_to_mistakes_log(M2_IS_NOT_CONNECTED); break;
	case 1: add_to_mistakes_log(M2_DIRECTION_MISMATCH); break;
	case 0: break;
	default: break;
	}

}
