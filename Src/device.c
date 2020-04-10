#include "device.h"


void device_self_diagnosticks(icm_20600_instance * icm_instance, /* nrf24_instance */ motor * first_motor_instance, motor * second_motor_instance)
{
	// Icm-20600 diagnostics
	if(icm_20600_check_if_alive(icm_instance))
	{
		add_to_mistakes_log(ICM_IS_NOT_RESPONDONG);
	}

//	// NRF24 diagnostics
//	if(nrf24_check_if_alive())
//	{
//		add_to_mistakes_log(NRF_IS_NOT_RESPONDING);
//	}

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
