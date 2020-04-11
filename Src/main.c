/*
	@file main.c

	@version 1.0

	@brief This file contains all basic setups for robots board v1.2 and should be used for firmware development for devices using robots board v1.2 as a control board.
			All unnecessary declarations can be deleted during development
 */


// For global variables declareation
#define VAR_DECLS

#include "implementation.h"
#include "device.h"

int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };
//float nrf_data[3] = {1.0f, 2.0f, 3.0f};
float nrf_data[2] = {1.0f, 2.0f};

uint32_t control_systems_counter = 0;
float	system_time_increment = 1.0f/SYSTICK_FREQUENCY;

motor motor1 =
			{
					.encoder_constant = 1540.0f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

motor motor2 =
			{
					.encoder_constant = 1540.0f,
					.max_duty_cycle_coefficient = PWM_PRECISION
			};

speed_control motor1_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor1_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};


speed_control motor2_speed_cotroller =
			{
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};

position_control motor2_position_controller =
			{
					.current_position = 0.0f,
					.previous_encoder_counter_value = 0.0f,
					.regulator_control_signal = 0.0f,
					.target_position = 0.0f,
			};

icm_20600_instance robot_imu;

nrf24l01p robot_nrf24;

uint8_t addrForTx[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

uint8_t current_register_state = 0;


uint32_t current_int_stat1 = 0;
uint32_t current_int_stat2 = 0;




int main(void)
{

	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor1_pwm;
	motor1.get_encoder_counter_value = get_motor1_encoder_value;
	motor1.speed_controller = &motor1_speed_cotroller;
	motor1.position_controller = &motor1_position_controller;
	motor1_speed_cotroller.kp = 200.0f;
	motor1_speed_cotroller.ki = 5000.0f;
	motor1_position_controller.kp = 1.0f;
	motor1_position_controller.position_precision = 8.0f/motor1.encoder_constant;

	motor2.motor_disable = gpioc6_low;
	motor2.motor_enable = gpioc6_high;
	motor2.set_pwm_duty_cycle = set_motor2_pwm;
	motor2.get_encoder_counter_value = get_motor2_encoder_value;
	motor2.speed_controller = &motor2_speed_cotroller;
	motor2.position_controller = &motor2_position_controller;
	motor2_speed_cotroller.kp = 200.0f;
	motor2_speed_cotroller.ki = 5000.0f;
	motor2_position_controller.kp = 1.0f;
	motor2_position_controller.position_precision = motor1_position_controller.position_precision;

	robot_imu.cs_high = gpiob12_high;
	robot_imu.cs_low = gpiob12_low;
	robot_imu.send_one_byte = spi2_write_single_byte;

	robot_nrf24.ce_high = gpiob0_high;
	robot_nrf24.ce_low = gpiob0_low;
	robot_nrf24.csn_high = gpiob1_high;
	robot_nrf24.csn_low = gpiob1_low;
	robot_nrf24.spi_write_byte = spi1_write_single_byte;
	robot_nrf24.frequency_channel = 45;
	robot_nrf24.payload_size_in_bytes = 12;
	robot_nrf24.power_output = nrf24_pa_high;
	robot_nrf24.data_rate = nrf24_1_mbps;

//	nrf24_power_up(&robot_nrf24);


	full_device_setup(no);


	// Enables both motors
//	motor1.motor_enable();

	basic_spi1_setup(5000000);
	basic_spi2_setup(5000000);

//	delay_in_milliseconds(100);

//	nrf24_basic_init_old();

//	setTxAddress(addrForTx);


	add_to_mistakes_log(nrf24_basic_init(&robot_nrf24));

	add_to_mistakes_log(nrf24_power_up(&robot_nrf24));

	add_to_mistakes_log(nrf24_set_tx_address(&robot_nrf24, addrForTx));

//	add_to_mistakes_log(nrf24_enable_interrupts(&robot_nrf24, yes, no, yes));


//	robot_nrf24.ce_high();

	icm_20600_basic_init(&robot_imu, 0);

//	motor1.speed_controller->target_speed = 1.0f;
//	motor2.speed_controller->target_speed = 1.0f;

	while(1)
	{

		robot_nrf24.csn_low();
		robot_nrf24.spi_write_byte(NRF24_R_REGISTER | NRF24_CONFIG);
		current_register_state = robot_nrf24.spi_write_byte(NRF24_NOP);
		robot_nrf24.csn_high();

		current_int_stat1 = nrf24_get_interrupts_status(&robot_nrf24);
		current_int_stat2 = nrf24_get_interrupts_status(&robot_nrf24);

		if(nrf24_check_if_alive(&robot_nrf24)){
			GPIOD->ODR |= 0x06;
		}

//		add_to_mistakes_log(nrf24_check_if_alive(&robot_nrf24));

		icm_20600_get_sensors_data(&robot_imu, icm_data, no);
		add_to_mistakes_log(nrf24_send_message(&robot_nrf24, nrf_data, 8, yes));

		delay_in_milliseconds(1000);
		GPIOD->ODR ^= 0x01;
	}
}

void SysTick_Handler()
{
	++control_systems_counter;

	// Speed control handling
	if(control_systems_counter%2 == 0)	// 20 time per second
	{
		motors_get_speed_by_incements(&motor1, system_time_increment * 2.0f);
		motors_get_speed_by_incements(&motor2, system_time_increment * 2.0f);
		float m1_speed_task = motors_speed_controller_handler(&motor1, system_time_increment * 2.0f);
		float m2_speed_task = motors_speed_controller_handler(&motor2, system_time_increment * 2.0f);
		motor1.set_pwm_duty_cycle((int32_t)m1_speed_task);
		motor2.set_pwm_duty_cycle((int32_t)m2_speed_task);
	}

	// Position control handling
	if(control_systems_counter%4 == 0)	// 10 times per second
	{
		motors_get_position(&motor1);
		motor1.speed_controller->target_speed = motors_position_controller_handler(&motor1);
	}

	if(control_systems_counter == 20)
	{
		GPIOD->ODR ^= 0x08;
		control_systems_counter = 0;
	}
}

