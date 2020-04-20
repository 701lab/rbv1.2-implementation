/*
	@file main.c

	@brief This is a main file of the balancing robot prototype project. All desired source code for sensing, handling, and controlling is accumulated in here.
 */

// For global variables declaration
#define VAR_DECLS

#include "implementation.h"
#include "device.h"


uint32_t control_systems_counter = 0;
float	system_time_increment = 1.0f/SYSTICK_FREQUENCY;


// ********************************** //
// ****** Motor 1 declarations ****** //
// ********************************** //
motor motor1 =
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

// ********************************** //
// ****** Motor 2 declarations ****** //
// ********************************** //
motor motor2 =
			{
					.encoder_constant = 1540.0f,
					.max_duty_cycle_coefficient = PWM_PRECISION
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


// *********************************** //
// ****** ICM-20600 declaration ****** //
// *********************************** //
icm_20600_instance robot_imu = {.device_was_initialized = 0};
int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };

// *********************************** //
// ****** NRF24L01+ declaration ****** //
// *********************************** //
nrf24l01p robot_nrf24 = {.device_was_initialized = 0};

uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x25};	// blue LEDs car
uint16_t nrf_input_data[5] = {0, 0, 0, 0, 0};


int main(void)
{

	// ****** Motor 1 initialization ****** //
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

	// ****** Motor 2 initialization ****** //
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

	// ****** ICM-20600 initialization ****** //
	robot_imu.cs_high = gpiob12_high;
	robot_imu.cs_low = gpiob12_low;
	robot_imu.send_one_byte = spi2_write_single_byte;

	// ****** NRF24L01+ initialization ****** //
	robot_nrf24.ce_high = gpiob0_high;
	robot_nrf24.ce_low = gpiob0_low;
	robot_nrf24.csn_high = gpiob1_high;
	robot_nrf24.csn_low = gpiob1_low;
	robot_nrf24.spi_write_byte = spi1_write_single_byte;
	robot_nrf24.frequency_channel = 45;
	robot_nrf24.payload_size_in_bytes = 10;
	robot_nrf24.power_output = nrf24_pa_high;
	robot_nrf24.data_rate = nrf24_1_mbps;

	// Init MCU peripherals
	full_device_setup(yes);

	// Enables both motors
	motor1.motor_enable();

//	delay_in_milliseconds(100);

	// NRF24L01+ device setup
	add_to_mistakes_log(nrf24_basic_init(&robot_nrf24));
	add_to_mistakes_log(nrf24_enable_pipe1(&robot_nrf24, nrf24_rx_address));
	add_to_mistakes_log(nrf24_rx_mode(&robot_nrf24));
//	add_to_mistakes_log(nrf24_enable_interrupts(&example_nrf24, yes, no, yes));

	icm_20600_basic_init(&robot_imu, 0);

	while(1)
	{
		icm_20600_get_sensors_data(&robot_imu, icm_data, no);

		delay_in_milliseconds(500);
		GPIOD->ODR ^= 0x01;
	}
}

void SysTick_Handler()
{
	++control_systems_counter;

	// Speed control handling
	if(control_systems_counter%2 == 0)
	{
		motors_get_speed_by_incements(&motor1, system_time_increment * 2.0f);
		motors_get_speed_by_incements(&motor2, system_time_increment * 2.0f);
		float m1_speed_task = motors_speed_controller_handler(&motor1, system_time_increment * 2.0f);
		float m2_speed_task = motors_speed_controller_handler(&motor2, system_time_increment * 2.0f);
		motor1.set_pwm_duty_cycle((int32_t)m1_speed_task);
		motor2.set_pwm_duty_cycle((int32_t)m2_speed_task);
	}

	// Position control handling
	if(control_systems_counter%4 == 0)
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
