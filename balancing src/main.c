// $branch$


// For global variables declaration
#define VAR_DECLS

#include "implementation.h"
#include "device.h"


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
// Пока оставлю, не уверен, чт одля данного проекта мне это нужно, но все же
icm_20600_instance robot_imu = {.device_was_initialized = 0};
int16_t icm_raw_data[7] = { 0, 0, 0, 0, 0, 0, 0 };
float icm_processed_data[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
int16_t gyro_calib_values[3] = {0, 0, 0};

// *********************************** //
// ****** NRF24L01+ declaration ****** //
// *********************************** //
nrf24l01p robot_nrf24 = {.device_was_initialized = 0};

//uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x15};	// green LEDs car
uint8_t nrf24_rx_address[5] = {0xAA,0xBB,0xCC,0xEE,0x25};	// blue LEDs car
uint16_t nrf_input_data[5] = {0, 0, 0, 0, 0};

uint32_t nrf24_data_has_been_captured = 0;
uint32_t nrf24_safety_counter = 0;

uint32_t someVar = 0;

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
	robot_imu.gyro_full_scale_setup = icm_gyro_500dps;
	robot_imu.accel_full_scale_setup = icm_accel_2g;
	robot_imu.enable_temperature_sensor = 0;
	// If the device is not calibrated should all be 0 for proper calibration. If gyro scale was changed, values should be recalibrated
	robot_imu.gyro_calibration_coefficients[icm_x] = -23;
	robot_imu.gyro_calibration_coefficients[icm_y] = 428;
	robot_imu.gyro_calibration_coefficients[icm_z] = -81;

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

	// Initialization of MCU peripherals
	full_device_setup(yes, yes);
	GPIOD->ODR |= 0x01;

	// Enables both motors
	motor1.motor_enable();

	delay_in_milliseconds(200);

	// NRF24L01+ device setup
	add_to_mistakes_log(nrf24_basic_init(&robot_nrf24));
	add_to_mistakes_log(nrf24_enable_pipe1(&robot_nrf24, nrf24_rx_address));
	add_to_mistakes_log(nrf24_enable_interrupts(&robot_nrf24, yes, no, no));
	add_to_mistakes_log(nrf24_rx_mode(&robot_nrf24));

	add_to_mistakes_log(icm_20600_basic_init(&robot_imu));

	imu_gyro_calibration(&robot_imu, gyro_calib_values);

//	device_self_diagnosticks(&robot_imu, &robot_nrf24, &motor1, &motor2);


	while(1)
	{
		delay_in_milliseconds(50);

		add_to_mistakes_log(icm_20600_get_raw_data(&robot_imu, icm_raw_data));
//		add_to_mistakes_log(icm_20600_get_proccesed_data(&robot_imu, icm_processed_data));
//		GPIOD->ODR ^= 0x08;
	}
}


// ****** Control loop handler ****** //
// Control loop handles by the SysTick timer

// *** Speed controller setup variables ***//
uint32_t speed_loop_call_counter = 0;

#define SPEED_LOOP_FREQUENCY				20	// Times per second. Must be not bigger then SYSTICK_FREQUENCY.
#define SPEED_LOOP_COUNTER_MAX_VALUE 		SYSTICK_FREQUENCY / SPEED_LOOP_FREQUENCY	// Times.
#define SPEED_LOOP_PERIOD					1.0f / (float)(SPEED_LOOP_FREQUENCY) // Seconds.

// *** Position controller setup variables *** //
uint32_t position_loop_call_counter = 0;

#define POSITION_LOOP_FREQUENCY				10	// Times per second. Must be not bigger than SYSTICK_FREQUENCY. It is better if it is at least 2 times slower than the speed loop.
#define POSITION_LOOP_COUNTER_MAX_VALUE 	SYSTICK_FREQUENCY / POSITION_LOOP_FREQUENCY	// Times
//#define POSITION_LOOP_PERIOD				(float)(POSITION_LOOP_FREQUENCY) / (float)(SYSTICK_FREQUENCY) // Seconds

void SysTick_Handler()
{

	// *** Speed control handling *** //
	speed_loop_call_counter += 1;
	if ( speed_loop_call_counter == SPEED_LOOP_COUNTER_MAX_VALUE )	// 20 times per second
	{
		speed_loop_call_counter = 0;
		motors_get_speed_by_incements(&motor1, SPEED_LOOP_PERIOD);
		motors_get_speed_by_incements(&motor2, SPEED_LOOP_PERIOD);
		float m1_speed_task = motors_speed_controller_handler(&motor1, SPEED_LOOP_PERIOD);
		float m2_speed_task = motors_speed_controller_handler(&motor2, SPEED_LOOP_PERIOD);
		motor1.set_pwm_duty_cycle((int32_t)m1_speed_task);
		motor2.set_pwm_duty_cycle((int32_t)m2_speed_task);
	}

//	// *** Position control handling *** //
//	position_loop_call_counter += 1;
//	if ( position_loop_call_counter == POSITION_LOOP_COUNTER_MAX_VALUE )	// 10 times per second
//	{
//		position_loop_call_counter = 0;
//		motors_get_position(&motor1);
//		motors_get_position(&motor2);
//		motor1.speed_controller->target_speed = motors_position_controller_handler(&motor1);
//		motor2.speed_controller->target_speed = motors_position_controller_handler(&motor2);
//	}


	// *** Nrf24l01+ safety clock *** //
	if(nrf24_data_has_been_captured == 1)
	{
		nrf24_data_has_been_captured = 0;
		nrf24_safety_counter = 0;
	}
	else
	{
		nrf24_safety_counter += 1;
		if(nrf24_safety_counter == SYSTICK_FREQUENCY) // Exactly one second delay
		{
			// Stop and show that data is not capturing any more
			GPIOD->ODR &= ~0x01;
			motor1.speed_controller->target_speed = 0.0f;
			motor2.speed_controller->target_speed = 0.0f;
		}
	}

}
// **************************************** //

// ****** NRf24l01+ IRQ handler ****** //
void EXTI2_3_IRQHandler()
{
	// Clear interrupt flag
	EXTI->FPR1 |= 0x04;

	// Get new data
	add_to_mistakes_log(nrf24_read_message(&robot_nrf24, nrf_input_data, 10));
	GPIOD->ODR |= 0x01;

	nrf24_data_has_been_captured = 1;
	float left_motor_speed_task = 0.0f;
	float left_motor_boost = 0.0f;
	float right_motor_speed_task = 0.0f;
	float right_motor_boost = 0.0f;

	// Check for buttons press
	if((nrf_input_data[4] & 0x14) == 0x14){ // means, that top right button is unpressed
		// Do nothing
	}
	else if((nrf_input_data[4] & 0x04) == 0){
		left_motor_boost = 0.3f;
		right_motor_boost = 0.3f;
	}
	else{
		left_motor_boost = 0.6f;
		right_motor_boost = 0.6f;
	}

	// Evaluate the speed tasks
	if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000) // Forward
	{
		left_motor_speed_task = 1.0f + left_motor_boost;
		right_motor_speed_task = 1.0f + right_motor_boost;
	}
	else if(nrf_input_data[2] < 1000 /*means it up*/ && nrf_input_data[1] < 1000)	// Forward left
	{
		left_motor_speed_task = 0.2f + left_motor_boost;
		right_motor_speed_task = 1.0f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] < 1000)	// Turn left
	{
		left_motor_speed_task = -0.6f - left_motor_boost;
		right_motor_speed_task = 0.6f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] < 1000)	// Backward left
	{
		left_motor_speed_task = -0.2f - left_motor_boost;
		right_motor_speed_task = -1.0f - right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000  && nrf_input_data[1] < 3000 && nrf_input_data[1] > 1000)	// Backward
	{
		left_motor_speed_task = -1.0f - left_motor_boost;
		right_motor_speed_task = -1.0f - right_motor_boost;
	}
	else if(nrf_input_data[2] > 3000 && nrf_input_data[1] > 3000)	// Backward right
	{
		left_motor_speed_task = -1.0f - left_motor_boost;
		right_motor_speed_task = -0.2f - right_motor_boost;
	}
	else if(nrf_input_data[2] < 1000 && nrf_input_data[1] > 3000)	// Forward right
	{
		left_motor_speed_task = 1.0f + left_motor_boost;
		right_motor_speed_task = 0.2f + right_motor_boost;
	}
	else if(nrf_input_data[2] > 1000 && nrf_input_data[2] < 3000 && nrf_input_data[1] > 3000)	// Turn right
	{
		left_motor_speed_task = 0.6f + left_motor_boost;
		right_motor_speed_task = -0.6f - right_motor_boost;
	}

	// Set new speed tasks
	motor1.speed_controller->target_speed = left_motor_speed_task;
	motor2.speed_controller->target_speed = right_motor_speed_task;
}
// **************************************** //

// EOF

