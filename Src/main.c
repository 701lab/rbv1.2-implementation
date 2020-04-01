
// For global variables declareation
#define VAR_DECLS

#include "implementation.h"
#include "icm-20600.h"
#include "nrf24l01.h"
#include "motors.h"

int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t icm_test_data[4] = { 0, 0, 0, 0 };

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
					.kp = 200.0f,
					.ki = 5000.0f,
					.current_integral = 0.0f,
					.controller_output_limitation_value = PWM_PRECISION,
					.previous_encoder_counter_value = 0,
					.previous_speed_mistake = 0.0f,
					.target_speed = 0.0f,
					.regulator_control_signal = 0.0f,
					.current_speed = 0.0f
			};



int main(void)
{
	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor1_pwm;
	motor1.get_encoder_counter_value = get_motor1_encoder_value;
	motor1.speed_controller = &motor1_speed_cotroller;

	motor2.motor_disable = gpioc6_low;
	motor2.motor_enable = gpioc6_high;
	motor2.set_pwm_duty_cycle = set_motor2_pwm;
	motor2.get_encoder_counter_value = get_motor2_encoder_value;


	full_device_setup();

	// Enables both motors
	motor1.motor_enable();


	basic_spi1_setup(5000000);
//
//	icm_20600_instance robot_imu =
//				{ robot_imu.cs_high = gpiob12_high,
//				robot_imu.cs_low = gpiob12_low,
//				robot_imu.send_one_byte = spi2_write_single_byte };
//
//
//	 if( icm_20600_check_if_alive(&robot_imu))
//	 {
//		 add_to_mistakes_log(23);
//	 }


//	icm_20600_basic_init(&robot_imu, 0);

//	icm_20600_setup(&robot_imu, icm_gyro_2000dps, icm_accel_16g);

	nrf24_basic_init();

//	TIM1->CCR4 = PWM_PRECISION/2;

	while(1){
//		icm_20600_get_sensors_data(&robot_imu, icm_data, 0);

		motor1.speed_controller->target_speed = 0.5;
		delay_in_milliseconds(10000000);


		motor1.speed_controller->target_speed = 1.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = 2.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = 1.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = 0.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = -0.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = -1.5;
		delay_in_milliseconds(10000000);

		motor1.speed_controller->target_speed = -0.5;
		delay_in_milliseconds(10000000);


//		motor1.set_pwm_duty_cycle(PWM_PRECISION/2);
//
//		motor1.set_pwm_duty_cycle(PWM_PRECISION);
//		delay_in_milliseconds(10000000);

		GPIOD->ODR ^= 0x03;
//		blink();
	}
}

/*
	@brief	System counter interrupt handler - place for all control logic code
 */
//void SysTick_Handler()
//{
//	__NOP();
//}

void SysTick_Handler()
{
	++control_systems_counter;

	if(control_systems_counter%10 == 0)	// 20 раз в секунду
	{
		motors_get_speed_by_incements(&motor1, system_time_increment * 10);
		float speed_task = motors_speed_controller_handler(&motor1, system_time_increment * 10);
		motor1.set_pwm_duty_cycle((int32_t)speed_task);
	}



	if(control_systems_counter == 200)
	{
		GPIOD->ODR ^= 0x03;
		control_systems_counter = 0;
	}
}

