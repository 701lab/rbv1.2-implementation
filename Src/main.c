
// For global variables declareation
#define VAR_DECLS

#include "implementation.h"
#include "icm-20600.h"
#include "nrf24l01.h"
#include "motors.h"

int16_t icm_data[6] = { 0, 0, 0, 0, 0, 0 };

uint8_t icm_test_data[4] = { 0, 0, 0, 0 };



int main(void)
{
	full_device_setup();

	delay_in_milliseconds(1000000);

	motor motor1 = {.encoder_constant = 1540.0f, .max_duty_cycle_coefficient = PWM_PRECISION};

	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor1_pwm;


	motor motor2 = {.encoder_constant = 1540.0f, .max_duty_cycle_coefficient = PWM_PRECISION};
	motor1.motor_disable = gpioc6_low;
	motor1.motor_enable = gpioc6_high;
	motor1.set_pwm_duty_cycle = set_motor2_pwm;


	motor1.motor_enable();


//	basic_spi2_setup(5000000);
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

//	nrf24_basic_init();

//	TIM1->CCR4 = PWM_PRECISION/2;

	while(1){
//		icm_20600_get_sensors_data(&robot_imu, icm_data, 0);


		motor1.set_pwm_duty_cycle(PWM_PRECISION/2);
		motor2.set_pwm_duty_cycle(PWM_PRECISION/2);
		delay_in_milliseconds(10000000);

		motor1.set_pwm_duty_cycle(PWM_PRECISION);
		motor1.set_pwm_duty_cycle(PWM_PRECISION);
		delay_in_milliseconds(10000000);

		GPIOD->ODR ^= 0x03;
//		blink();
	}
}

/*
	@brief	System counter interrupt handler - place for all control logic code
 */
void SysTick_Handler()
{
	__NOP();
}

