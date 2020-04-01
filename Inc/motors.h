#ifndef MOTORS_H_
#define MOTORS_H_

#include "implementation.h"





typedef struct{

	float Kp;
	float targetPosition;
	float currentPosition;
	float regulatorControlSignal;
	int16_t previousEncoderCounterValue;

} position_control;


typedef struct{

	float current_integral;
	float kp;
	float ki;
	float target_speed;		// Revolutions of reducer shaft per second
	float current_speed;
	float previous_speed_mistake; // Used in trapezoidal integration
	float regulator_control_signal;
	float controller_output_limitation_value;
	int16_t previous_encoder_counter_value;

} speed_control;


typedef struct{

	float torque_coeficient;	// Cm

}torque_control;

//****** Motor definition for control ******//
/*
	@ brief

	@Calculations

	encoder_constant = 4 * encoder_PPR * reducer_reduction_ratio [pulses per reducer shaft revolution]

	reducer_reduction_ration = 1 if reduces isn't used

								 encoder_counter_value
	reducer_shaft_revolutions = -----------------------
								   encoder_constant

	where 4 is used because of the algorithm
 */
typedef struct{

	position_control *position_controller;
	speed_control *speed_controller;

	void (*motor_enable)(void);
	void (*motor_disable)(void);
	uint32_t (*set_pwm_duty_cycle)(const int32_t duty_cycle_coefficient);
	int16_t	(*get_encoder_counter_value)(void);

	const float encoder_constant;

	const uint32_t max_duty_cycle_coefficient;
	uint32_t currentPwmWidth;

} motor;


uint32_t encoderSetupTest (motor *m);

float motors_get_position(motor *motor_instance);
float motors_get_speed_by_incements(const motor *motor_instance, const float time_increment);

float position_controller_handler(motor *motor_instance); // returns speed task
float motors_speed_controller_handler(const motor *motor_instance, const float time_increment); // ideally should return current controller task (float)





#endif /* MOTORS_H_ */
