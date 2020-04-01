
#include "motors.h"


float motors_get_position(motor *motor_instance)
{
	return 0;
}



float motors_get_speed_by_incements(const motor *motor_instance, const float time_increment)
{
	int16_t current_encoder_couter_value = motor_instance->get_encoder_counter_value();
	int16_t encoder_counter_increment =  motor_instance->get_encoder_counter_value() - motor_instance->speed_controller->previous_encoder_counter_value;

	if (encoder_counter_increment == 0)
	{
		motor_instance->speed_controller->current_speed = 0.0f;
		return 0.0f;
	}

	motor_instance->speed_controller->current_speed = (float)(encoder_counter_increment) / (motor_instance->encoder_constant * time_increment);
	motor_instance->speed_controller->previous_encoder_counter_value = current_encoder_couter_value;

	return motor_instance->speed_controller->current_speed;
}

float motors_speed_controller_handler(const motor *motor_instance, const float time_increment)
{
	float current_speed_mistake = motor_instance->speed_controller->target_speed - motor_instance->speed_controller->current_speed;
	float speed_limitation = motor_instance->speed_controller->controller_output_limitation_value;

	// At that point contain previous control signal
	float control_signal = motor_instance->speed_controller->regulator_control_signal;

	if ( !( (control_signal > speed_limitation && current_speed_mistake >= 0)
				|| (control_signal < -speed_limitation && current_speed_mistake < 0) ) )
	{
		motor_instance->speed_controller->current_integral += (current_speed_mistake + motor_instance->speed_controller->previous_speed_mistake) / 2.0f * time_increment;
	}

	motor_instance->speed_controller->previous_speed_mistake = current_speed_mistake;

	control_signal = (motor_instance->speed_controller->kp * current_speed_mistake)
				+ (motor_instance->speed_controller->ki * motor_instance->speed_controller->current_integral);

	motor_instance->speed_controller->regulator_control_signal = control_signal;

	if ( control_signal > speed_limitation )
	{
		return speed_limitation;
	}
	else if ( control_signal < -speed_limitation )
	{
		return -speed_limitation;
	}
	else
	{
		return control_signal;
	}
}



/****** Right motor encoder connection test ******
 * Takes pointer to motor instance.  Tests if expected forward direction of motor shaft rotation is counted up by encoder timer. If true - return 1 else 0.
 *
 * Stops motor, zeros encoder counter -> Starts motor and waits for some time -> if encoder value were decrementing return 0.
 *
 * Decrement of encoder value doesn't show rotation direction of the motor. It only shows if  encoder setup matches chosen forward rotation direction/
 * If it doesn't encoder can be inversed by TIMx->CCER register.*/
//uint32_t encoderSetupTest (motor *m){
//
//	DISABLE_MOTOR((*m)) 	// Stop motor, so zeroing encoder will have sense
//
//	*m->encoderCounterRegisterAddress = 0;		// set encoder data to 0 before test
//
//	setPwmWidth(m, m->pwmMaxWidth/2);	// starts motor with half of the maximum speed
//	ENABLE_MOTOR((*m))
//
//	for(int i = 0; i < 100000; ++i);
//
//	DISABLE_MOTOR((*m))
//
//	if(((int16_t)*(m->encoderCounterRegisterAddress)) < 0){	// Register that contains timer counter value is 32 bit, but counter itself is 16,
//														// so we need to make type conversion to check real timer value.
//														// In this situation int16_t conversion will make all values bigger then 2^15 negative, which can happen if counter decrements
//		return 0; 	// Mistake occured
//		//		mistakeState = 1;
//		//		addMistakeToLog(ENCODER_WRONG_COUTING_DIRECTION); - probably should do mistake writing in the main code and not here
//	}
//
//	return 1; 		// Test passed
//}
//

//
//
////*** Get specified motor speed ***//
///* Takes pointer to motor instance. Returns position in revolutions */
//
//float getSpeed(motor *m, float timeVar)
//{
//	int16_t currentEncoderCounterValue = *(m->encoderCounterRegisterAddress);
//
//	m->speedController->currentSpeed = (int16_t)(currentEncoderCounterValue - m->speedController->previousEncoderCounterValue)/(m->encoderScaleConstant*timeVar);
//	m->speedController->previousEncoderCounterValue = currentEncoderCounterValue;
//
//	return m->speedController->currentSpeed;
//}
//
//
////****** Speed controller calculation function ******//
///* Takes pointer to motor control instance, time interval value and current speed value. Returns speed PI-regulator (with wind-up control) signal.
// *
// */
//
//float speedControllerHandler(motor *m, float timeVar)
//{
//	float currentSpeedMistake = m->speedController->targetSpeed - m->speedController->currentSpeed;
//	float speedLimitation = m->speedController->controllerOutputLimitationValue;
//	float controlSignal = m->speedController->regulatorControlSignal; 		// At that poin contains previous control signal value
//
//	if (!((controlSignal > speedLimitation && currentSpeedMistake >= 0) || (controlSignal < -speedLimitation && currentSpeedMistake < 0)))
//	{			// control signal is normal, so we need to calculate integral value (wind-up control method)
//		m->speedController->integral += (currentSpeedMistake + m->speedController->previousSpeedMistake)/2.0f*timeVar;
//	}
//
//	m->speedController->previousSpeedMistake = currentSpeedMistake; // for trapezoidal integration
//
//	controlSignal = (m->speedController->Kp * currentSpeedMistake) + (m->speedController->Ki * m->speedController->integral);
//	m->speedController->regulatorControlSignal = controlSignal;
//
//	if (controlSignal > speedLimitation)
//	{
//		return speedLimitation;
//	}
//	else if (controlSignal < -speedLimitation)
//	{
//		return -speedLimitation;
//	}
//	else
//	{
//		return controlSignal;
//	}
//}
//
////*** Get specified motor position ***//
///* Takes pointer to motor instance. Returns position in revolutions */
//
//float getPosition(motor *m)
//{
//	int16_t currentEncoderCounterValue = *(m->encoderCounterRegisterAddress);
//
//	m->positionController->currentPosition += (int16_t)(currentEncoderCounterValue - m->positionController->previousEncoderCounterValue)/(m->encoderScaleConstant); // Revolutions
//	m->positionController->previousEncoderCounterValue = currentEncoderCounterValue;
//
//	return m->positionController->currentPosition;
//}
//
//
//float positionController(motor *m)
//{
//	float currentPositionMistake = m->positionController->targetPosition - m->positionController->currentPosition;
//	m->positionController->regulatorControlSignal = m->positionController->Kp * currentPositionMistake;
//
//	return m->positionController->regulatorControlSignal;
//}
//
//




//
//
//
//int getPosition(motor *m,  float timeVar)
//{
//	int16_t currentCounterValue = *(m->encoderRegisterAddress);
//	if(m->isReversed){
//		m->positionController->currentPosition -= (currentCounterValue + m->positionController->previousEncoderTicks);
//		m->positionController->previousEncoderTicks = -1*currentCounterValue;
//	}
//	else{
//		m->positionController->currentPosition += (currentCounterValue - m->positionController->previousEncoderTicks);
//		m->positionController->previousEncoderTicks = currentCounterValue;
//	}
//
//	return m->positionController->currentPosition;
//}
//
//
//float positionController(motor *m, float timeVar, float currentPosition)
//{
//	float mistake = (float)((int32_t)(m->positionController->targetPosition - m->positionController->currentPosition))/1000; 	// deviding by 1000 becouse counting in encoder ticks, so we will have huge numbers in tasks
//	if (!(m->positionController->regControlSignal > m->maxSpeed && m->positionController->currentPosition < m->positionController->targetPosition) ||
//	(m->positionController->regControlSignal < -m->maxSpeed && m->positionController->currentPosition > m->positionController->targetPosition)){
//		m->positionController->integral += mistake*timeVar/1000;
//	}
//	m->positionController->regControlSignal = m->positionController->Kp*mistake + m->positionController->Ki*m->positionController->integral;
//
//	return m->positionController->regControlSignal;
//}



