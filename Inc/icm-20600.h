#ifndef ICM_20600_H_
#define ICM_20600_H_

#include "implementation.h"
#include "icm-20600_registers.h"



typedef struct{
	const void (*cs_high)(void);
	const void (*cs_low)(void);
	const uint8_t (*send_one_byte)(const uint8_t byte_to_be_sent);

} icm_20600_instance;

//icm_20600_instance icm_20600_instance_default = { 0, 0, 0 };

//****** ICM functions ******//

uint32_t icm_20600_basic_init(const icm_20600_instance *icm_instance,
			uint32_t enable_temperature_sensor);

uint32_t icm_20600_get_sensors_data(const icm_20600_instance *icm_instance,
			int16_t *icm_data_storage_array,
			uint32_t add_temperature_sensor_data);

// @brief send basic read message onto register with enable bit. If answer is 0 there is problems with connection to the device, or device didn't start.
uint32_t icm_20600_check_if_alive(const icm_20600_instance * icm_instance);
// Можно сделать вызов этой функции при инициализации, если с первого раза устройство не обнаружено



void icm_20600_registers_reset(icm_20600_instance *icm_instance);

void icm_20600_disable_gyro(icm_20600_instance *icm_instance);

void icm_20600_disable_accel(icm_20600_instance *icm_instance);

void icm_20600_disable_one_gyro_channel(icm_20600_instance *icm_instance, uint32_t number_of_channel_to_desable);

void icm_20600_disable_one_accel_channel(icm_20600_instance *icm_instance, uint32_t number_of_channel_to_desable);


void icm_20600_setup(const icm_20600_instance *icm_instance, const uint8_t gyro_desired_dps_scale, const uint8_t accel_desired_g_scale);

//****** ICM enums ******//

// @brief Used for icm-20600 data pointing in arrays
enum icm_data_order
{
	icm_accelerometer_x,
	icm_accelerometer_y,
	icm_accelerometer_z,
	icm_gyroscope_x,
	icm_gyroscope_y,
	icm_gyroscope_z,
	icm_temperature
};


// @brief Gyroscope configuration parameters
enum icm_gyro_config_params
{
	icm_gyro_250dps,
	icm_gyro_500dps,
	icm_gyro_1000dps,
	icm_gyro_2000dps
};

// @brief Gyroscope configuration parameters
enum icm_accel_config_params
{
	icm_accel_2g,
	icm_accel_4g,
	icm_accel_8g,
	icm_accel_16g
};


//****** Implementation checks ******//

#ifndef ICM_THROUGH_SPI
	#ifndef ICM_THROUGH_I2C
		#error	ICM preferable interface is not defined

	#endif /* ICM_THROUGH_I2C */
#endif /* ICM_THROUGH_SPI */

#ifdef ICM_THROUGH_SPI
	#ifdef ICM_THROUGH_I2C
		#error I2C and SPI can not be used at the same time

	#endif /* ICM_THROUGH_I2C */
#endif /* ICM_THROUGH_SPI */

#endif /* ICM_20600_H_ */
