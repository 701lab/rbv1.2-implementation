#ifndef ICM_20600_H_
#define ICM_20600_H_

#include "implementation.h"
#include "icm-20600_registers.h"


//****** ICM functions ******//

uint32_t icm_20600_basic_init(uint32_t enable_temperature_sensor);

uint32_t icm_20600_get_sensors_data(int16_t * icm_data_storage_array, uint32_t add_temperature_sensor_data);

// @brief send basic read message onto register with enable bit. If answer is 0 there is problems with connection to the device, or device didn't start.
uint32_t icm_20600_check_if_alive(void);
// Можно сделать вызов этой функции при инициализации, если с первого раза устройство не обнаружено

void icm_20600_registers_reset(void);

void icm_20600_disable_gyro(void);

void icm_20600_disable_accel(void);

void icm_20600_disable_one_gyro_channel(uint32_t number_of_channel_to_desable);

void icm_20600_disable_one_accel_channel(uint32_t number_of_channel_to_desable);


//****** ICM enums ******//

// @brief Used for icm-20600 data pointing in arrays
enum icm_data_order {icm_accelerometer_x, icm_accelerometer_y, icm_accelerometer_z,	icm_gyroscope_x, icm_gyroscope_y, icm_gyroscope_z, icm_temperature};


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
