#include "icm-20600.h"

uint32_t icm_20600_basic_init(uint32_t enable_temperature_sensor)
{
// SPI based implementation
#ifdef ICM_THROUGH_SPI

	ICM_CS_LOW

	icm_spi_write(ICM_PWR_MGMT_1 | ICM_WRITE);
	if ( enable_temperature_sensor )
	{
		// Wakes up. Doen't disable temperature sensor
		icm_spi_write(ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}
	else
	{
		// Wakes up. Disables temperature sensor
		icm_spi_write(ICM_PWR_MGMT_1_TEMP_DISABLE | ICM_PWR_MGMT_1_CLKSEL_AUTO);
	}

	// Reset SPI connection
	ICM_CS_HIGH
	ICM_CS_LOW

	// Disable I2C interface
	icm_spi_write(ICM_I2C_IF | ICM_WRITE);
	icm_spi_write(ICM_I2C_IF_DISABLE);

	// Reset SPI connection
	ICM_CS_HIGH
	ICM_CS_LOW

	// Check if connection is established
	icm_spi_write(ICM_WHO_AM_I | ICM_READ);
	if ( icm_spi_write(0xff) == 0 )
	{
		ICM_CS_HIGH
		return 1;
	}

	ICM_CS_HIGH
	return 0;

#endif /* ICM_THROUGH_SPI */

// I2C based implementation
#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}

//enum icm_data_order {accelerometer_x, accelerometer_y, accelerometer_z,	gyroscope_x, gyroscope_y, gyroscope_z};

uint32_t icm_20600_get_sensors_data(int16_t * icm_data_storage_array, uint32_t add_temperature_sensor_data)
{

// SPI based implementation
#ifdef ICM_THROUGH_SPI

	ICM_CS_LOW

	// Start sequential reading of sensor data
	icm_spi_write(ICM_ACCEL_XOUT_H | ICM_READ);

	// Get accelerometer data
	icm_data_storage_array[icm_accelerometer_x] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);
	icm_data_storage_array[icm_accelerometer_y] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);
	icm_data_storage_array[icm_accelerometer_z] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);

	if (add_temperature_sensor_data)
	{
		// Add temperature sensor data to array
		icm_data_storage_array[icm_temperature] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);
	}
	else
	{
		// Skip temperature sensor data
		icm_spi_write(0xFF);
		icm_spi_write(0xFF);
	}

	// Get gyroscope data
	icm_data_storage_array[icm_gyroscope_x] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);
	icm_data_storage_array[icm_gyroscope_y] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);
	icm_data_storage_array[icm_gyroscope_z] = icm_spi_write(0xFF)<<8 | icm_spi_write(0xFF);

	ICM_CS_HIGH

	return 0;

#endif /* ICM_THROUGH_SPI */

// I2C based implementation
#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}


uint32_t icm_20600_check_if_alive(void)
{
#ifdef ICM_THROUGH_SPI

	ICM_CS_LOW

	// Check if device is connected
	icm_spi_write(ICM_WHO_AM_I | ICM_READ);
	if ( icm_spi_write(0xff) == 0 )
	{
		ICM_CS_HIGH
		return 1;
	}

	ICM_CS_HIGH
	return 0;

#endif /* ICM_THROUGH_SPI */

#ifdef ICM_THROUGH_I2C

	#error I2C is not implemented yet

#endif /* ICM_THROUGH_I2C */
}

