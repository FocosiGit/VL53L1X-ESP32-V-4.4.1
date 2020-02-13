#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "soc/gpio_struct.h"

#include "vl53l1_platform.h"
#include "vl53l1_platform_user_data.h"
#include "vl53l1_api.h"


/*
 * Wiring :  connect SCL cable in  GPIO 15 (D15)
 * 			 connect SDA cable in  GPIO 16 (RX2)
 */
#define I2C_MASTER_SCL_IO               15
#define I2C_MASTER_SDA_IO               16
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

VL53L1_Dev_t 	dev;
VL53L1_DEV 		Dev 		= &dev;

void			f_Operating_error(VL53L1_Error 	Status, char *Msg)
	{
	printf("Error %s, status=%d \n", Msg, Status);
	printf("Sensor problem. Boot in 10 seconds. \n");
	vTaskDelay(10000 / portTICK_PERIOD_MS);
	esp_restart();
	}

char Lastmsg[100] = "";
void			f_Reading_error(VL53L1_Error 	Status, char *Msg)
	{
	if (! strcmp (Lastmsg, Msg) ) return;
	strcpy(Lastmsg, Msg);
	printf("Error %s, status=%d \n", Msg, Status);
	}

static void 	f_i2c_init(void)
{
    i2c_config_t 	conf;
	int				Status;

    int i2c_master_port 	= I2C_MASTER_NUM;

    conf.mode 				= I2C_MODE_MASTER;

    conf.sda_io_num 		= I2C_MASTER_SDA_IO;
    conf.sda_pullup_en 		= GPIO_PULLUP_ENABLE;

    conf.scl_io_num 		= I2C_MASTER_SCL_IO;
    conf.scl_pullup_en 		= GPIO_PULLUP_ENABLE;

    conf.master.clk_speed 	= I2C_MASTER_FREQ_HZ;

    Status = i2c_param_config(i2c_master_port, &conf);
    if (Status != ESP_OK)	f_Operating_error(Status, "i2c_param_config");

    printf("i2c_param_config - Ok. \n");


    Status = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (Status != ESP_OK)	f_Operating_error(Status, "i2c_driver_install");

	printf("i2c_driver_install - Ok. \n");

}


VL53L1_Error 	f_vl53l1_init(VL53L1_DEV 	Dev)
{
 	VL53L1_Error 	Status 	 		= VL53L1_ERROR_NONE;
    				Dev->I2cHandle 	= I2C_MASTER_NUM;
    				Dev->I2cDevAddr = 0x52;

	Status = VL53L1_WaitDeviceBooted(Dev);
	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_WaitDeviceBooted");

 	Status = VL53L1_DataInit(Dev);
 	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_DataInit");

 	Status = VL53L1_StaticInit(Dev);
 	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_StaticInit");

// 	printf("VL53L1_SetDistanceMode Inicio\n");
// 	VL53L1_SetDistancexMode(Dev, VL53L1_DISTANCEMODE_LONG);

 	Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
 	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_SetMeasurementTimingBudgetMicroSeconds");

 	Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
 	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_SetInterMeasurementPeriodMilliSeconds");

 	Status = VL53L1_StartMeasurement(Dev);
 	if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "VL53L1_StartMeasurement");

 	return(Status);
}
int f_read_sensor()
{
 	VL53L1_Error 						Status	= 999;
 	VL53L1_RangingMeasurementData_t		RangingData;
 	uint8_t 							pDataReady;
 	int									retry = 0;

 	while (Status != VL53L1_ERROR_NONE)
 		{
 	 	Status = VL53L1_GetMeasurementDataReady( Dev , &pDataReady);
 		if (Status != VL53L1_ERROR_NONE)
 			{
 			f_Reading_error(Status, "VL53L1_GetMeasurementDataReady");
 			continue;
 			}

 	 	Status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
 		if (Status != VL53L1_ERROR_NONE)
 			{
 			f_Reading_error(Status, "VL53L1_GetRangingMeasurementData");
 			continue;
 			}

 		Status = RangingData.RangeStatus;
 		if (Status != VL53L1_ERROR_NONE)
 			{
 			f_Reading_error(Status, "VL53L1_GetRangingMeasurementData - RangeStatus");
 			}

 		retry++;
 		if (retry > 100)	return(-1);
 		}

	Status = VL53L1_ClearInterruptAndStartMeasurement( Dev );
	if (Status != VL53L1_ERROR_NONE)	f_Reading_error(Status, "VL53L1_ClearInterruptAndStartMeasurement");

 	memset(Lastmsg,0,sizeof(Lastmsg));

	return (RangingData.RangeMilliMeter);
}

void app_main(void)
{
    int 					Blink = 0, Stat;
 	VL53L1_Error 			Status;


    nvs_flash_init();
    f_i2c_init();

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    Status = f_vl53l1_init(Dev);
    if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "vl53l1_init");

    printf("Init sensor Ok. \n");
    while (	1 )
		{
		vTaskDelay(2000 / portTICK_PERIOD_MS);				// Aguardar antes do próximo teste

		Blink = !Blink;
		gpio_set_level(GPIO_NUM_2, Blink);

		Stat = f_read_sensor();
		if (Stat == -1)
			{
		    Status = f_vl53l1_init(Dev);
		    if (Status != VL53L1_ERROR_NONE)	f_Operating_error(Status, "vl53l1_init");
			}
		else
			{
			printf("Distance =%d mm\n", Stat);
			}
		}
}

