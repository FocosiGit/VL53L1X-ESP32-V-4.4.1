/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_data.h"
//#include "vl53l1_api.h"

//#define I2C_PORT                        I2C_NUM_1
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1


int				_I2CWrite(VL53L1_DEV Dev, uint8_t *buf, uint32_t len);
int				_I2CRead(VL53L1_DEV Dev,  uint8_t *buf, uint32_t len);

void			VL53L1_GetI2cBus(void);
void			VL53L1_PutI2cBus(void);

VL53L1_Error	VL53L1_WriteMulti	(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
// the ranging_sensor_comms.dll will take care of the page selection
VL53L1_Error	VL53L1_ReadMulti	(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error	VL53L1_WrByte		(VL53L1_DEV Dev, uint16_t index, uint8_t data);
VL53L1_Error	VL53L1_WrWord		(VL53L1_DEV Dev, uint16_t index, uint16_t data);
VL53L1_Error	VL53L1_WrDWord		(VL53L1_DEV Dev, uint16_t index, uint32_t data);
VL53L1_Error	VL53L1_UpdateByte	(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData);
VL53L1_Error	VL53L1_RdByte		(VL53L1_DEV Dev, uint16_t index, uint8_t *data);
VL53L1_Error	VL53L1_RdWord		(VL53L1_DEV Dev, uint16_t index, uint16_t *data);
VL53L1_Error	VL53L1_RdDWord		(VL53L1_DEV Dev, uint16_t index, uint32_t *data);
VL53L1_Error	VL53L1_GetTickCount	(uint32_t *ptick_count_ms);
VL53L1_Error	VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz);
VL53L1_Error	VL53L1_WaitMs		(VL53L1_Dev_t *pdev, int32_t wait_ms);
VL53L1_Error	VL53L1_WaitUs		(VL53L1_Dev_t *pdev, int32_t wait_us);
VL53L1_Error	VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t  index, uint8_t value, uint8_t mask, uint32_t  poll_delay_ms);
