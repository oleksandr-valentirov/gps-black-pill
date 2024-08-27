#pragma once

/*
 * TI battery charge controller
 * https://www.ti.com/lit/ds/symlink/bq25895.pdf?ts=1715472339780
 */

#include "main.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_gpio.h"

void BQ25895_ProcessInterrupt(void);
sys_status BQ25895_Init(I2C_TypeDef *I2Cx);

extern volatile uint8_t pmu_int_flag;
