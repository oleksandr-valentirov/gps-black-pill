#pragma once

/*
 * ADXL345 accelerometer header file
 * https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
 */

#include "main.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"


/* Redister map */
#define ADXL345_DEVID           0x00    /* device ID - 0b11100101 */
/* tap */
#define ADXL345_TAP_THRESH      0x1D
#define ADXL345_TAP_DUR         0x21    /* tap duration */
#define ADXL345_TAP_LATENCY     0x22
#define ADXL345_TAP_WINDOW      0x23
#define ADXL345_TAP_AXIS        0x2A    /* Axis control for single tap/double tap */
#define ADXL345_TAP_STATUS      0x2B    /* Source of single tap/double tap */
/* offsets */
#define ADXL345_OFSX            0x1E    /* x-axis offset */
#define ADXL345_OFSY            0x1F    /* y-axis offset */
#define ADXL345_OFSZ            0x20    /* z-axis offset */
/* activity */
#define ADXL345_ACT_THRESH      0x24    /* activity threshold  */
#define ADXL345_INACT_THRESH    0x25    /* inacivity threshold */
#define ADXL345_INCAT_TIME      0x26
#define ADXL345_ACT_INACT_CTL   0x27    /* Axis enable control for activity and inactivity detection */
/* free fall */
#define ADXL345_FF_THRESH       0x28
#define ADXL345_FF_TIME         0x29
/* data settings */
#define ADXL345_BW_RATE         0x2C    /* Data rate and power mode control - 0b00001010 */
#define ADXL345_POWER_CTL       0x2D    /* Power-saving features control */
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39
/* interrupts */
#define ADXL345_INT_MAP         0x2F
#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_INT_SOURCE      0x30    /* 0b00000010 */
/* interrupt masks */
#define ADXL345_INT_DATA_READY  0x80
#define ADXL345_INT_SINGLE_TAP  0x40
#define ADXL345_INT_DOUBLE_TAP  0x20
#define ADXL345_INT_ACTIVITY    0x10
#define ADXL345_INT_INACTIVITY  0x08
#define ADXL345_INT_FREE_FALL   0x04
#define ADXL345_INT_WATERMARK   0x02
#define ADXL345_INT_OVERRUN     0x01

typedef enum adxl345_status {
    ADXL345_OK = 0,
    ADXL345_TIMEOUT,
} adxl345_status;

sys_status ADXL345_Init(DMA_TypeDef *DMAx, uint32_t streamx, SPI_TypeDef *SPIx, GPIO_TypeDef *cs_portx, uint32_t cs_pinx);
sys_status adxl345_ProcessInterrupt(void);

extern volatile uint8_t acc_int_flag;
