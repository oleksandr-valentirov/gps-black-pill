/* includes */
#include "bq25895.h"
#include "pmu_reg.h"

/* defines */
#define ADDRESS 0x6A

/* macros */

/* variables */
static I2C_TypeDef *i2c;
volatile uint8_t pmu_int_flag = 0;

/* function prototypes */
static sys_status poll_tx(uint8_t registerAddress, uint8_t data);
static sys_status poll_rx(uint8_t registerAddress, uint8_t *dest);


sys_status BQ25895_Init(I2C_TypeDef *I2Cx) {
    if (!I2Cx)
        return NULL_ARG_ERR;

    return STATUS_OK;
}

void BQ25895_ProcessInterrupt(void) {

}

static sys_status poll_tx(uint8_t registerAddress, uint8_t data) {
    LL_I2C_GenerateStartCondition(i2c);

    /* wait for Start Bit */
    while (!LL_I2C_IsActiveFlag_SB(i2c));

    /* send addr with write cmd */
    LL_I2C_TransmitData8(i2c, ADDRESS & 0xFE);

    /* wait for ADDR (Address Acknowledgement) */
    while (!LL_I2C_IsActiveFlag_ADDR(i2c));
    LL_I2C_ClearFlag_ADDR(i2c);

    /* send reg addr */
    LL_I2C_TransmitData8(i2c, registerAddress);

    /* wait for transfer complete */
    while (!(LL_I2C_IsActiveFlag_TXE(i2c) || LL_I2C_IsActiveFlag_BTF(i2c)));

    /* send data */
    LL_I2C_TransmitData8(i2c, data);

    /* wait for transfer complete */
    while (!(LL_I2C_IsActiveFlag_TXE(i2c) || LL_I2C_IsActiveFlag_BTF(i2c)));

    /* stop */
    LL_I2C_GenerateStopCondition(i2c);

    return STATUS_OK;
}

static sys_status poll_rx(uint8_t registerAddress, uint8_t *dest) {
    LL_I2C_GenerateStartCondition(i2c);

    /* wait for Start Bit */
    while (!LL_I2C_IsActiveFlag_SB(i2c));

    /* send addr with write cmd */
    LL_I2C_TransmitData8(i2c, ADDRESS & 0xFE);

    /* wait for ADDR (Address Acknowledgement) */
    while (!LL_I2C_IsActiveFlag_ADDR(i2c));
    LL_I2C_ClearFlag_ADDR(i2c);

    /* send reg addr */
    LL_I2C_TransmitData8(i2c, registerAddress);

    /* wait for transfer complete */
    while (!(LL_I2C_IsActiveFlag_TXE(i2c) || LL_I2C_IsActiveFlag_BTF(i2c)));

    /* start for reading */
    LL_I2C_GenerateStartCondition(i2c);

    /* wait for Start Bit */
    while (!LL_I2C_IsActiveFlag_SB(i2c));

    /* send addr with read cmd */
    LL_I2C_TransmitData8(i2c, ADDRESS | 0x01);

    /* wait for ADDR (Address Acknowledgement) */
    while (!LL_I2C_IsActiveFlag_ADDR(i2c));
    LL_I2C_ClearFlag_ADDR(i2c);

    /* disable ack before reading the last byte */
    LL_I2C_AcknowledgeNextData(i2c, LL_I2C_NACK);

    /* wait for transfer complete */
    while (!LL_I2C_IsActiveFlag_RXNE(i2c));

    /* fetch data */
    *dest = LL_I2C_ReceiveData8(i2c);

    /* stop */
    LL_I2C_GenerateStopCondition(i2c);

    return STATUS_OK;
}
