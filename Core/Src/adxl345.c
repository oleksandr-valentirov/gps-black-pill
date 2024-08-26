/* includes */
#include "adxl345.h"

/* defines */
#define PACKET_SIZE     6
#define BUFF_LENGTH     32
#define SCALE           39  /* 3.9 for 2g * 10 */
#define TIMEOUT_MS      2000
#define DMA_DATA_LENGTH (BUFF_LENGTH * PACKET_SIZE)

/* macros */
#define CS_PIN_LOW(port, pin)    LL_GPIO_ResetOutputPin(port, pin)
#define CS_PIN_HIGH(port, pin)   LL_GPIO_SetOutputPin(port, pin)

/* variables */
static uint16_t fifo[PACKET_SIZE];
uint32_t acc_data[32][4];
uint8_t acc_int_index = 0;
volatile uint8_t acc_int_flag = 0;
static SPI_TypeDef *acc_spi = NULL;
static DMA_TypeDef *dma = NULL;
static uint32_t stream = 0xFFFFFFFF;
static GPIO_TypeDef *cs_port = NULL;
static uint32_t cs_pin = 0xFFFFFFFF;

/* data structures */

/* function prototypes */
static adxl345_status poll_tx(uint8_t addr, uint8_t data);
static adxl345_status poll_rx(uint8_t addr, uint8_t length, uint8_t *dest);
static inline uint32_t convert_data(uint16_t data);


adxl345_status ADXL345_Init(DMA_TypeDef *DMAx, uint32_t streamx, SPI_TypeDef *SPIx, GPIO_TypeDef *cs_portx, uint32_t cs_pinx) {
    adxl345_status status = ADXL345_OK;

    if (!SPIx || !DMAx || !cs_portx)
        return ADXL345_INIT_ERROR;
    acc_spi = SPIx;
    dma = DMAx;
    cs_port = cs_portx;
    stream = streamx;
    cs_pin = cs_pinx;

    /* config data format - set +-4g */
    if ((status = poll_tx(ADXL345_DATA_FORMAT, 0x01)))
        return status;
    /* config rate - set 1600 Hz */
    if ((status = poll_tx(ADXL345_BW_RATE, 0x0E)))
        return status;
    /* config fifo - set FIFO mode */
    if ((status = poll_tx(ADXL345_FIFO_CTL, 0x01)))
        return status;

    LL_DMA_SetMemoryAddress(dma, stream, (uint32_t)(&(fifo[0])));
    LL_DMA_SetPeriphAddress(dma, stream, LL_SPI_DMA_GetRegAddr(acc_spi));
    LL_DMA_SetDataLength(dma, stream, PACKET_SIZE);
    LL_DMA_EnableIT_TC(dma, stream);
    LL_DMA_EnableStream(dma, stream);
    LL_SPI_EnableDMAReq_RX(acc_spi);

    /* enable interrupts before return */
    if ((status = poll_tx(ADXL345_INT_ENABLE, ADXL345_INT_DATA_READY)))
        return status;

    return ADXL345_OK;
}

adxl345_status adxl345_ProcessInterrupt(void) {
    static uint8_t ring_buf_i = 0;  /* it is needed to track some number of latest FIFOs */
    uint32_t cur_time = 0, start_time = 0;

    acc_int_flag = 0;
    
    /* maybe it will be too short and it will be nessesary to bring it to the inrerrupt */
    CS_PIN_HIGH(cs_port, cs_pin);

    if (!acc_int_index) {
        /* disable GPIO interrupt */
        NVIC_DisableIRQ(EXTI1_IRQn);
        /* clear previous result */
        acc_data[ring_buf_i][1] = 0;
        acc_data[ring_buf_i][2] = 0;
        acc_data[ring_buf_i][3] = 0;
    } else {
        /* add new data to the accumulator */
        acc_data[ring_buf_i][1] += ((fifo[0] << 8) | fifo[1]);  /* ax */
        acc_data[ring_buf_i][2] += ((fifo[2] << 8) | fifo[3]);  /* ay */
        acc_data[ring_buf_i][3] += ((fifo[4] << 8) | fifo[5]);  /* az */
    }

    acc_int_index++;
    acc_int_index &= (BUFF_LENGTH - 1);
    if (!acc_int_index) {
        /* calc average & enable GPIO interrupt */

        /* ax */
        acc_data[ring_buf_i][1] /= BUFF_LENGTH;
        acc_data[ring_buf_i][1] = convert_data((uint16_t)acc_data[ring_buf_i][1]);
        /* ay */
        acc_data[ring_buf_i][2] /= BUFF_LENGTH;
        acc_data[ring_buf_i][2] = convert_data((uint16_t)acc_data[ring_buf_i][2]);
        /* az */
        acc_data[ring_buf_i][3] /= BUFF_LENGTH;
        acc_data[ring_buf_i][3] = convert_data((uint16_t)acc_data[ring_buf_i][3]);
        
        ring_buf_i++;
        ring_buf_i &= (BUFF_LENGTH - 1);
        NVIC_EnableIRQ(EXTI1_IRQn);
    } else {
        /* start next FIFO read sequence */
        CS_PIN_LOW(cs_port, cs_pin);
        /* check busy */
        while (!LL_SPI_IsActiveFlag_TXE(acc_spi) || LL_SPI_IsActiveFlag_BSY(acc_spi)) {
            cur_time = HAL_GetTick();
            if (cur_time - start_time >= TIMEOUT_MS)
                return ADXL345_TIMEOUT;
        }
        /* transmit addr */
        LL_SPI_TransmitData8(acc_spi, ADXL345_DATAX0 | 0x80 | 0x40);
    }

    return ADXL345_OK;
}

adxl345_status ADXL345_ProcessData(void) {
    return ADXL345_OK;
}

static adxl345_status poll_tx(uint8_t addr, uint8_t data) {
    uint32_t cur_time = 0, start_time = 0;

    CS_PIN_LOW(cs_port, cs_pin);
    while (!LL_SPI_IsActiveFlag_TXE(acc_spi) || LL_SPI_IsActiveFlag_BSY(acc_spi)) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= TIMEOUT_MS)
            return ADXL345_TIMEOUT;
    }
    LL_SPI_TransmitData8(acc_spi, addr);
    while (!LL_SPI_IsActiveFlag_TXE(acc_spi) || LL_SPI_IsActiveFlag_BSY(acc_spi)) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= TIMEOUT_MS)
            return ADXL345_TIMEOUT;
    }
    LL_SPI_TransmitData8(acc_spi, data);
    while (!LL_SPI_IsActiveFlag_TXE(acc_spi) || LL_SPI_IsActiveFlag_BSY(acc_spi)) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= TIMEOUT_MS)
            return ADXL345_TIMEOUT;
    }
    CS_PIN_HIGH(cs_port, cs_pin);

    return ADXL345_OK;
}

static adxl345_status poll_rx(uint8_t addr, uint8_t length, uint8_t *dest) {
    uint32_t cur_time = 0, start_time = 0;

    if (!length)
        return ADXL345_OK;

    CS_PIN_LOW(cs_port, cs_pin);

    /* transmit addr */
    while (!LL_SPI_IsActiveFlag_TXE(acc_spi) || LL_SPI_IsActiveFlag_BSY(acc_spi)) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= TIMEOUT_MS)
            return ADXL345_TIMEOUT;
    }
    LL_SPI_TransmitData8(acc_spi, addr | (length > 1 ? (0x80 | 0x40) : 0x80));  /* set read bits */

    for (uint8_t i = 0; i < length; i++) {
        while (!LL_SPI_IsActiveFlag_RXNE(acc_spi)) {
            cur_time = HAL_GetTick();
            if (cur_time - start_time >= TIMEOUT_MS)
                return ADXL345_TIMEOUT;
        }
        *(dest++) = LL_SPI_ReceiveData8(SPI1);
    }
    while (!LL_SPI_IsActiveFlag_RXNE(acc_spi)) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= TIMEOUT_MS)
            return ADXL345_TIMEOUT;
    }

    CS_PIN_HIGH(cs_port, cs_pin);

    return ADXL345_OK;
}

/*
 * @return: g * 10^4
 */
static inline uint32_t convert_data(uint16_t data) {
    return data * SCALE;
}
