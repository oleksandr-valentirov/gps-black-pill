/* includes */
#include <string.h>
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "ubx.h"
#include "usbd_cdc_if.h"
#include "out_protocol.h"

/* defines */
#define BUFF_LENGTH     256
#define UBX_CRC_SIZE    2
#define UBX_SYNC_SIZE   2
#define USART_CLOCK_MHZ 96000000
#define TIMEOUT_MS      2000

#define DMA_DATA_LENGTH (\
    (6 + 84 + UBX_CRC_SIZE)  /* PVT NEO 7M */ \
    )

#if (DMA_DATA_LENGTH > BUFF_LENGTH)
#error
#endif

/* UBX class IDs */
#define ACK_CLASS_ID    0x05
#define NAV_CLASS_ID    0x01
#define CFG_CLASS_ID    0x06

/* UBX msg IDs */
#define ACK_MSG_ID      0x01
#define NACK_MSG_ID     0x00
#define NAV_PVT_ID      0x07
#define CFG_MSG_ID      0x01
#define CFG_PRT_ID      0x00
#define CFG_RATE_ID     0x08

#define PROTO_UBX       1
#define PROTO_NMEA      2
#define PROTO_RTCM      4
#define PROTO_RTCM3     32

#define UART_CHAR_LEN   (3 << 6)
#define UART_PARITI     (4 << 9)

#define PVT_GNSS_FIX_OK 1

/* macros */
#define CALC_CRC_RANGE(len) (sizeof(ubx_header) - UBX_SYNC_SIZE + len)

/* data structures */
typedef struct ubx_header {
    uint8_t  sync1;
    uint8_t  sync2;
    uint8_t  class_id;
    uint8_t  msg_id;
    uint16_t length;
} ubx_header;

typedef struct ubx_ack_nack {
    uint8_t class_id;
    uint8_t msg_id;
} ubx_ack_nack;

typedef struct ubx_nav_pvt {
    uint32_t    itow;
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     min;
    uint8_t     sec;
    uint8_t     timeValidFlags;
    uint32_t    tAcc;               /* ns */
    int32_t     nano;               /* ns, fraction of second, -1e9..1e9*/
    uint8_t     fixType;
    uint8_t     fixFlags;
    uint8_t     flags2;             /* additional validity flags */
    uint8_t     numSv;
    int32_t     lon;                /* 1e-7, deg */
    int32_t     lat;                /* 1e-7, deg */
    int32_t     height;             /* height above elipsoid, mm */
    int32_t     hMSL;               /* height above mean sea level, mm*/
    uint32_t    hAcc;               /* mm */
    uint32_t    vAcc;
    int32_t     velN;               /* mm/s, velocity */
    int32_t     velE;
    int32_t     velD;
    int32_t     gSpeed;             /* mm/s, ground speed, 2D */
    int32_t     headMot;            /* 1e-5, deg, heading of motion, 2D */
    uint32_t    sAcc;               /* mm/s */
    uint32_t    headAcc;            /* 1e-5 deg */
    uint16_t    pDOP;
    // uint16_t    flags3;
    uint8_t     reserved[6];
    // int32_t     headVeh;            /* 1e-5 deg */
    // int16_t     magDec;
    // uint16_t    magAcc;
} ubx_nav_pvt;

typedef struct ubx_cfg_prt {
    uint8_t     portID;
    uint8_t     reserved1;
    uint16_t    txReady;
    uint32_t    mode;
    uint32_t    baudrate;
    uint16_t    inProtoMask;
    uint16_t    outProtoMask;
    uint16_t    flags;
    uint8_t     reserved2[2];
} ubx_cfg_prt;

typedef struct ubx_cfg_msg_rate {
    uint8_t class_id;
    uint8_t msg_id;
    uint8_t rate[6];
} ubx_cfg_msg_rate;

typedef struct ubx_cfg_nav_rate {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
} ubx_cfg_nav_rate;

typedef enum ubx_fix_type {
    NoFix = 0,
    DeadReckoning,
    Fix2D,
    Fix3D,
    GNSS_DeadReckoning,
    TimeOnly
} ubx_fix_type;

/* variables */
static uint8_t buffer[BUFF_LENGTH];
static USART_TypeDef *usart = NULL;

/* function prototypes */
static uint16_t ubx_calc_crc(uint32_t offset, uint16_t length);
static ubx_status poll_tx(ubx_header *header, uint8_t *data);
static ubx_status poll_rx(uint16_t length);
static ubx_status set_msg_rates(void);
static ubx_status set_ports(void);
static ubx_status set_nav_rate(void);


ubx_status UBX_Init(DMA_TypeDef *DMAx, USART_TypeDef *USARTx) {
    usart = USARTx;

    /* update */
    if (set_ports())
        return UBX_CFG_PRT_ERROR;
    
    // LL_USART_Disable(usart);
    // LL_USART_SetBaudRate(usart, USART_CLOCK_MHZ, LL_USART_OVERSAMPLING_16, 115200);
    // LL_USART_Enable(usart);

    if (set_nav_rate())
        return UBX_CFG_NAV_RATE_ERROR;

    if (set_msg_rates())
        return UBX_CFG_MSG_RATE_ERROR;

    LL_DMA_SetMemoryAddress(DMAx, LL_DMA_STREAM_2, (uint32_t)(&(buffer[0])));
    LL_DMA_SetPeriphAddress(DMAx, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USARTx));
    LL_DMA_SetDataLength(DMAx, LL_DMA_STREAM_2, DMA_DATA_LENGTH);
    LL_DMA_EnableIT_TC(DMAx, LL_DMA_STREAM_2);
    LL_DMA_EnableStream(DMAx, LL_DMA_STREAM_2);
    LL_USART_EnableDMAReq_RX(USARTx);

    return UBX_OK;
}

ubx_status UBX_ProcessData(void) {
    ubx_header *rx_header = (ubx_header *)buffer;
    ubx_nav_pvt *rx_data = (ubx_nav_pvt *)(buffer + sizeof(ubx_header));
    uint32_t cur_time = 0, start_time = 0;
    uint8_t out_buf[sizeof(out_header) + sizeof(ubx_output)] = {0};
    out_header *out_h = (out_header *)out_buf;
    ubx_output *output = (ubx_output *)(out_buf + sizeof(out_header));

    /* check all possible errors */
    if (ubx_calc_crc(2, CALC_CRC_RANGE(rx_header->length)) != *((uint16_t*)(buffer + sizeof(ubx_header) + rx_header->length)))
        return UBX_CRC_ERROR;
    if ((!(rx_data->fixFlags & PVT_GNSS_FIX_OK)) || (rx_data->fixType < Fix2D || rx_data->fixType > GNSS_DeadReckoning))
        return UBX_BadFix;

    out_h->sync1 = 71;
    out_h->sync2 = 73;
    out_h->length = sizeof(ubx_output);
    out_h->classID = 1;
    output->lon = rx_data->lon;
    output->lat = rx_data->lat;
    output->gSpeed = rx_data->gSpeed;
    output->heading = rx_data->headMot;
    output->velN = rx_data->velN;
    output->velE = rx_data->velE;
    output->nano = rx_data->nano;
    output->year = rx_data->year;
    output->month = rx_data->month;
    output->day = rx_data->day;
    output->hour = rx_data->hour;
    output->min = rx_data->min;
    output->sec = rx_data->sec;
    /* ToDo - add checksum calculation */

    start_time = HAL_GetTick();
    while (CDC_Transmit_FS(out_buf, sizeof(ubx_output) + sizeof(out_header)) != USBD_OK) {
        cur_time = HAL_GetTick();
        if (cur_time - start_time >= 100)
            return UBX_TIMEOUT;
    }

    return UBX_OK;
}

static uint16_t ubx_calc_crc(uint32_t offset, uint16_t length) {
    uint8_t a = 0, b = 0;
    uint16_t i = 0;

    for(i = 0; i < length; i++) {
        a += buffer[offset + i];
        b += a;
    }

    return (uint16_t)((((uint16_t)b) << 8) | a);
}

static ubx_status set_msg_rates(void) {
    ubx_cfg_msg_rate config[1] = {
        {.class_id = NAV_CLASS_ID, .msg_id = NAV_PVT_ID, .rate = {0, 1, 0, 0, 0, 0}}
    };
    ubx_header header = {.sync1 = 0xb5, .sync2 = 0x62, .class_id = CFG_CLASS_ID, .msg_id = CFG_MSG_ID, .length = 8};
    ubx_header *rx_header = (ubx_header *)buffer;
    ubx_ack_nack *rx_data = (ubx_ack_nack *)(buffer + sizeof(ubx_header));
    uint8_t i = 0;
    ubx_status status = UBX_OK;

    for (i = 0; i < 1; i++) {
        status = poll_tx(&header, (uint8_t *)(config + i));
        if (status)
            return status;
        status = poll_rx(sizeof(ubx_header) + sizeof(ubx_ack_nack) + UBX_CRC_SIZE);
        if (status)
            return status;
        /* check all possible errors */
        if (ubx_calc_crc(2, CALC_CRC_RANGE(rx_header->length)) != *((uint16_t*)(buffer + sizeof(ubx_header) + rx_header->length)))
            return UBX_CRC_ERROR;
        if (rx_header->class_id != ACK_CLASS_ID || rx_data->class_id != header.class_id)
            return UBX_CFG_MSG_CLASSID_ERROR;
        if (rx_data->msg_id != header.msg_id)
            return UBX_CFG_MSG_MSGID_ERROR;
        if (rx_header->msg_id == NACK_MSG_ID)
            return UBX_NACK;
    }

    return UBX_OK;
}

static ubx_status set_ports(void) {
    ubx_status status = UBX_OK;
    ubx_header *rx_header = (ubx_header *)buffer;
    ubx_ack_nack *rx_data = (ubx_ack_nack *)(buffer + sizeof(ubx_header));
    uint8_t i = 0, r = 0;
    ubx_header header = {.sync1 = 0xb5, .sync2 = 0x62, .class_id = CFG_CLASS_ID, .msg_id = CFG_PRT_ID, .length = 20};
    ubx_cfg_prt config[1] = {
        {
            .portID = 1,
            .baudrate = 9600,
            .inProtoMask = PROTO_UBX,
            .outProtoMask = PROTO_UBX,
            .txReady = 0,
            .mode = UART_CHAR_LEN | UART_PARITI,
            .flags = 0
        }
    };

    for (i = 0; i < 1; i++) {
        for (r = 5; r > 0; r--) {
            status = poll_tx(&header, (uint8_t *)(config + i));
            if (status)
                continue;
            status = poll_rx(sizeof(ubx_header) + sizeof(ubx_ack_nack) + UBX_CRC_SIZE);
            if (status)
                continue;
            /* check all possible errors */
            if (ubx_calc_crc(2, CALC_CRC_RANGE(rx_header->length)) != *((uint16_t*)(buffer + sizeof(ubx_header) + rx_header->length))) {
                status = UBX_CRC_ERROR;
                continue;
            }
            if (rx_header->class_id != ACK_CLASS_ID || rx_data->class_id != header.class_id) {
                status = UBX_CFG_MSG_CLASSID_ERROR;
                continue;
            }
            if (rx_data->msg_id != header.msg_id) {
                status = UBX_CFG_MSG_MSGID_ERROR;
                continue;
            }
            if (rx_header->msg_id == NACK_MSG_ID) {
                status = UBX_NACK;
                continue;
            }
            if (status == UBX_OK)
                break;
        }
        if (!r)
            return status;
    }

    return UBX_OK;
}

static ubx_status set_nav_rate(void) {
    ubx_status status = UBX_OK;
    ubx_header *rx_header = (ubx_header *)buffer;
    ubx_ack_nack *rx_data = (ubx_ack_nack *)(buffer + sizeof(ubx_header));
    ubx_header header = {.sync1 = 0xb5, .sync2 = 0x62, .class_id = CFG_CLASS_ID, .msg_id = CFG_RATE_ID, .length = 6};
    ubx_cfg_nav_rate config = {
        .measRate = 1000,
        .navRate = 1,
        .timeRef = 0
    };

    status = poll_tx(&header, (uint8_t *)(&config));
    if (status)
        return status;
    status = poll_rx(sizeof(ubx_header) + sizeof(ubx_ack_nack) + UBX_CRC_SIZE);
    if (status)
        return status;
    /* check all possible errors */
    if (ubx_calc_crc(2, CALC_CRC_RANGE(rx_header->length)) != *((uint16_t*)(buffer + sizeof(ubx_header) + rx_header->length)))
        return UBX_CRC_ERROR;
    if (rx_header->class_id != ACK_CLASS_ID || rx_data->class_id != header.class_id)
        return UBX_CFG_MSG_CLASSID_ERROR;
    if (rx_data->msg_id != header.msg_id)
        return UBX_CFG_MSG_MSGID_ERROR;
    if (rx_header->msg_id == NACK_MSG_ID)
        return UBX_NACK;    
    
    return UBX_OK;
}

static ubx_status poll_tx(ubx_header *header, uint8_t *data) {
    uint16_t i = 0, crc = 0;
    uint32_t cur_time = 0, start_time = 0;

    (void)memcpy(buffer, (uint8_t *)header, sizeof(ubx_header));
    (void)memcpy(buffer + sizeof(ubx_header), data, header->length);
    crc = ubx_calc_crc(2, CALC_CRC_RANGE(header->length));
    (void)memcpy(buffer + sizeof(ubx_header) + header->length, &crc, UBX_CRC_SIZE);

    for (i = 0; i < sizeof(ubx_header) + header->length + UBX_CRC_SIZE; i++) {
        /* check timeout */
        start_time = HAL_GetTick();
        while (!(LL_USART_IsActiveFlag_TC(usart) || LL_USART_IsActiveFlag_TXE(usart))) {
            cur_time = HAL_GetTick();
            if (cur_time - start_time >= TIMEOUT_MS)
                return UBX_TIMEOUT;
        }
        /* send data */
        LL_USART_TransmitData8(usart, buffer[i]);
    }

    return UBX_OK;
}

static ubx_status poll_rx(uint16_t length) {
    uint16_t i = 0;
    uint32_t cur_time = 0, start_time = 0;
    uint8_t res = 0, stage = 0, total = length + 100;

    while (length) {
        /* check timeout */
        start_time = HAL_GetTick();
        while (!(LL_USART_IsActiveFlag_RXNE(usart))) {
            cur_time = HAL_GetTick();
            if (cur_time - start_time >= TIMEOUT_MS)
                return UBX_TIMEOUT;
        }
        /* receive data */
        res = LL_USART_ReceiveData8(usart);
        if ((res == 0xb5 && stage == 0) || (res == 0x62 && stage == 1)) {
            buffer[i++] = res;
            length--;
            stage++;
        } else if (stage == 2) {
            buffer[i++] = res;
            length--;
        }
        else if (total-- == 0)
            return UBX_TIMEOUT;
    }

    return UBX_OK;
}
