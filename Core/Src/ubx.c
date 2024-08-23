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
#define NAV_DOP_ID      0x04

#define PROTO_UBX       1
#define PROTO_NMEA      2
#define PROTO_RTCM      4
#define PROTO_RTCM3     32

#define UART_CHAR_LEN   (3 << 6)
#define UART_PARITI     (4 << 9)

#define PVT_GNSS_FIX_OK 1

/* macros */
#define CALC_CRC_RANGE(len) (sizeof(ubx_header) - (UBX_SYNC_SIZE) + (len))
#define CALC_CRC_OFFSET(cur_pos, start_pos) (UBX_SYNC_SIZE + (cur_pos) - (start_pos))
#define FETCH_CRC(cur_pos, len) (sizeof(ubx_header) + (cur_pos) + (len))

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

typedef struct ubx_nav_dop
{
    uint32_t    itow;
    uint16_t    gdop;
    uint16_t    pdop;
    uint16_t    tdop;
    uint16_t    vdop;
    uint16_t    hdop;
    uint16_t    ndop;
    uint16_t    edop;
} ubx_nav_dop;

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
    ubx_nav_pvt *pvt = NULL;
    ubx_nav_dop *dop = NULL;
    uint8_t out_buf[50] = {0};

/*
 * racechrono BLE proto description
 * https://github.com/aollin/racechrono-ble-diy-device?tab=readme-ov-file
 */
    static int32_t prevDateHour = 0;
    static uint8_t rc_sync_bits = 0;
    int32_t dateHour = 0, timeFromHourStart = 0;
    int gps_bearing = 0, gps_altitude = 0, gps_speed = 0;
    double hMSL_km = 0, gSpeed_kmh = 0;

    do {
        if ((((uint8_t *)rx_header) + sizeof(ubx_header) + rx_header->length + UBX_CRC_SIZE) > (buffer + BUFF_LENGTH)) {
            /* check overflow */
            break;
        }
        if (!(rx_header->sync1 == 0xb5 && rx_header->sync2 == 0x62)) {
            /* check sync */
            rx_header = (ubx_header *)(((uint8_t *)rx_header) + 1);
            continue;
        }
        if (ubx_calc_crc(CALC_CRC_OFFSET((uint8_t *)rx_header, buffer), CALC_CRC_RANGE(rx_header->length)) != *((uint16_t*)(FETCH_CRC((uint8_t *)rx_header, rx_header->length)))) {
            /* check CRC and go to the next msg in case of error */
            rx_header = (ubx_header *)(((uint8_t *)rx_header) + sizeof(ubx_header) + rx_header->length + UBX_CRC_SIZE);
            continue;
        }

        /* check header to choose processing methond */
        if (rx_header->class_id == NAV_CLASS_ID) {
            if (rx_header->msg_id == NAV_PVT_ID) {
                pvt = (ubx_nav_pvt *)(((uint8_t *)rx_header) + sizeof(ubx_header));
#if 0
                /* check validity */
                if ((!(pvt->fixFlags & PVT_GNSS_FIX_OK)) || (pvt->fixType > Fix3D))
                    continue;
#endif

                dateHour = (pvt->year - 2000) * 8928 + (pvt->month - 1) * 744 + (pvt->day - 1) * 24 + pvt->hour;
                timeFromHourStart = pvt->min * 30000 + pvt->sec * 500 + (pvt->nano / 1000000.0) / 2;

                gps_bearing = (pvt->headMot > 0) ? (int)((pvt->headMot + 500) / 1000) : 0;
                
                hMSL_km = pvt->hMSL * 1e-3;
                if (hMSL_km > 6.0) {
                    gps_altitude = (((int)(hMSL_km + 0.5)) & 0x7FFF) | 0x8000;
                } else {
                    gps_altitude = (((int)((hMSL_km + 0.5) * 10)) & 0x7FFF);
                }

                gSpeed_kmh = pvt->gSpeed * 0.0036;
                if (gSpeed_kmh > 600.0) {
                    gps_speed = (((int)(gSpeed_kmh * 10)) & 0x7FFF) | 0x8000;
                } else {
                    gps_speed = (((int)(gSpeed_kmh * 100)) & 0x7FFF);
                }

                out_buf[0] = ((rc_sync_bits & 0x7) << 5) | ((timeFromHourStart >> 16) & 0x1F);
                out_buf[1] = timeFromHourStart >> 8;
                out_buf[2] = timeFromHourStart;
                out_buf[3] = (pvt->fixType << 6) | (pvt->numSv & 0x3F);
                out_buf[4] = pvt->lat >> 24;
                out_buf[5] = pvt->lat >> 16;
                out_buf[6] = pvt->lat >> 8;
                out_buf[7] = pvt->lat;
                out_buf[8] = pvt->lon >> 24;
                out_buf[9] = pvt->lon >> 16;
                out_buf[10] = pvt->lon >> 8;
                out_buf[11] = pvt->lon;
                out_buf[12] = gps_altitude >> 8;
                out_buf[13] = gps_altitude;
                out_buf[14] = gps_speed >> 8;
                out_buf[15] = gps_speed;
                out_buf[16] = gps_bearing >> 8;
                out_buf[17] = gps_bearing;
            } else if (rx_header->msg_id == NAV_DOP_ID) {
                dop = (ubx_nav_dop *)(((uint8_t *)rx_header) + sizeof(ubx_header));
                out_buf[18] = dop->hdop / 10;
                out_buf[19] = dop->vdop / 10;
            }

            /* update data */

            if (prevDateHour != dateHour) {
                /* check date & hour */
                prevDateHour = dateHour;
                rc_sync_bits++;
                out_buf[0] = ((rc_sync_bits & 0x7) << 5) | ((dateHour >> 16) & 0x1F);
                out_buf[1] = dateHour >> 8;
                out_buf[2] = dateHour;
                /* update data */

            }
        }

        /* check next header */
        rx_header = (ubx_header *)(((uint8_t *)rx_header) + sizeof(ubx_header) + rx_header->length + UBX_CRC_SIZE);
    } while(((((uint8_t *)rx_header) + sizeof(ubx_header)) < (buffer + BUFF_LENGTH)) && rx_header->sync1 == 0xb5 && rx_header->sync2 == 0x62);

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
