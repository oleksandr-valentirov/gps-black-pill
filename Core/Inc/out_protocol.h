#pragma once

#include <stdint.h>

typedef struct out_header {
    uint8_t     sync1;
    uint8_t     sync2;
    uint16_t    checksum;
    uint8_t     classID;
    uint8_t     length;
} out_header;

typedef struct ubx_output {
    int32_t     lon;
    int32_t     lat;
    int32_t     gSpeed;
    int32_t     heading;
    int32_t     velN;
    int32_t     velE;
    int32_t     nano;
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     min;
    uint8_t     sec;
    uint8_t     fixType;
} ubx_output;

typedef struct error_output {
    uint32_t error;
} error_output;
