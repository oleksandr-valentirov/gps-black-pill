/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#include "!Project_library.h"

/* ------------- LoRa functions ----------- */
void SPI3_Init(void);
uint8_t SPI3_send_data(uint16_t data);
uint8_t SPI3_receive_data(uint16_t *dest);
void SPI3_data_size_config(uint8_t size);
/* ---------------------------------------- */


/* ------------- General functions -------- */
void SPI1_Init(void);
/* ---------------------------------------- */

#endif