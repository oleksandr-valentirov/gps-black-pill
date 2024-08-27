#pragma once

typedef enum ubx_status {
    UBX_OK = 0,
    UBX_TIMEOUT,
    UBX_NACK,
    UBX_CRC_ERROR,
    UBX_CFG_MSG_CLASSID_ERROR,
    UBX_CFG_MSG_MSGID_ERROR,
    UBX_CFG_PRT_ERROR,
    UBX_CFG_NAV_RATE_ERROR,
    UBX_CFG_MSG_RATE_ERROR,
    UBX_BadFix
} ubx_status;

sys_status UBX_Init(DMA_TypeDef *DMAx, USART_TypeDef *USARTx);
void UBX_ProcessData(void);
