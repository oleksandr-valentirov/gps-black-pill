#include "gpio.h"


static void USART_12_pins_init(void);
static void SPI_pins_init(void);
static void OneWire_Pin_Init(void);


/**
 * @brief Настраивает пины, ориентируется на Pin_Configuration.h
 * @retval None
 */
void MyGPIO_Init(void)
{
    GPIO_DeInit(GPIOC);
    GPIO_DeInit(GPIOB);
    GPIO_DeInit(GPIOA);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    
    OneWire_Pin_Init();

    
#ifdef MCO1
    GPIO_InitTypeDef mco_1;
    mco_1.GPIO_Pin = MCO1_PIN;
    mco_1.GPIO_Speed = GPIO_Low_Speed;
    mco_1.GPIO_Mode = GPIO_Mode_AF;
    mco_1.GPIO_OType = GPIO_OType_PP;
    mco_1.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MCO1_PORT, &mco_1);
    GPIO_PinAFConfig(MCO1_PORT, MCO1_AF_PIN, GPIO_AF_MCO);
#endif
    
    USART_12_pins_init();
    SPI_pins_init();
    
#ifdef __SX1268_H
    LoRa_PinsInit();
#endif
}


static void USART_12_pins_init(void)
{
    GPIO_InitTypeDef pin;

    // USART1 TX pin
    pin.GPIO_OType = GPIO_OType_PP;
    pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
    pin.GPIO_Mode = GPIO_Mode_AF;
    pin.GPIO_Pin = USART_1_TX_PIN;
    pin.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(USART_1_TX_PORT, &pin);
    GPIO_PinAFConfig(USART_1_TX_PORT, USART_1_TX_AF_SRC, GPIO_AF_USART1);

    // USART1 RX pin
    pin.GPIO_Pin = USART_1_RX_PIN;
    GPIO_Init(USART_1_RX_PORT, &pin);
    GPIO_PinAFConfig(USART_1_RX_PORT, USART_1_RX_AF_SRC, GPIO_AF_USART1);
    
    // USART2 TX pin
//    pin.GPIO_Pin = USART_2_TX_PIN;
//    GPIO_Init(USART_2_TX_PORT, &pin);
//    GPIO_PinAFConfig(USART_2_TX_PORT, USART_2_TX_AF_SRC, GPIO_AF_USART2);
    
    // USART2 RX pin
//    pin.GPIO_Pin = USART_2_RX_PIN;
//    GPIO_Init(USART_2_RX_PORT, &pin);
//    GPIO_PinAFConfig(USART_2_RX_PORT, USART_2_TX_AF_SRC, GPIO_AF_USART2);
}


static void SPI_pins_init(void)
{
    GPIO_InitTypeDef pin;
    
    // SPI3
    pin.GPIO_OType = GPIO_OType_PP;
    pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
    pin.GPIO_Mode = GPIO_Mode_AF;
    pin.GPIO_Pin = SPI3_MISO_PIN | SPI3_MOSI_PIN | SPI3_CLK_PIN;
    pin.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(SPI3_PORT, &pin);
    
    // SPI3 AF
    GPIO_PinAFConfig(SPI3_PORT, SPI3_MISO_AF, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_PORT, SPI3_MOSI_AF, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_PORT, SPI3_CLK_AF, GPIO_AF_SPI3);
}


/**
 * @brief configs TIM9 CH 2 pin to AF mode
 */
static void OneWire_Pin_Init(void)
{
    GPIO_InitTypeDef pin;
    pin.GPIO_OType = GPIO_OType_PP;
    pin.GPIO_PuPd = GPIO_PuPd_NOPULL;
    pin.GPIO_Mode = GPIO_Mode_AF;
    pin.GPIO_Speed = GPIO_Low_Speed;
    pin.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &pin);
    
    CLEAR_BIT(GPIOA->ODR, GPIO_Pin_2);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9);
}
