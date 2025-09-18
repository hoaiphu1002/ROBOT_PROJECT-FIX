#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define PN532_PREAMBLE      0x00
#define PN532_STARTCODE1    0x00
#define PN532_STARTCODE2    0xFF
#define PN532_POSTAMBLE     0x00
#define PN532_HOSTTOPN532   0xD4
#define PN532_PN532TOHOST   0xD5

#define PN532_ACK_WAIT_TIME 100
#define PN532_TIMEOUT       -1
#define PN532_INVALID_ACK   -2
#define PN532_INVALID_FRAME -3
#define PN532_NO_SPACE      -4

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *ssPort;
    uint16_t ssPin;
    uint8_t command;
} PN532_SPI;

void PN532_SPI_Init(PN532_SPI *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin);
void PN532_SPI_Begin(PN532_SPI *dev);
void PN532_SPI_Wakeup(PN532_SPI *dev);
int8_t PN532_SPI_WriteCommand(PN532_SPI *dev, const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen);
int16_t PN532_SPI_ReadResponse(PN532_SPI *dev, uint8_t *buf, uint8_t len, uint16_t timeout);

#endif
