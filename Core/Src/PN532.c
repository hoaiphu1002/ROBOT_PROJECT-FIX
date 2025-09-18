/*
 * PN532.c
 *
 *  Created on: Jul 1, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "PN532.h"

static void PN532_SPI_Write(PN532_SPI *dev, uint8_t data) {
    HAL_SPI_Transmit(dev->hspi, &data, 1, HAL_MAX_DELAY);
}

static uint8_t PN532_SPI_Read(PN532_SPI *dev) {
    uint8_t tx = 0x00, rx = 0;
    HAL_SPI_TransmitReceive(dev->hspi, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

void PN532_SPI_Init(PN532_SPI *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *port, uint16_t pin) {
    dev->hspi = hspi;
    dev->ssPort = port;
    dev->ssPin = pin;
    dev->command = 0;
}

void PN532_SPI_Begin(PN532_SPI *dev) {
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);
}

void PN532_SPI_Wakeup(PN532_SPI *dev) {
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);
}

static bool PN532_SPI_IsReady(PN532_SPI *dev) {
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_RESET);
    PN532_SPI_Write(dev, 0x02);  // STATUS_READ
    uint8_t status = PN532_SPI_Read(dev) & 1;
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);
    return status;
}

static void PN532_SPI_WriteFrame(PN532_SPI *dev, const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen) {
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_RESET);
    HAL_Delay(2);

    PN532_SPI_Write(dev, 0x01);  // DATA_WRITE
    PN532_SPI_Write(dev, PN532_PREAMBLE);
    PN532_SPI_Write(dev, PN532_STARTCODE1);
    PN532_SPI_Write(dev, PN532_STARTCODE2);

    uint8_t length = hlen + blen + 1;
    PN532_SPI_Write(dev, length);
    PN532_SPI_Write(dev, ~length + 1);

    PN532_SPI_Write(dev, PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532;

    for (uint8_t i = 0; i < hlen; i++) {
        PN532_SPI_Write(dev, header[i]);
        sum += header[i];
    }

    for (uint8_t i = 0; i < blen; i++) {
        PN532_SPI_Write(dev, body[i]);
        sum += body[i];
    }

    PN532_SPI_Write(dev, ~sum + 1);
    PN532_SPI_Write(dev, PN532_POSTAMBLE);

    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);
}

static int8_t PN532_SPI_ReadAckFrame(PN532_SPI *dev) {
    const uint8_t ACK_FRAME[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    uint8_t ackBuf[6];

    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    PN532_SPI_Write(dev, 0x03); // DATA_READ

    for (int i = 0; i < 6; i++) {
        ackBuf[i] = PN532_SPI_Read(dev);
    }

    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);

    for (int i = 0; i < 6; i++) {
        if (ackBuf[i] != ACK_FRAME[i]) return PN532_INVALID_ACK;
    }
    return 0;
}

int8_t PN532_SPI_WriteCommand(PN532_SPI *dev, const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen) {
    dev->command = header[0];
    PN532_SPI_WriteFrame(dev, header, hlen, body, blen);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    while (!PN532_SPI_IsReady(dev)) {
        HAL_Delay(1);
        if (--timeout == 0) return PN532_TIMEOUT;
    }

    return PN532_SPI_ReadAckFrame(dev);
}

int16_t PN532_SPI_ReadResponse(PN532_SPI *dev, uint8_t *buf, uint8_t len, uint16_t timeout) {
    uint16_t timer = 0;
    while (!PN532_SPI_IsReady(dev)) {
        HAL_Delay(1);
        if (++timer > timeout) return PN532_TIMEOUT;
    }

    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    PN532_SPI_Write(dev, 0x03); // DATA_READ

    if (PN532_SPI_Read(dev) != 0x00 || PN532_SPI_Read(dev) != 0x00 || PN532_SPI_Read(dev) != 0xFF)
        return PN532_INVALID_FRAME;

    uint8_t length = PN532_SPI_Read(dev);
    if ((length + PN532_SPI_Read(dev)) != 0x00)
        return PN532_INVALID_FRAME;

    uint8_t cmd = dev->command + 1;
    if (PN532_SPI_Read(dev) != PN532_PN532TOHOST || PN532_SPI_Read(dev) != cmd)
        return PN532_INVALID_FRAME;

    length -= 2;
    if (length > len) {
        for (uint8_t i = 0; i < length + 2; i++) {
            PN532_SPI_Read(dev);  // dump
        }
        return PN532_NO_SPACE;
    }

    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length; i++) {
        buf[i] = PN532_SPI_Read(dev);
        sum += buf[i];
    }

    if ((sum + PN532_SPI_Read(dev)) != 0x00)
        return PN532_INVALID_FRAME;

    PN532_SPI_Read(dev);  // postamble
    HAL_GPIO_WritePin(dev->ssPort, dev->ssPin, GPIO_PIN_SET);

    return length;
}


