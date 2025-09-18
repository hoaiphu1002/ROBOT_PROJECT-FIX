#ifndef __BNO055_IT_H
#define __BNO055_IT_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef BNO055_IT_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len);
HAL_StatusTypeDef BNO055_IT_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len);

#endif
