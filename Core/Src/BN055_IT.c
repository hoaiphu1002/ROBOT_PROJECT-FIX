/*
 * BNO055_IT.c
 *
 *  Created on: Aug 28, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#include "BNO055_STM32.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern I2C_HandleTypeDef bno_i2c;  // handle I2C cho BNO055

// Cờ báo hiệu khi I2C xong hoặc lỗi
volatile uint8_t BNO055_I2C_Done  = 0;
volatile uint8_t BNO055_I2C_Error = 0;

/* =====================================================
 *      WRAPPERS: Dùng IT mode thay cho blocking
 * ==================================================== */

// I2C Read (non-blocking với interrupt)
HAL_StatusTypeDef BNO055_IT_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len)
{
    BNO055_I2C_Done  = 0;
    BNO055_I2C_Error = 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(&bno_i2c,
                                                   devAddr,
                                                   regAddr,
                                                   I2C_MEMADD_SIZE_8BIT,
                                                   pData,
                                                   len);

    if (status != HAL_OK) return status;

    // chờ callback báo hiệu (timeout 100ms)
    uint32_t tickstart = HAL_GetTick();
    while (!BNO055_I2C_Done && !BNO055_I2C_Error)
    {
        if ((HAL_GetTick() - tickstart) > 100) {
            return HAL_TIMEOUT;
        }
    }

    return (BNO055_I2C_Error ? HAL_ERROR : HAL_OK);
}

// I2C Write (non-blocking với interrupt)
HAL_StatusTypeDef BNO055_IT_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t len)
{
    BNO055_I2C_Done  = 0;
    BNO055_I2C_Error = 0;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&bno_i2c,
                                                    devAddr,
                                                    regAddr,
                                                    I2C_MEMADD_SIZE_8BIT,
                                                    pData,
                                                    len);

    if (status != HAL_OK) return status;

    uint32_t tickstart = HAL_GetTick();
    while (!BNO055_I2C_Done && !BNO055_I2C_Error)
    {
        if ((HAL_GetTick() - tickstart) > 100) {
            return HAL_TIMEOUT;
        }
    }

    return (BNO055_I2C_Error ? HAL_ERROR : HAL_OK);
}

/* =====================================================
 *      I2C CALLBACKS
 * ==================================================== */

// Callback khi đọc xong
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == bno_i2c.Instance) {
        BNO055_I2C_Done = 1;
    }
}

// Callback khi ghi xong
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == bno_i2c.Instance) {
        BNO055_I2C_Done = 1;
    }
}

// Callback khi có lỗi I2C
// Thêm biến cờ toàn cục
volatile uint8_t bno055_need_reset = 0;

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == bno_i2c.Instance)
    {
        BNO055_I2C_Error = 1;
        bno055_need_reset = 1;   // báo cho main loop biết
    }

}
