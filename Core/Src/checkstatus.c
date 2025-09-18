/*
 * checkstatus.c
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "can_receive.h"
#include "stm32f4xx_hal_uart.h"
#include "main.h"
#include "RC522.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "MQ135.h"
#include "liquidcrystal_i2c.h"
#include "BNO055_STM32.h"
#include "display.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;

uint16_t adc_value;

void I2C_Scanner(I2C_HandleTypeDef *hi2c)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)"\n--- I2C SCANNER START ---\n", 27, HAL_MAX_DELAY);
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK)
        {
            char msg[32];
            sprintf(msg, "Found device at: 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"--- SCAN DONE ---\n", 19, HAL_MAX_DELAY);
}

void ADC_CHECK (){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_value=HAL_ADC_GetValue(&hadc1);
    char adc[32];
    sprintf(adc, "ADC VALUE: %d\r\n", adc_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)adc, strlen(adc), HAL_MAX_DELAY);
}

void check_it(TIM_HandleTypeDef *htimx) {
    char *timer_name = "UNKNOWN";

    if (htimx->Instance == TIM1) timer_name = "TIM1";
    else if (htimx->Instance == TIM2) timer_name = "TIM2";
    else if (htimx->Instance == TIM4) timer_name = "TIM4";
    else if (htimx->Instance == TIM8) timer_name = "TIM8";

    char msg[64];
    snprintf(msg, sizeof(msg), ">> [INTERRUPT] Callback from %s\r\n", timer_name);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}


