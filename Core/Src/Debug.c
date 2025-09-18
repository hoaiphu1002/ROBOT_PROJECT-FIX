/*
 * Debug.c
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "RC522.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "MQ135.h"
#include "liquidcrystal_i2c.h"
#include "BNO055_STM32.h"
#include "display.h"
#include "rfid.h"
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
void CAN_DebugStatus(void)
{
    char msg[128];
    uint32_t msr = hcan1.Instance->MSR;
    uint32_t esr = hcan1.Instance->ESR;

    // In trạng thái FIFO
    uint32_t fifo0_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    uint32_t fifo1_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1);

    snprintf(msg, sizeof(msg),
             "\r\n[CAN DEBUG]\r\nMSR=0x%08lX\r\nESR=0x%08lX\r\nFIFO0=%lu, FIFO1=%lu\r\n",
             msr, esr, fifo0_level, fifo1_level);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Giải thích các trạng thái nếu cần
    if (esr & CAN_ESR_BOFF) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️ CAN BUS-OFF\r\n", 17, HAL_MAX_DELAY);
    }
    if (esr & CAN_ESR_EPVF) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️ Error Passive\r\n", 19, HAL_MAX_DELAY);
    }
    if (esr & CAN_ESR_EWGF) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️ Error Warning\r\n", 19, HAL_MAX_DELAY);
    }

    if ((esr & CAN_ESR_LEC_Msk) != 0) {
        const char* lec_msgs[] = {
            "No Error", "Stuff Error", "Form Error", "Ack Error",
            "Bit recessive Error", "Bit dominant Error", "CRC Error", "Unknown"
        };
        uint8_t lec = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
        snprintf(msg, sizeof(msg), "❌ Last Error Code (LEC): %s\r\n", lec_msgs[lec]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

