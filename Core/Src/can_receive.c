/*
 * can_receive.c
 *
 *  Created on: Jul 11, 2025
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

//  Chỉ khai báo extern, KHÔNG định nghĩa lại
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t txData[8];
extern uint8_t rxData[8];
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t can_rx_count ;


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//    HAL_UART_Transmit(&huart1, (uint8_t*)"INTERRUPT OK\r\n", 15, HAL_MAX_DELAY);
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxData) ;
                  can_rx_flag = 1;  // báo về main xử lý
                  can_rx_count++;  // tăng biến đếm khi nhận
}

void CAN_ReceiveAndPrint(void)
{
//    uint8_t RxData[8];
    char buffer[64];

//    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rxData) ;

    if (RxHeader.DLC == 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️ EMPTY FRAME RECEIVED\r\n", 26, HAL_MAX_DELAY);
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"✅ CAN RECEIVED\r\n", 17, HAL_MAX_DELAY);

    snprintf(buffer, sizeof(buffer), "RX : ID=0x%03lX DLC=%lu DATA=",
             (uint32_t)RxHeader.StdId, (uint32_t)RxHeader.DLC);

    for (uint8_t i = 0; i < RxHeader.DLC; i++) {
        char hex[8];
        sprintf(hex, "%02X ", rxData[i]);
        strcat(buffer, hex);
    }

    strcat(buffer, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    for (int i = 0; i < 4; i++) {
        char dbg[32];
        snprintf(dbg, sizeof(dbg), "Byte[%d] = 0x%02X\r\n", i, rxData[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
    }
    float velocity_f = 0.00f;
            int32_t velocity_i = 0;

            memcpy(&velocity_f, rxData, sizeof(float));
            memcpy(&velocity_i, rxData, sizeof(int32_t)); // nếu cần int thay float

            snprintf(buffer, sizeof(buffer),
                     "\rVelocity: float = %.2f | int = %ld\r\n",
                     velocity_f, (long)velocity_i);

            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}



