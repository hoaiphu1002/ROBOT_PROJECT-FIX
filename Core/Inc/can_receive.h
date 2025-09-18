/*
 * can_receive.h
 *
 *  Created on: Jul 11, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_CAN_RECEIVE_H_
#define INC_CAN_RECEIVE_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// External variables
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t rxData[8];
extern volatile uint8_t can_rx_flag;

// Function prototypes
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_ReceiveAndPrint(void);
void print_velocity_both_nodes();

#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_RECEIVE_H_ */
