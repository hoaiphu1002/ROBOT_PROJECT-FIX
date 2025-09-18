/*
 * can_send_noblocking.h
 *
 *  Created on: Sep 16, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_CAN_SEND_NOBLOCKING_H_
#define INC_CAN_SEND_NOBLOCKING_H_
#include "stm32f4xx_hal.h"   // hoặc dòng đúng với dòng chip của bạn

HAL_StatusTypeDef CAN_SendNonBlocking(CAN_HandleTypeDef *hcan,
                                      CAN_TxHeaderTypeDef *txHeader,
                                      uint8_t *data,
                                      uint32_t *txMailbox);


#endif /* INC_CAN_SEND_NOBLOCKING_H_ */
