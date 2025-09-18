/*
 * can_send_noblocking.c
 *
 *  Created on: Sep 16, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "can_send_noblocking.h"
//HAL_StatusTypeDef CAN_SendNonBlocking(CAN_HandleTypeDef *hcan,
//                                      CAN_TxHeaderTypeDef *header,
//                                      uint8_t *data,
//                                      uint32_t *mailbox)
//{
//    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
//        return HAL_CAN_AddTxMessage(hcan, header, data, mailbox);
//    } else {
//        return HAL_BUSY;
//    }
//}

HAL_StatusTypeDef CAN_SendNonBlocking(CAN_HandleTypeDef *hcan,
                                      CAN_TxHeaderTypeDef *header,
                                      uint8_t *data,
                                      uint32_t *mailbox)
{
    // 1. Kiểm tra mailbox nào trống (theo TSR)
    uint32_t tsr = hcan->Instance->TSR;

    if (tsr & CAN_TSR_TME0) {
        *mailbox = 0;
    } else if (tsr & CAN_TSR_TME1) {
        *mailbox = 1;
    } else if (tsr & CAN_TSR_TME2) {
        *mailbox = 2;
    } else {
        // mailbox đầy → abort để tránh nghẽn
        HAL_CAN_AbortTxRequest(hcan, 0x7);
        return HAL_BUSY;
    }

    // 2. Thử gửi
    if (HAL_CAN_AddTxMessage(hcan, header, data, mailbox) == HAL_OK) {
        return HAL_OK;
    }

    // 3. Nếu lỗi → retry nhanh (tối đa 2 lần)
    for (int retry = 0; retry < 2; retry++) {
        if (HAL_CAN_AddTxMessage(hcan, header, data, mailbox) == HAL_OK) {
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}



