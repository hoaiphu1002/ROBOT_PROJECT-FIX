  /*
 * can_tranceive.h
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "stm32f4xx_hal.h"   // để có HAL_StatusTypeDef
#include <stdbool.h>         // để dùng kiểu bool
#ifndef INC_CAN_TRANCEIVE_H_
#define INC_CAN_TRANCEIVE_H_

void CAN_Loopback_Test(void);
HAL_StatusTypeDef CAN_SendTopicData(uint16_t topic_id, uint8_t *data, uint8_t len);
HAL_StatusTypeDef CAN_SendString(uint16_t stdId, const char *str);
void Send_All_SensorData_CAN(void);
void BNO055_CheckAndRecover(void);
bool BNO055_IsStable(void);

#endif /* INC_CAN_TRANCEIVE_H_ */
