/*
 * sensor_test.h
 *
 *  Created on: Jul 16, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_SENSOR_TEST_H_
#define INC_SENSOR_TEST_H_

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void delay(uint16_t time);
uint8_t HCSR04_GetDis (void);


#endif /* INC_SENSOR_TEST_H_ */
