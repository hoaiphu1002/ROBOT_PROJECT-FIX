/*
 * ultrasonic_sensor.h
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_ULTRASONIC_SENSOR_H_
#define INC_ULTRASONIC_SENSOR_H_
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htimx) ;
void HAL_TIM2_IC_CaptureCallback(TIM_HandleTypeDef *htimx);
uint32_t US01_GetDistance(uint8_t id);
void delay_us(uint16_t us);
void US01_TriggerAll(void);
void UART_SendString(char *str) ;
void PrintAllDistances(void) ;
uint32_t US01_GetDis (void);
void US01_TriggerOne(uint8_t id) ;
void Debug_Check_Echo_Level(uint8_t id) ;
void US01_SendAllDistances_CAN();
void US01_TriggerAll_Sequential(void);
uint32_t Median_Filter(uint8_t id, uint32_t new_sample);


//// ==== Khởi động và xử lý đo siêu âm dạng state machine ====
//void US01_ProcessSequential_NonBlocking(void);   // Gọi trong vòng lặp chính
//uint8_t US01_IsMeasurementDone(void);            // Kiểm tra đo xong chưa
//void US01_ResetMeasurementFlag(void);            // Reset cờ để bắt đầu đo lại
//
//// ==== Giao tiếp & xử lý cảm biến ====
//void US01_TriggerOne(uint8_t id);                // Trigger 1 cảm biến
//uint32_t US01_GetDistance(uint8_t id);           // Lấy khoảng cách sau lọc
//void US01_SendAllDistances_CAN(void);            // Gửi kết quả CAN
//void PrintAllDistances(void);                    // In UART debug
//
//void UART_SendString(char *str);                 // Gửi chuỗi qua UART

#ifdef __cplusplus
}
#endif
#endif /* INC_ULTRASONIC_SENSOR_H_ */
