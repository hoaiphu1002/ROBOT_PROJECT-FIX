#include "display.h"
#include "can_receive.h"
#include "can_tranceive.h"
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
#include "rfid.h"
#include "can_topic.h"
#include "ultrasonic_sensor.h"
#include "BN055_IT.h"

extern volatile uint8_t timer10ms_flag ;

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;


volatile uint16_t current_display_topic = 0;
static char uart_rx_char = 0;
extern MQ135_HandleTypeDef mq135;


void DisplayTopicMenuUART(void)
{
    const char *menu =
        "\r\n=== CHỌN DỮ LIỆU HIỂN THỊ ===\r\n"
        "1. Euler (Góc Roll, Pitch, Yaw)\r\n"
        "2. Gyroscope (Tốc độ xoay)\r\n"
        "3. Acceleration (Gia tốc)\r\n"
        "4. Ultrasonic sensor (Khoảng cách)\r\n"
        "5. MQ135 (Khí gas - PPM)\r\n"
        "6. RFID (UID người dùng)\r\n"
        "9. Gửi tất cả dữ liệu cảm biến\r\n"
        "0. Tắt hiển thị / Dừng cập nhật\r\n"
        "------------------------------\r\n"
        "Chọn chế độ: ";
    HAL_UART_Transmit(&huart1, (uint8_t*)menu, strlen(menu), HAL_MAX_DELAY);
}


void HandleUARTChoice(void)
{
    if (HAL_UART_Receive(&huart1, (uint8_t*)&uart_rx_char, 1, 10) == HAL_OK) {
        switch (uart_rx_char)
        {
            case '1':
                current_display_topic = TOPIC_ID_IMU_EULER;
                const char *msg = "\r\n Góc: EULER\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                break;

            case '2':
                current_display_topic = TOPIC_ID_IMU_Gyro;
                const char *msg2 = "\r\n Tốc độ quay: Gyro\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
                break;

            case '3':
                current_display_topic = TOPIC_ID_IMU_Accel;
                const char *msg3 = "\r\n Gia tốc quay: Accel\r\n";
                 HAL_UART_Transmit(&huart1, (uint8_t*)msg3, strlen(msg3), HAL_MAX_DELAY);
                break;
            case '4':
                current_display_topic =TOPIC_ID_US01;
                const char *msg4 = "\r\n Distance: \r\n";
                 HAL_UART_Transmit(&huart1, (uint8_t*)msg4, strlen(msg4), HAL_MAX_DELAY);
                break;
            case '5':
                current_display_topic = TOPIC_ID_MQ135;
                HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n MQ135: PPM\r\n", 16, HAL_MAX_DELAY);
                break;

            case '6':
                current_display_topic = TOPIC_ID_RFID;
                HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n RFID: UID\r\n", 14, HAL_MAX_DELAY);
                break;

            case '9':
                current_display_topic = TOPIC_ID_SEND_ALL;  // gửi tất cả
                HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n Gửi tất cả dữ liệu\r\n", 25, HAL_MAX_DELAY);
                break;

            case '0':
                current_display_topic = 0;
                const char *msg5 = "\r\n Dừng cập nhật \r\n";
                 HAL_UART_Transmit(&huart1, (uint8_t*)msg5, strlen(msg5), HAL_MAX_DELAY);
                DisplayTopicMenuUART();
                break;

            default:
                break;
        }

        uart_rx_char = 0;
    }
}


void DisplaySensorData_ByTopic(void)
{
    extern uint32_t debug_timer;
    extern MQ135_HandleTypeDef mq135;
    switch (current_display_topic) {

    case TOPIC_ID_IMU_EULER:
        if (timer10ms_flag){
        	timer10ms_flag = 0;
            BNO055_SendEulerCAN();
        }
        if (HAL_GetTick() - debug_timer >= 10) {
            BNO055_PrintEulerDebug();
            debug_timer = HAL_GetTick();
        }
        break;

//    case TOPIC_ID_IMU_Gyro:
//        BNO055_SendGyroCAN();
//        break;
//
//    case TOPIC_ID_IMU_Accel:
//        BNO055_SendAccelCAN();
//        break;

    case TOPIC_ID_US01:
        US01_TriggerAll_Sequential();
        HAL_Delay(100);
        US01_SendAllDistances_CAN();
        break;

    case TOPIC_ID_MQ135:
        MQ135_Send_CAN(&mq135, 25.0f, 50.0f, &huart1, TOPIC_ID_MQ135);
        break;

    case TOPIC_ID_RFID:
        checkRFIDAndControlRelay();
        break;

    case TOPIC_ID_SEND_ALL:
        Send_All_SensorData_CAN();
        break;

    default:
        // Không làm gì nếu không có topic được chọn
        break;
    }
}
