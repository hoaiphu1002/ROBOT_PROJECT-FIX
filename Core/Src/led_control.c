///*
// * led_control.c
// *
// *  Created on: Aug 8, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// */
//#include "stm32f4xx_hal.h"
//#include "main.h"
//#include "stdbool.h"
//#include "ultrasonic_sensor.h"
//#include "can_tranceive.h"
//#include "can_topic.h"
//#include "led_control.h"
// // - Nếu có vật thể ở gần (dưới 30cm phía trước hoặc dưới 10cm hai bên) → bật đèn đỏ, tắt đèn xanh, gửi CAN 90.
// // - Nếu không phát hiện → bật đèn xanh, tắt đèn đỏ.
// // - Nếu khoảng cách = 0cm → bỏ qua (không tính là có người).
//void Process_Ultrasonic_And_Control_Relay(void)
//{
//    static uint8_t is_person_detected = 0;  // Ghi nhớ trạng thái trước
//
//    uint32_t Truoc2 = US01_GetDistance(0);  // Trước2
//    uint32_t Truoc1 = US01_GetDistance(2);  // Trước1
//    uint32_t Trai   = US01_GetDistance(1);  // Trái
//    uint32_t Phai   = US01_GetDistance(3);  // Phải
//
//    bool detected = false;  // Ghi trạng thái hiện tại
//
//    if ((Truoc2 < 25 && Truoc2 != 0) || (Truoc1 < 25 && Truoc1 != 0) ||
//        (Trai   < 20  && Trai   != 0) || (Phai   < 20  && Phai   != 0)) {
//        detected = true;
//    }
//
//    if (detected && !is_person_detected) {
//        // 🧍 Người mới đến: Đèn đỏ bật, xanh tắt
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // PA2 = 1 → Relay đỏ KHÔNG kích → Đèn đỏ sáng (qua NC)
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // PA3 = 0 → Relay xanh kích → NC ngắt → Đèn xanh tắt
//
//        uint8_t signal = 90;
//        CAN_SendTopicData(TOPIC_ID_SENSOR, &signal, 1);
//        is_person_detected = 1;
//    }
//    else if (!detected && is_person_detected) {
//        // 👤 Người đã rời đi: Đèn xanh bật, đỏ tắt
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // PA2 = 0 → Relay đỏ kích → NC ngắt → Đèn đỏ tắt
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // PA3 = 1 → Relay xanh không kích → Đèn xanh sáng
//
//        is_person_detected = 0;
//    }
//}
//
//
//
//
/*
/*
 *
 *  Created on: Aug 8, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
//#include "stm32f4xx_hal.h"
//#include "main.h"
//#include "stdbool.h"
//#include "ultrasonic_sensor.h"
//#include "can_tranceive.h"
//#include "can_topic.h"
//#include "led_control.h"
//void Process_Ultrasonic_And_Control_Relay(void)
//{
//    static uint8_t initialized = 0;
//
//    uint32_t Truoc2 = US01_GetDistance(0);
//    uint32_t Truoc1 = US01_GetDistance(2);
//    uint32_t Trai   = US01_GetDistance(1);
//    uint32_t Phai   = US01_GetDistance(3);
//
//    if (!initialized) {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // Đỏ OFF
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Xanh ON
//        initialized = 1;
//        return;
//    }
//
//    // Kiểm tra các vùng
//    bool Truoc_OK  = (Truoc1 < 25 && Truoc1 != 0) || (Truoc2 < 25 && Truoc2 != 0);
//    bool HaiBen_OK = (Trai   < 20 && Trai   != 0) || (Phai   < 20 && Phai   != 0);
//    bool BaBen_OK  = Truoc_OK && HaiBen_OK;
//    bool TatCa_OK  = (Truoc1 < 25 && Truoc1 != 0) &&
//                     (Truoc2 < 25 && Truoc2 != 0) &&
//                     (Trai   < 20 && Trai   != 0) &&
//                     (Phai   < 20 && Phai   != 0);
//
//    uint8_t signal = 0x00;
//
//    // Xác định giá trị gửi
//    if (TatCa_OK) {
//        signal = 0x03;
//    } else if (BaBen_OK) {
//        signal = 0x02;
//    } else if (Truoc_OK) {
//        signal = 0x01;
//    } else {
//        signal = 0x00; // Ra khỏi 3 trường hợp trên
//    }
//
//    // Điều khiển đèn
//    if (TatCa_OK || BaBen_OK || Truoc_OK || HaiBen_OK) {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // Đỏ ON
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // Xanh OFF
//    } else {
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // Đỏ OFF
//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Xanh ON
//    }
//
//    // Luôn gửi giá trị hiện tại
//    uint8_t data[8] = {0};
//    data[0] = signal;
//    CAN_SendTopicData(TOPIC_ID_SENSOR, data, 8);
//}

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include "ultrasonic_sensor.h"
#include "can_tranceive.h"
#include "can_topic.h"
#include "led_control.h"

extern CAN_HandleTypeDef hcan1;

void Process_Ultrasonic_And_Control_Relay(void)
{
    static uint8_t initialized = 0;

    uint32_t Truoc2 = US01_GetDistance(0);
    uint32_t Truoc1 = US01_GetDistance(2);
    uint32_t Trai   = US01_GetDistance(1);
    uint32_t Phai   = US01_GetDistance(3);

    if (!initialized) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // Đỏ OFF
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // Xanh ON
        initialized = 1;
        return;
    }

    // Kiểm tra các vùng
    bool Truoc_OK  = (Truoc1 < 30 && Truoc1 != 0) || (Truoc2 < 30 && Truoc2 != 0);
    bool HaiBen_OK = (Trai   < 25 && Trai   != 0) || (Phai   < 25 && Phai   != 0);
    bool BaBen_OK  = Truoc_OK && HaiBen_OK;
    bool TatCa_OK  = (Truoc1 < 25 && Truoc1 != 0) &&
                     (Truoc2 < 25 && Truoc2 != 0) &&
                     (Trai   < 20 && Trai   != 0) &&
                     (Phai   < 20 && Phai   != 0);

    uint8_t signal = 0x00;

    // Xác định giá trị gửi
    if (TatCa_OK) {
        signal = 0x03;
    } else if (BaBen_OK) {
        signal = 0x02;
    } else if (Truoc_OK) {
        signal = 0x01;
    } else {
        signal = 0x00; // Ra khỏi 3 trường hợp trên
    }

    // Điều khiển đèn
    if (TatCa_OK || BaBen_OK || Truoc_OK || HaiBen_OK) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // Đỏ ON
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // Xanh OFF
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // Đỏ OFF
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // Xanh ON
    }

    // Luôn gửi giá trị hiện tại nhưng không để LED bị treo nếu CAN lỗi
    uint8_t data[8] = {0};
    data[0] = signal;
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_READY ||
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING)
    {
        if (CAN_SendTopicData(TOPIC_ID_SENSOR, data, 8) != HAL_OK) {
            // Bỏ qua lỗi CAN, LED vẫn hoạt động
        }
    }

}


