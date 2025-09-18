// ultrasonic_sensor.c

#include "stm32f4xx_hal.h"
#include "ultrasonic_sensor.h"
#include <string.h>
#include <stdio.h>
#include "can_receive.h"
#include "can_tranceive.h"
#include <checkstatus.h>
#include "can_topic.h"

#define NUM_SENSORS 4
#define FILTER_WINDOW_SIZE 5

// ==== GPIOS ====
GPIO_TypeDef* GPIOx_for_ECHO[NUM_SENSORS] = {GPIOE, GPIOA, GPIOD, GPIOC};
uint16_t GPIO_PIN_ECHO[NUM_SENSORS] = {GPIO_PIN_9, GPIO_PIN_5, GPIO_PIN_12, GPIO_PIN_6};
GPIO_TypeDef* TRIG_PORT[NUM_SENSORS] = {GPIOE, GPIOA, GPIOD, GPIOD};
uint16_t      TRIG_PIN[NUM_SENSORS]  = {GPIO_PIN_10, GPIO_PIN_6, GPIO_PIN_11, GPIO_PIN_15};

// ==== TIMERS ====
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

TIM_HandleTypeDef* htim[NUM_SENSORS] = {&htim1, &htim2, &htim4, &htim8};
uint32_t TIM_CHANNEL[NUM_SENSORS] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_1};
uint32_t TIM_IT_CC[NUM_SENSORS] = {TIM_IT_CC1, TIM_IT_CC1, TIM_IT_CC1, TIM_IT_CC1};

// ==== DỮ LIỆU ====
volatile uint32_t IC_Val1[NUM_SENSORS];
volatile uint32_t IC_Val2[NUM_SENSORS];
volatile uint8_t  Is_First_Captured[NUM_SENSORS] = {0};
volatile uint32_t Distances[NUM_SENSORS];

uint32_t distance_history[NUM_SENSORS][FILTER_WINDOW_SIZE] = {0};
uint8_t filter_index[NUM_SENSORS] = {0};

// ==== HÀM LỌC TRUNG VỊ ====
uint32_t Median_Filter(uint8_t id, uint32_t new_sample) {
    distance_history[id][filter_index[id]] = new_sample;
    filter_index[id] = (filter_index[id] + 1) % FILTER_WINDOW_SIZE;

    uint32_t temp[FILTER_WINDOW_SIZE];
    memcpy(temp, distance_history[id], sizeof(temp));

    // Sort
    for (int i = 0; i < FILTER_WINDOW_SIZE - 1; i++) {
        for (int j = i + 1; j < FILTER_WINDOW_SIZE; j++) {
            if (temp[i] > temp[j]) {
                uint32_t t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }

    return temp[FILTER_WINDOW_SIZE / 2];
}

// ==== DELAY MICRO GIÂY ====
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);
    while(__HAL_TIM_GET_COUNTER(&htim6) < us);
    HAL_TIM_Base_Stop(&htim6);
}

// ==== TRIGGER MỘT CẢM BIẾN ====
void US01_TriggerOne(uint8_t id) {
    HAL_GPIO_WritePin(TRIG_PORT[id], TRIG_PIN[id], GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT[id], TRIG_PIN[id], GPIO_PIN_RESET);

    Is_First_Captured[id] = 0;
    HAL_TIM_IC_Start_IT(htim[id], TIM_CHANNEL[id]);
    __HAL_TIM_ENABLE_IT(htim[id], TIM_IT_CC[id]);

    char msg[64];
    snprintf(msg, sizeof(msg), "TRIG sensor %d\r\n", id);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// ==== NGẮT INPUT CAPTURE ====
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htimx) {
    check_it(htimx);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (htimx->Instance == htim[i]->Instance) {
            if (Is_First_Captured[i] == 0) {
                IC_Val1[i] = HAL_TIM_ReadCapturedValue(htimx, TIM_CHANNEL[i]);
                Is_First_Captured[i] = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htimx, TIM_CHANNEL[i], TIM_INPUTCHANNELPOLARITY_FALLING);
            } else {
                IC_Val2[i] = HAL_TIM_ReadCapturedValue(htimx, TIM_CHANNEL[i]);
                __HAL_TIM_SET_COUNTER(htimx, 0);

                uint32_t max_timer = __HAL_TIM_GET_AUTORELOAD(htim[i]);
                uint32_t diff = (IC_Val2[i] > IC_Val1[i]) ?
                                (IC_Val2[i] - IC_Val1[i]) :
                                ((max_timer - IC_Val1[i]) + IC_Val2[i]);

                uint32_t raw = (diff * 0.034f) / 2.0f;
//                Distances[i] = Median_Filter(i, raw);
                Distances[i]=raw;

                Is_First_Captured[i] = 0;

                char msg[128];
                snprintf(msg, sizeof(msg),
                         "[US%u] Raw: %lu, Filtered: %lu cm\r\n",
                         i, raw, Distances[i]);
                UART_SendString(msg);

                __HAL_TIM_SET_CAPTUREPOLARITY(htimx, TIM_CHANNEL[i], TIM_INPUTCHANNELPOLARITY_RISING);
                __HAL_TIM_DISABLE_IT(htimx, TIM_IT_CC[i]);
                HAL_TIM_IC_Stop_IT(htimx, TIM_CHANNEL[i]);
            }
            break;
        }
    }
}

// ==== TRUY XUẤT GIÁ TRỊ ====
uint32_t US01_GetDistance(uint8_t id) {
    return (id < NUM_SENSORS) ? Distances[id] : 0;
}

void UART_SendString(char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// ==== GỬI DỮ LIỆU CAN ====
void US01_SendAllDistances_CAN(void) {
    uint8_t data[8];

    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t dist = (uint16_t)US01_GetDistance(i);

        if (dist > 500 || dist <= 1) {
            dist = 0x0033;
        }

        data[2 * i]     = (dist >> 8) & 0xFF;
        data[2 * i + 1] = dist & 0xFF;
    }

    CAN_SendTopicData(TOPIC_ID_US01, data, 8);
}

void US01_TriggerAll_Sequential(void) {
    // KHÔNG DÙNG nếu bạn đã dùng trigger từng sensor trong vòng lặp
    for (int i = 0; i < NUM_SENSORS; i++) {
        US01_TriggerOne(i);
        delay_us(10);
    }
}

// ==== IN KẾT QUẢ ====
void PrintAllDistances(void) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "Truoc2: %lucm | Trai: %lucm | Truoc1: %lucm | Phai: %lucm\r\n",
             Distances[0], Distances[1], Distances[2], Distances[3]);
    UART_SendString(buf);
	}
