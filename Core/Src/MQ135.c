#include "MQ135.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "can_tranceive.h"

// Biến bên ngoài dùng cho lấy mẫu ADC qua ngắt
extern uint32_t ADC_SAMPLES[1000];
extern uint32_t NUM_SAMPLES;
extern volatile uint8_t mq135_done;

void MQ135_Config(MQ135_HandleTypeDef *mq, ADC_HandleTypeDef *hadc) {
    mq->hadc = hadc;
    mq->rl_value = 10.0f;
    mq->ro_clean_air = 10.0f;
    mq->a = 116.6020682f;
    mq->b = -2.769034857f;
    mq->vref = 3.3f;
    mq->resolution = 4096.0f;
}

float MQ135_CorrectionFactor(float temp, float hum) {
    return MQ135_CORA * temp * temp
         - MQ135_CORB * temp
         + MQ135_CORC
         - (hum - 33.0f) * MQ135_CORD;
}

float MQ135_ReadRs(MQ135_HandleTypeDef *mq) {
    float rs = 0.0f;
    mq135_done = 0;
    NUM_SAMPLES = MQ135_READ_SAMPLES;
    HAL_ADC_Start_IT(mq->hadc);
    while (!mq135_done);  // đợi hoàn tất lấy mẫu

    for (uint32_t j = 0; j < NUM_SAMPLES; j++) {
        float v = ADC_SAMPLES[j] * mq->vref / mq->resolution;
        rs += ((mq->vref - v) * mq->rl_value) / v;
    }

    return rs / NUM_SAMPLES;
}

float MQ135_ReadCorrectedPPM(MQ135_HandleTypeDef *mq, float temp, float hum) {
    float rs = MQ135_ReadRs(mq);
    float corr = MQ135_CorrectionFactor(temp, hum);
    float rsc = rs / corr;
    return mq->a * powf(rsc / mq->ro_clean_air, mq->b);
}

void MQ135_CalibrateRo(MQ135_HandleTypeDef *mq, float temp, float hum) {
    mq->ro_clean_air = MQ135_ReadRs(mq) *
        powf((MQ135_ATMOCO2 / mq->a), (1.0f / mq->b));
}

void MQ135_Send_CAN(MQ135_HandleTypeDef *mq, float temp, float hum, UART_HandleTypeDef *huart, uint16_t topic) {
    float ppm = MQ135_ReadCorrectedPPM(mq, temp, hum);
    uint16_t v = (uint16_t)ppm;
    char buf[80];
    sprintf(buf, "PPM: %.1f\r\n", ppm);
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    uint8_t payload[2] = { (v >> 8) & 0xFF, v & 0xFF };
    CAN_SendTopicData(topic, payload, 2);
}
