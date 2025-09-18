#ifndef MQ135_H
#define MQ135_H

#include "main.h"
#include <math.h>

// Hằng số hiệu chỉnh
#define MQ135_CORA 0.00035f
#define MQ135_CORB 0.02718f
#define MQ135_CORC 1.39538f
#define MQ135_CORD 0.0018f

#define MQ135_ATMOCO2 400.0f
#define MQ135_READ_SAMPLES 100

typedef struct {
    ADC_HandleTypeDef *hadc;
    float rl_value;
    float ro_clean_air;
    float a;
    float b;
    float vref;
    float resolution;
} MQ135_HandleTypeDef;

void MQ135_Config(MQ135_HandleTypeDef *mq, ADC_HandleTypeDef *hadc);
float MQ135_ReadRs(MQ135_HandleTypeDef *mq);
float MQ135_CorrectionFactor(float temp, float hum);
float MQ135_ReadCorrectedPPM(MQ135_HandleTypeDef *mq, float temp, float hum);
void MQ135_CalibrateRo(MQ135_HandleTypeDef *mq, float temp, float hum);
void MQ135_Send_CAN(MQ135_HandleTypeDef *mq, float temp, float hum, UART_HandleTypeDef *huart, uint16_t topic);

#endif
