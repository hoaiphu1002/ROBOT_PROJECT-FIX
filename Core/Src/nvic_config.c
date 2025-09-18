#include "can_receive.h"
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


	void NVIC_Config(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);   // Gửi dữ liệu nhanh khi có sẵn
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_NVIC_SetPriority(TIM4_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

    HAL_NVIC_SetPriority(ADC_IRQn, 6, 0);       // MQ135 đo xong
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);  // Thấp nhất
}
