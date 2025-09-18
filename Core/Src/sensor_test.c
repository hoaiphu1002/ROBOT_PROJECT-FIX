///*
// * sensor_test.c
// *
// *  Created on: Jul 16, 2025
// *      Author: TRƯƠNG VŨ HOÀI PHÚ
// */
//
//#include "stm32f4xx_hal.h"
//#include "sensor_test.h"
//
//extern TIM_HandleTypeDef htim8;
//
//uint32_t IC_Val1 = 0;
//uint32_t IC_Val2 = 0;
//uint32_t Difference = 0;
//uint8_t Is_First_Captured = 0;  // is the first value captured ?
//uint8_t Distance  = 0;
//
//#define TRIG_PIN GPIO_PIN_9
//#define TRIG_PORT GPIOE
//
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
//	{
//		if (Is_First_Captured==0) // if the first value is not captured
//		{
//			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//			Is_First_Captured = 1;  // set the first captured as true
//			// Now change the polarity to falling edge
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
//		}
//		else if (Is_First_Captured==1)   // if the first is already captured
//		{
//			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
//			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
//
//			if (IC_Val2 > IC_Val1)
//			{
//				Difference = IC_Val2-IC_Val1;
//			}
//			else if (IC_Val1 > IC_Val2)
//			{
//				Difference = (0xffff - IC_Val1) + IC_Val2;
//			}
//			Distance = (Difference * 0.340)/2;
//			Is_First_Captured = 0; // set it back to false
//			// set polarity to rising edge
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC1);
//		}
//	}
//}
//
//void delay(uint16_t time){
//	__HAL_TIM_SET_COUNTER(&htim8, 0); // reset the counter
//	while(__HAL_TIM_GET_COUNTER(&htim8) < time);
//}
//volatile uint8_t distance_ready = 0;
//
//uint8_t HCSR04_GetDis (void)
//{
//	__HAL_TIM_SET_COUNTER(&htim8, 0);
//	    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
//	    delay(10);  // 10 µs
//	    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
//	    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC1);
//	    return Distance;
//}
//
//
//
//
//
