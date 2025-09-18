#ifndef DISPLAY_TOPIC_H
#define DISPLAY_TOPIC_H

#include "main.h"
#include "BNO055_STM32.h"
#include "can_topic.h"

void DisplayTopicMenuUART(void);
void HandleUARTChoice(void);
void DisplaySensorData_ByTopic(void);

extern volatile uint16_t current_display_topic;

#endif
