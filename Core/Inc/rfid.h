/*
 * rfid.h
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef __RFID_H
#define __RFID_H

#include "main.h"
#include <stdbool.h>

#define UID_LEN 5

typedef struct {
    uint8_t uid[UID_LEN];
    char name[32];
} RFID_User;


void checkRFIDAndControlRelay(void);
void printUserName(uint8_t *uid);
bool isAuthorizedUID(uint8_t *uid);
void CAN_Send_UID(uint8_t *uid);
void MFRC522_SoftReset(void);
void checkerror();
void checkstatus();
#endif


