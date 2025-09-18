/*
 * rfid.c
 *
 *  Created on: Jul 14, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */
#include "rfid.h"
#include "RC522.h"
#include <string.h>
#include <stdio.h>
#include "can_tranceive.h"   // để dùng CAN_SendTopicData()
#include "can_topic.h"       // để dùng TOPIC_ID_RFID

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;

extern RFID_User userList[];
extern const int userCount;

uint8_t currentUID[UID_LEN] = {0};
uint8_t lastUID[UID_LEN] = {0};
bool relayOn = false;
int rfidLostCounter = 0;  // Đếm số vòng không đọc được thẻ
int rfidDetected = 0;  // Cờ xác định có thẻ trong vòng hiện tại
int uidcheck = 0;  // =1 nếu thẻ đang được giữ

#define RFID_LOST_THRESHOLD 20
extern uchar str[MAX_LEN];
extern uchar status;
// Danh sách người dùng
RFID_User userList[] = {
	{{0xD2, 0xB1, 0x3D, 0x05, 0x5B}, "MINH KY"},
    {{0xEF, 0xA8, 0x98, 0x1E, 0xC1}, "CHI THIEN"},
    {{0xD2, 0x0F, 0x49, 0x2E, 0xBA}, "HOAI PHU"},
	{{0xFA, 0xDC, 0x02, 0xCD, 0xE9}, "QUANG DUY"},
	{{0xB6, 0x87, 0x13, 0x2B, 0x09}, "VAN LOI"},
	{{0xC2, 0xBF, 0xB0, 0x2E, 0xE3}, "BACH THU"},

};

const int userCount = sizeof(userList) / sizeof(RFID_User);
char debug[64];



void MFRC522_SoftReset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
    HAL_Delay(50);  // đợi chip khởi động lại
}

void checkerror()
{
	uchar errorVal = Read_MFRC522(ErrorReg);
	sprintf(debug, "ErrorReg: 0x%02X\r\n", errorVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);
	 if (errorVal & 0x01) HAL_UART_Transmit(&huart1, (uint8_t*)"CRC error\r\n", 11, HAL_MAX_DELAY);
	    if (errorVal & 0x02) HAL_UART_Transmit(&huart1, (uint8_t*)"Parity error\r\n", 14, HAL_MAX_DELAY);
	    if (errorVal & 0x04) HAL_UART_Transmit(&huart1, (uint8_t*)"CollErr (va cham)\r\n", 20, HAL_MAX_DELAY);
	    if (errorVal & 0x10) HAL_UART_Transmit(&huart1, (uint8_t*)"Protocol error\r\n", 17, HAL_MAX_DELAY);
	}

void checkstatus(){
    char msg[32];
    sprintf(msg, "Status: %d\r\n", status);
	 MFRC522_SoftReset();
	 status= MFRC522_Request(PICC_REQIDL, str);
	 if (status==1){
      status = MFRC522_Anticoll(str);
	    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 }
        // Không tìm thấy với REQIDL → thử với REQALL (đánh thức thẻ HALT)
	    if (status==2){
        HAL_UART_Transmit(&huart1, (uint8_t*)"No tag with REQIDL. Trying REQALL...\r\n", 40, HAL_MAX_DELAY);
        status = MFRC522_Request(PICC_REQALL, str);
        status = MFRC522_Anticoll(str);
	    }

	    void CAN_Send_UID(uint8_t* uid) {
	        // Gửi UID qua CAN topic chuẩn
	        CAN_SendTopicData(TOPIC_ID_RFID, uid, UID_LEN);  // UID_LEN = 5

	        // In UID ra UART để kiểm tra
	        char dbg[64];
	        sprintf(dbg, "[RFID->CAN] UID: %02X %02X %02X %02X %02X\r\n",
	                uid[0], uid[1], uid[2], uid[3], uid[4]);
	        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
	    }
}


void printUserName(uint8_t *uid)
{
	   // Không có thẻ
	    if (uid == NULL)
	    {
	        if (uidcheck)
	        {
	            HAL_UART_Transmit(&huart1, (uint8_t*)"Please charge your ID CARD \r\n", 31, HAL_MAX_DELAY);
	            memset(lastUID, 0, UID_LEN); //Gán toàn bộ giá trị trong mảng lastUID về 0 (zero)
	        }
	        return;
	    }

	    //  Nếu là UID giống lần trước thì không in lại
	    if (uidcheck && memcmp(uid, lastUID, UID_LEN) == 0)
	        return;
	    if (uid!=NULL)
	    {
	    //  UID mới thì cập nhật và xử lý
	    memcpy(lastUID, uid, UID_LEN);
	    uidcheck = 1;
    for (int i = 0; i < userCount; i++)
    {
        if (memcmp(uid, userList[i].uid, UID_LEN) == 0)
        {
            char msg[64];
            sprintf(msg, "YOUR ID: %02X %02X %02X %02X %02X - %s\r\n",
                    uid[0], uid[1], uid[2], uid[3], uid[4],
                    userList[i].name);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }
    // Không tìm thấy trong danh sách
    char unknown[64];
    sprintf(unknown, "UID: %02X %02X %02X %02X %02X - User not valid\r\n",
            uid[0], uid[1], uid[2], uid[3], uid[4]);
    HAL_UART_Transmit(&huart1, (uint8_t*)unknown, strlen(unknown), HAL_MAX_DELAY);
}
}


bool isAuthorizedUID(uint8_t *uid)
{
    for (int i = 0; i < userCount; i++)
    {
        if (memcmp(uid, userList[i].uid, 5) == 0)
            return true;
    }
    return false;
}


void checkRFIDAndControlRelay(void)
{
    status = MFRC522_Request(PICC_REQIDL, str);

    if (status == MI_OK && MFRC522_Anticoll(str) == MI_OK)
    {
        memcpy(currentUID, str, UID_LEN);
        rfidDetected = 1;
        rfidLostCounter = 0;

        // UART Debug
        char dbg[64];
        sprintf(dbg, "[RFID] UID: %02X %02X %02X %02X %02X\r\n",
                currentUID[0], currentUID[1], currentUID[2],
                currentUID[3], currentUID[4]);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);

//        // ✅ Chỉ gửi CAN nếu là UID mới
        if (!uidcheck || memcmp(currentUID, lastUID, UID_LEN) != 0)
        {
            printUserName(currentUID);
            memcpy(lastUID, currentUID, UID_LEN);
            uidcheck = 1;

            // Gửi CAN
            CAN_SendTopicData(TOPIC_ID_RFID, currentUID, UID_LEN);
        }

        // ✅ Điều khiển Relay
        if (isAuthorizedUID(currentUID))
        {
            if (!relayOn)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                relayOn = true;
            }
        }
        else
        {
            if (relayOn)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                relayOn = false;
            }
        }

        return;
    }

    //  Không đọc được thẻ
    rfidLostCounter++;
//    if (rfidLostCounter >= RFID_LOST_THRESHOLD)
//    {
        // Chỉ thực hiện reset khi thực sự không còn thẻ
        if (uidcheck)
        {
            printUserName(NULL);
            uidcheck = 0;
            memset(lastUID, 0, UID_LEN);
            rfidDetected = 0;

            HAL_UART_Transmit(&huart1,
                              (uint8_t*)"[RFID] Không phát hiện thẻ\r\n",
                              strlen("[RFID] Không phát hiện thẻ\r\n"),
                              HAL_MAX_DELAY);

            if (relayOn)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
                relayOn = false;
            }
        }

        rfidLostCounter = 0;
    }
//}




