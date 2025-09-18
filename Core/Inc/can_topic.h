#ifndef __CAN_TOPIC_H
#define __CAN_TOPIC_H
typedef enum {
    TOPIC_ID_RFID      = 0x010,
    TOPIC_ID_MQ135	   = 0x011,
    TOPIC_ID_IMU_EULER = 0x012,
    TOPIC_ID_IMU_Gyro  = 0x013,
    TOPIC_ID_IMU_Accel = 0x014,
    TOPIC_ID_TEST  	   = 0x015,
    TOPIC_ID_US01      = 0x016,
//    TOPIC_ID_vel1      = 0x017,
//    TOPIC_ID_vel2  	   = 0x018,
    TOPIC_ID_SEND_ALL  = 0x023,
	TOPIC_ID_SENSOR    = 0X024,

} CAN_TopicID;
#endif // __CAN_TOPIC_H
