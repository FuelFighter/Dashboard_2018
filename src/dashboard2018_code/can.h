#include "arduino.h"

#ifndef CAN_H_
#define CAN_H_

#define CANTX 3
#define CANRX 4
#define CAN_BAUDRATE 500000
#define SERIAL_BAUDRATE 500000

#define ID_MOTOR_CONTROLLER 0x450
#define DASHBOARD_CAN_ID    0x230
#define MOTOR_1_STATUS_CAN_ID 0x250
#define MOTOR_2_STATUS_CAN_ID 0x260
#define BMS_VOLT_CURRENT_CAN_ID 0x444
#define E_CLUTCH_1_CAN_ID 0x120
#define E_CLUTCH_2_CAN_ID 0x220

static const int CAN_LEN = 25;  // bits

typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;



/* UART FUNCTIONS */
// Reads CAN message from UART connected to a UM in CANtoUART mode. (See github)
// This is a work around since I still can't get CANbus to work directly on the
// Teensy
void readCANfromUARTtoBuffer(char out[]);
void sendCANoverUART(CAN_message_t& msg);

// Parses the UART buffer to a CAN_message_t object for easier reading. 
void parseUARTbufferToCANmessage(const char bufferCorrected[], CAN_message_t& msg1, CAN_message_t& msg2);

#endif