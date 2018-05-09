#include "arduino.h"

#ifndef CAN_H_
#define CAN_H_

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
void parseUARTbufferToCANmessage(char bufferCorrected[], CAN_message_t& msg1, CAN_message_t& msg2);

#endif