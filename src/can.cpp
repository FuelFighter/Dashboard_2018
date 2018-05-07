#include "can.h"
#include "helpers.h"
#include "arduino.h"


void readCANfromUARTtoBuffer(char out[]) {
    char buffer[CAN_LEN*2] = {0};
    char bufferCorrected[CAN_LEN*2] = {0};

    if (Serial3.available()) {
        Serial3.readBytes(buffer, CAN_LEN*2);
    }


    // correct order
    const int OFFSET_INDEX = 13;
    for (int i = 0; i < CAN_LEN; ++i) {  // place at start
        bufferCorrected[i] = buffer[OFFSET_INDEX + i];
    }
    for (int i = CAN_LEN; i < 2 * CAN_LEN - OFFSET_INDEX; ++i) {
        bufferCorrected[i] = buffer[OFFSET_INDEX + i];
    }
    for (int i = 2 * CAN_LEN - OFFSET_INDEX; i < 2 * CAN_LEN; ++i) {
        bufferCorrected[i] = buffer[i - (2 * CAN_LEN - OFFSET_INDEX)];
    }

    for (int i = 0; i < CAN_LEN * 2; ++i) {
        out[i] = bufferCorrected[i];
    }
}

void sendCANoverUART(CAN_message_t& msg) {
    char msgBuffer[CAN_LEN] = {0};

    sprintf(msgBuffer, "[%03X:%d:%02X%02X%02X%02X%02X%02X%02X%02X]\n", 
            msg.id, msg.len, msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3], msg.buf[4],
            msg.buf[5], msg.buf[6], msg.buf[7]);

    Serial3.print(msgBuffer);
}

void parseUARTbufferToCANmessage(char bufferCorrected[], CAN_message_t& msg1, CAN_message_t& msg2) {
    // received message parsing
    // msg1
    msg1.id = asciiToDec(bufferCorrected[1]) << 8 | asciiToDec(bufferCorrected[2]) << 4 | asciiToDec(bufferCorrected[3]) << 0;
    msg1.len = asciiToDec(bufferCorrected[5]);

    msg1.buf[7] = (asciiToDec(bufferCorrected[21])) << 4 | (asciiToDec(bufferCorrected[22]));
    msg1.buf[6] = (asciiToDec(bufferCorrected[19])) << 4 | (asciiToDec(bufferCorrected[20]));
    msg1.buf[5] = (asciiToDec(bufferCorrected[17])) << 4 | (asciiToDec(bufferCorrected[18]));
    msg1.buf[4] = (asciiToDec(bufferCorrected[15])) << 4 | (asciiToDec(bufferCorrected[16]));
    msg1.buf[3] = (asciiToDec(bufferCorrected[13])) << 4 | (asciiToDec(bufferCorrected[14]));
    msg1.buf[2] = (asciiToDec(bufferCorrected[11])) << 4 | (asciiToDec(bufferCorrected[12]));
    msg1.buf[1] = (asciiToDec(bufferCorrected[ 9])) << 4 | (asciiToDec(bufferCorrected[10]));
    msg1.buf[0] = (asciiToDec(bufferCorrected[ 7])) << 4 | (asciiToDec(bufferCorrected[ 8]));

    // msg2
    msg2.id = asciiToDec(bufferCorrected[26]) << 8 | asciiToDec(bufferCorrected[27]) << 4 | asciiToDec(bufferCorrected[28]) << 0;
    msg2.len = asciiToDec(bufferCorrected[30]);

    msg2.buf[7] = (asciiToDec(bufferCorrected[46])) << 4 | (asciiToDec(bufferCorrected[47]));
    msg2.buf[6] = (asciiToDec(bufferCorrected[44])) << 4 | (asciiToDec(bufferCorrected[45]));
    msg2.buf[5] = (asciiToDec(bufferCorrected[42])) << 4 | (asciiToDec(bufferCorrected[43]));
    msg2.buf[4] = (asciiToDec(bufferCorrected[40])) << 4 | (asciiToDec(bufferCorrected[41]));
    msg2.buf[3] = (asciiToDec(bufferCorrected[38])) << 4 | (asciiToDec(bufferCorrected[39]));
    msg2.buf[2] = (asciiToDec(bufferCorrected[36])) << 4 | (asciiToDec(bufferCorrected[37]));
    msg2.buf[1] = (asciiToDec(bufferCorrected[34])) << 4 | (asciiToDec(bufferCorrected[35]));
    msg2.buf[0] = (asciiToDec(bufferCorrected[32])) << 4 | (asciiToDec(bufferCorrected[33]));
}