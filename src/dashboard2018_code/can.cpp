#include "can.h"
#include "helpers.h"
#include "arduino.h"


void correctCANbuffer(char buf[], char out[]) {
    const int LEN = CAN_LEN * 2;
    int i_opensquare = LEN + 1;
    int i_closedsquare = LEN + 1;  // to definetly be outside of bounds

    // first, find index of first [ and index of first ] AFTER the first [
    for (int i = 0; i < LEN; ++i) {
        if (buf[i] == '[') i_opensquare = i;
        if (buf[i] == ']' && i > i_opensquare) i_closedsquare = i;  // first ']' after first '['

        // take the middle message in input buffer and copy it into the begining of the output buffer
        if (i >= i_opensquare && i <= i_closedsquare) {  // here is why it's originally set to outside of bounds
            out[i - i_opensquare] = buf[i];
        }
    }

    // staples together the second message
    // first, take the last part of the buffer (which is the beginning of the second message) and paste in into output buffer from middle and out
    for (int i = i_closedsquare + 1; i < LEN; ++i) {
        out[CAN_LEN + i - (i_closedsquare + 1)] = buf[i];
    }

    for (int i = 0; i < i_opensquare; ++i) {
        out[CAN_LEN + LEN - (i_closedsquare + 1) + i] = buf[i];
    }
}

// void readCANfromUARTtoBuffer(char out[]) {
//     char buffer[CAN_LEN*2] = {0};
//     char bufferCorrected[CAN_LEN*2] = {0};

//     if (Serial3.available()) {
//         Serial3.readBytes(buffer, CAN_LEN*2);
//     }

//     Serial.print(" B4Corr:  ");
//     Serial.print(buffer);
//     Serial.print("  AFTcorr  ");


//     // OLD ODER CORRECT. SWITCHES THINGS AROUND WRONGLY NOW FOR SOME STUPID REASON????????
//     // // correct order
//     // const int OFFSET_INDEX = 13;
//     // for (int i = 0; i < CAN_LEN; ++i) {  // place at start
//     //     bufferCorrected[i] = buffer[OFFSET_INDEX + i];
//     // }
//     // for (int i = CAN_LEN; i < 2 * CAN_LEN - OFFSET_INDEX; ++i) {
//     //     bufferCorrected[i] = buffer[OFFSET_INDEX + i];
//     // }
//     // for (int i = 2 * CAN_LEN - OFFSET_INDEX; i < 2 * CAN_LEN; ++i) {
//     //     bufferCorrected[i] = buffer[i - (2 * CAN_LEN - OFFSET_INDEX)];
//     // }

//     correctCANbuffer(buffer, bufferCorrected);

//     for (int i = 0; i < CAN_LEN * 2; ++i) {
//         out[i] = bufferCorrected[i];
//     }
// }

void readCANfromUARTtoBuffer(char out[]) {
    char buffer[CAN_LEN] = {0};
    char bufferCorrected[CAN_LEN * 2] = {0};

    if (Serial3.available()) {
        Serial3.readBytes(buffer, CAN_LEN);
    }

    Serial.print(buffer);

    int i_opensquare = -1;
    int i_closedsquare = -1;
    for (int i = 0; i < CAN_LEN; ++i) {
        if (buffer[i] == '[') i_opensquare   = i;
        if (buffer[i] == ']') i_closedsquare = i;

        if (i_opensquare != -1) {
            bufferCorrected[i - i_opensquare] = buffer[i];
        }
    }
    for (int i = 0; i <= i_closedsquare; ++i) {
        if (i_closedsquare != -1) {
            bufferCorrected[CAN_LEN - i_opensquare + i] = buffer[i];
        }
    }

    for (int i = 0; i < CAN_LEN; ++i) {
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

void parseUARTbufferToCANmessage(const char bufferCorrected[], CAN_message_t& msg1, CAN_message_t& msg2) {
    // received message parsing
    // msg1
    msg1.id = asciiToDec(bufferCorrected[1]) << 8 | asciiToDec(bufferCorrected[2]) << 4 | asciiToDec(bufferCorrected[3]) << 0;
    msg1.len = asciiToDec(bufferCorrected[5]);

    // for (int i = 0; i < msg1.len; ++i) {
    //     switch (i) {
    //         case 0:
    //             msg1.buf[0] = (asciiToDec(bufferCorrected[ 7])) << 4 | (asciiToDec(bufferCorrected[ 8]));
    //             break;
    //         case 1:        
    //             msg1.buf[1] = (asciiToDec(bufferCorrected[ 9])) << 4 | (asciiToDec(bufferCorrected[10]));
    //             break;
    //         case 2:
    //             msg1.buf[2] = (asciiToDec(bufferCorrected[11])) << 4 | (asciiToDec(bufferCorrected[12]));
    //             break;
    //         case 3:
    //             msg1.buf[3] = (asciiToDec(bufferCorrected[13])) << 4 | (asciiToDec(bufferCorrected[14]));
    //             break;
    //         case 4:
    //             msg1.buf[4] = (asciiToDec(bufferCorrected[15])) << 4 | (asciiToDec(bufferCorrected[16]));
    //             break;
    //         case 5:
    //             msg1.buf[5] = (asciiToDec(bufferCorrected[17])) << 4 | (asciiToDec(bufferCorrected[18]));
    //             break;
    //         case 6:
    //             msg1.buf[6] = (asciiToDec(bufferCorrected[19])) << 4 | (asciiToDec(bufferCorrected[20]));
    //             break;
    //         case 7:
    //             msg1.buf[7] = (asciiToDec(bufferCorrected[21])) << 4 | (asciiToDec(bufferCorrected[22]));
    //         default:
    //             msg1.buf[i] = 0;
    //     }

    //     // I know this is terribly stupid and ugly code. 
    //     // Not going to figure out a smarter way, though. So deal with it.
    // }

    msg1.buf[7] = (asciiToDec(bufferCorrected[21])) << 4 | (asciiToDec(bufferCorrected[22]));
    msg1.buf[6] = (asciiToDec(bufferCorrected[19])) << 4 | (asciiToDec(bufferCorrected[20]));
    msg1.buf[5] = (asciiToDec(bufferCorrected[17])) << 4 | (asciiToDec(bufferCorrected[18]));
    msg1.buf[4] = (asciiToDec(bufferCorrected[15])) << 4 | (asciiToDec(bufferCorrected[16]));
    msg1.buf[3] = (asciiToDec(bufferCorrected[13])) << 4 | (asciiToDec(bufferCorrected[14]));
    msg1.buf[2] = (asciiToDec(bufferCorrected[11])) << 4 | (asciiToDec(bufferCorrected[12]));
    msg1.buf[1] = (asciiToDec(bufferCorrected[ 9])) << 4 | (asciiToDec(bufferCorrected[10]));
    msg1.buf[0] = (asciiToDec(bufferCorrected[ 7])) << 4 | (asciiToDec(bufferCorrected[ 8]));

    for (int i = msg1.len; i < 8; ++i) {
        msg1.buf[i] = 0;  // had issues with getting wrong data where it shouldn't be any. this should fix it. 
    }

    // // msg2
    // msg2.id = asciiToDec(bufferCorrected[26]) << 8 | asciiToDec(bufferCorrected[27]) << 4 | asciiToDec(bufferCorrected[28]) << 0;
    // msg2.len = asciiToDec(bufferCorrected[30]);

    // msg2.buf[7] = (asciiToDec(bufferCorrected[46])) << 4 | (asciiToDec(bufferCorrected[47]));
    // msg2.buf[6] = (asciiToDec(bufferCorrected[44])) << 4 | (asciiToDec(bufferCorrected[45]));
    // msg2.buf[5] = (asciiToDec(bufferCorrected[42])) << 4 | (asciiToDec(bufferCorrected[43]));
    // msg2.buf[4] = (asciiToDec(bufferCorrected[40])) << 4 | (asciiToDec(bufferCorrected[41]));
    // msg2.buf[3] = (asciiToDec(bufferCorrected[38])) << 4 | (asciiToDec(bufferCorrected[39]));
    // msg2.buf[2] = (asciiToDec(bufferCorrected[36])) << 4 | (asciiToDec(bufferCorrected[37]));
    // msg2.buf[1] = (asciiToDec(bufferCorrected[34])) << 4 | (asciiToDec(bufferCorrected[35]));
    // msg2.buf[0] = (asciiToDec(bufferCorrected[32])) << 4 | (asciiToDec(bufferCorrected[33]));

    msg2.id = 0;
    msg2.len = 0;
    msg2.buf[7] = 0;
    msg2.buf[6] = 0;
    msg2.buf[5] = 0;
    msg2.buf[4] = 0;
    msg2.buf[3] = 0;
    msg2.buf[2] = 0;
    msg2.buf[1] = 0;
    msg2.buf[0] = 0;
}