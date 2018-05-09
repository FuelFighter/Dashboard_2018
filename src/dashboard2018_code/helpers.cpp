#include "helpers.h"


bool checkPinHigh(int pin) {
    return digitalRead(pin) == HIGH;
}

int readADCRaw(int pin) {
    return analogRead(pin);
}

// returns a number in [0..1]
double readADC(int pin) {
    int adcRes = 10;
    double res = 1;
    for (int i = 0; i < adcRes; ++i) {
        res *= 2;
    }
    return readADCRaw(pin) / res;
}

uint8_t asciiToDec(char c) {
    uint8_t dec = 0;
    
    if ((c >= '0') && (c <= '9')) {
        dec = c - '0';
        
    } 
    else if ((c >= 'A') && (c <= 'F')) {
        dec = c - 'A' + 10;
        
    } 
    else if ((c >= 'a') && (c <= 'f')) {
        dec = c - 'a' + 10;    
    } 
    else {
        dec = 0;
    }
    
    return (uint8_t)dec;
}


double exponential(const double& base, const int& exponent) {
    double result = 1;

    for (int i = 0; i < exponent; ++i) {
        result *= base;
    }

    return result;
}

void printEntireCANmsg(CAN_message_t msg, int id) {
    Serial.print(" ");
    Serial.print(id);
    Serial.print(": ");
    Serial.print(msg.id);
    Serial.print(".");
    Serial.print(msg.len);
    Serial.print(":");
    Serial.print(msg.buf[0]);
    Serial.print("-");
    Serial.print(msg.buf[1]);
    Serial.print("-");
    Serial.print(msg.buf[2]);
    Serial.print("-");
    Serial.print(msg.buf[3]);
    Serial.print("-");
    Serial.print(msg.buf[4]);
    Serial.print("-");
    Serial.print(msg.buf[5]);
    Serial.print("-");
    Serial.print(msg.buf[6]);
    Serial.print("-");
    Serial.print(msg.buf[7]);
    Serial.print("  ");
}