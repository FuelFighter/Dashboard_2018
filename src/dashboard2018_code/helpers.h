#include "arduino.h"
#include "can.h"

#ifndef HELPERS_H_
#define HELPERS_H_

bool checkPinHigh(int pin);
int readADCRaw(int pin);
double readADC(int pin);

// returns an ascii hex value in it's deciaml representation
uint8_t asciiToDec(char c);

// calculate exponential
double exponential(const double& base, const int& exponent);

void printEntireCANmsg(CAN_message_t msg, int id);
void printTXmsg(CAN_message_t txmsg);

#endif