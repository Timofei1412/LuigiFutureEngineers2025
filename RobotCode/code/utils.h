#ifndef utils_h
#define utils_h

#include <Arduino.h>
#include "Defines.h"
#include "BluetoothSerial.h"

extern BluetoothSerial DebugSerial;

void printArray(uint8_t*, uint8_t);
void printArray(uint16_t*, uint8_t);
void printArray(bool*, uint8_t);

bool inArray(uint8_t*, uint8_t, uint8_t);
bool inArray(uint16_t*, uint8_t, uint16_t);

void removeFromArray(uint8_t*, uint8_t, uint8_t);
void removeFromArray(uint16_t*, uint8_t, uint16_t);

uint8_t getIndex(int8_t*, uint8_t, int8_t);
#endif