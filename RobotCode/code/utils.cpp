#include "utils.h"

// extern  DebugSerial;


void printArray(uint8_t* array, uint8_t size) {
  for (int i = 0; i < size - 1; ++i) {
    DebugSerial.print(array[i]);
    DebugSerial.print(", ");
  }
  DebugSerial.print(array[size - 1]);
  DebugSerial.println();
}

void printArray(uint16_t* array, uint8_t size) {
  for (int i = 0; i < size - 1; ++i) {
    DebugSerial.print(array[i]);
    DebugSerial.print(", ");
  }
  DebugSerial.print(array[size - 1]);
  DebugSerial.println();
}

void printArray(bool* array, uint8_t size) {
  for (int i = 0; i < size; ++i) {
    DebugSerial.print(array[i] ? "@" : ".");
  }
  DebugSerial.println();
}

bool inArray(uint8_t* array, uint8_t size, uint8_t val) {
  for (int i = 0; i < size; ++i) {
    if (array[i] == val) return 1;
  }
  return 0;
}

bool inArray(uint16_t* array, uint8_t size, uint16_t val) {
  for (int i = 0; i < size; ++i) {
    if (array[i] == val) return 1;
  }
  return 0;
}

void removeFromArray(uint8_t* array, uint8_t size, uint8_t idx) {
  for (int i = idx; i < size - 1; ++i) {
    array[i] = array[i + 1];
  }
}

void removeFromArray(uint16_t* array, uint8_t size, uint16_t idx) {
  for (int i = idx; i < size - 1; ++i) {
    array[i] = array[i + 1];
  }
}

uint8_t getIndex(int8_t* array, uint8_t size, int8_t val) {
  for (int i = 0; i < size; i++) {
    if (array[i] == val) return i;
  }
  return size + 1;
}