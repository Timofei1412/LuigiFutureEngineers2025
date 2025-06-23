#ifndef DEFINES_H
#define DEFINES_H


#include "Arduino.h"

// #define DebugSerial Serial
#define CameraSerial Serial
#define ArraysLengthType uint16_t

#define NO_SECOND_FLOOR false
#define GRID_SIZE 17
#define MAX_NODES 289
#define MAX_EDGES 4


struct CameraInfo{
    uint8_t red;
    uint8_t line;
    uint8_t green;
    uint8_t blue;
    uint8_t floor;
};


#endif