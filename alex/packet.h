/**
 * Define the type of a packet used in the communication between Alex and RPi.
 */

#ifndef __CG_CONTROL_H__
#define __CG_CONTROL_H__

#include <stdint.h>

#define MAX_STR_LEN 32

// This packet has 1 + 1 + 2 + 32 + 16 * 4 = 100 bytes
typedef struct {
    char packetType;
    char command;
    char dummy[2];              // Padding to make up 4 bytes
    char data[MAX_STR_LEN];     // String data
    uint32_t params[16];
} TPacket;

#endif
