/**
 * Low-level utilities function used for serial communication
 */

#ifndef __CG_SERIAL_H__
#define __CG_SERIAL_H__

#define MAX_BUFFER_LEN 1024
#include <termios.h>

void startSerial(const char *portName, int baudRate, int byteSize, char parity, int stopBits, int maxAttempts);
int serialRead(char *buffer);
void serialWrite(char *buffer, int len);
void endSerial();

#endif
