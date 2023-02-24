/**
 * Arduino code used to power Alex
 */

#include <serialize.h>
#include <buffer.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

typedef enum {
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;





/****************************************
 * Configuration constants
 ****************************************/

// Alex sizes in centimeters
#define ALEX_LENGTH 24.0
#define ALEX_BREADTH 18.0

// Alex forward/reverse PWM adjustment to straighten the movement
#define LEFT_FORWARD 1.05
#define RIGHT_FORWARD 1
#define LEFT_REVERSE 1
#define RIGHT_REVERSE 1

// Alex turn calibration constants
// Lower = softer turn, higher = heavier Turn
// Current settings are for 80% power, smooth table surface
#define TURN_LEFT_PWM 0.8
#define RIGHT_ROTATE_PWM 0.8

// Alex's diagonal
// ALEX_DIAGONAL = sqrt(ALEX_LENGTH ^ 2 + ALEX_BREADTH ^ 2);
#define ALEX_DIAGONAL 30.0

// Alex's turning circumference
// ALEX_CIRC = PI * ALEX_DIAGONAL;
#define ALEX_CIRC 94.2477796

// Number of ticks per revolution from the wheel encoder
#define COUNTS_PER_REV 30 // LEFT

// Wheel circumference in cm.
// We will use this to calculate distance traveled by taking revs * WHEEL_CIRC
#define WHEEL_CIRC 21
#define PI 3.141592654

// Control Pins PWM Baremetal
// PORTD - Timer 0
#define PIN6 (1 << 6)
#define PIN5 (1 << 5)
// PORTB - Timer 1
#define PIN9 (1 << 1)
#define PIN10 (1 << 2)





/****************************************
 * State variables
 ****************************************/

// Tick count from Alex's left and right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

/**
 * Reset all state variables
 */
void clearCounters() {
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;
    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;
    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

// Keep track of current movement state
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

// Buffer variables, used to read and send data respectively
TBuffer RBuffer;
TBuffer XmitBuffer;





/****************************************
 * Functions for serial communication
 ****************************************/

/**
 * Write data to the serial port whenever it is possible to do so by using
 * interrupts.
 */
ISR(USART_UDRE_vect) {
    unsigned char data;
    TBufferResult result = readBuffer(&XmitBuffer, &data);
    if (result == BUFFER_OK) UDR0 = data;
    else if (result == BUFFER_EMPTY) UCSR0B &= 0b11011111;
}

/**
 * Receive data from the serial port whenever there is data available by using
 * interrupts.
 */
ISR(USART_RX_vect) {
    unsigned char data = UDR0;
    writeBuffer(&RBuffer, data);
}

/**
 * Set up serial communication with bare-metal programming
 */
void setupSerial() {
    UBRR0L = 103;
    UBRR0H = 0;
    UCSR0C = 0b00000110;
    UCSR0A = 0;
    initBuffer(&RBuffer, 128);
    initBuffer(&XmitBuffer, 128);
}

/**
 * Start the serial communication with bare-metal programming
 */
void startSerial() {
    UCSR0B = 0b10111000;
}

/**
 * Read data from the serial port
 * @param buffer The buffer to read into
 * @return The size of the data read
 */
int readSerial(char *buffer) {
    int count = 0;
    TBufferResult result;
    do {
        result = readBuffer(&RBuffer, &(buffer[count]));
        if (result == BUFFER_OK) count++;
    } while (result == BUFFER_OK);
    return count;
}

/**
 * Write data to the serial port
 * @param buffer The buffer to write from
 * @param len The length of the data to write
 */
void writeSerial(const char *buffer, int len) {
    TBufferResult result = BUFFER_OK;
    freeBuffer(&XmitBuffer);

    for (int i = 0; i < len && result == BUFFER_OK; i++)
        result = writeBuffer(&XmitBuffer, buffer[i]);

    UDR0 = buffer[0]; // ball rolling
    UCSR0B |= 0b00100000;
}

ISR(TIMER0_COMPA_vect) {}
ISR(TIMER0_COMPB_vect) {}
ISR(TIMER1_COMPA_vect) {}
ISR(TIMER1_COMPB_vect) {}

/**
 * Read in data from the serial port and deserialize it for processing.
 * @param packet The packet to deserialize into.
 */
TResult readPacket(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len = readSerial(buffer);
    if (len == 0) return PACKET_INCOMPLETE;
    else return deserialize(buffer, len, packet);
}

/**
 * Send a response to RPi by serial communication. We serialize the input packet,
 * then write it to the serial port.
 *
 * @param packet The packet to send.
 */
void sendResponse(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}

/**
 * Set the current state variables to a packet and send it to the serial port.
 */
void sendStatus() {
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    statusPacket.params[0] = leftForwardTicks;
    statusPacket.params[1] = rightForwardTicks;
    statusPacket.params[2] = leftReverseTicks;
    statusPacket.params[3] = rightReverseTicks;
    statusPacket.params[4] = leftForwardTicksTurns;
    statusPacket.params[5] = rightForwardTicksTurns;
    statusPacket.params[6] = leftReverseTicksTurns;
    statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[8] = forwardDist;
    statusPacket.params[9] = reverseDist;
    sendResponse(&statusPacket);
}

/**
 * Debugging: Send a message back to RPi.
 */
void sendMessage(const char* message) {
    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

/**
 * Debugging: A wrapper for sendMessage() that sends a formatted string.
 * The format is similar to printf().
 */
void dbprint(char* format, ...) {
    va_list args;
    char buffer[128];
    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}

/**
 * Send a BAD_PACKET response to RPi. Used when the packet is invalid (bad magic number)
 */
void sendBadPacket() {
    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);

}

/**
 * Send a BAD_CHECKSUM response to RPi. Used when the packet is invalid (bad checksum)
 */
void sendBadChecksum() {
    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

/**
 * Send a BAD_COMMAND response to RPi. Used when the packet cannot be understood
 * or has been corrupted.
 */
void sendBadCommand() {
    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);
}

/**
 * Send a BAD_RESPONSE packet to RPi.
 */
void sendBadResponse() {
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

/**
 * Send a OK packet to RPi. If this function is executed instead of the above
 * BAD functions, congratulations.
 */
void sendOK() {
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}





/****************************************
 * Functions for external interrupts and pull-up resistors
 ****************************************/

/**
 * Enable pull-up resistors on pins 2 and 3 in bare metal. Those pins are PD2
 * and PD3 respectively. We set bits 2 and 3 in DDRD to 0 to make them inputs.
 */
void enablePullups() {
    DDRD &= 0b11111001;
    PORTD |= 0b0000110;
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.

/**
 * Set up the external interrupt pins INT0 and INT1 for falling edge triggered,
 * in bare-metal programming.
 */
void setupEINT() {
    EICRA = 0b1010;
    EIMSK = 0b11;
}

/**
 * Handle external interrupts on INT0 (leftISR) and INT1 (rightISR). We modify
 * state variables accordingly.
 */
void leftISR() {
    switch (dir) {
        case FORWARD:
            leftForwardTicks += 1;
            forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV *WHEEL_CIRC);
            break;
        case BACKWARD:
            leftReverseTicks += 1;
            reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV *WHEEL_CIRC);
            break;
        case RIGHT:
            leftForwardTicksTurns += 1;
            break;
        case LEFT:
            leftReverseTicksTurns += 1;
            break;
        default:
            break;
    }
}
void rightISR() {
    switch (dir) {
        case FORWARD:
            rightForwardTicks += 1;
            break;
        case BACKWARD:
            rightReverseTicks += 1;
            break;
        case RIGHT:
            rightReverseTicksTurns += 1;
            break;
        case LEFT:
            rightForwardTicksTurns += 1;
            break;
        default:
            break;
    }
}
ISR(INT0_vect) {
    leftISR();
}
ISR(INT1_vect) {
    rightISR();
}





/****************************************
 * Handling motor and movement
 ****************************************/

/**
 * Set up Alex's motors with bare-metal programming. Our motor set up is:
 * - A1IN - Pin 5, PD5, OC0B
 * - A2IN - Pin 6, PD6, OC0A
 * - B1IN - Pin 9, PB2, OC1B
 * - B2IN - Pin 10, PB3, OC2A
 */
void setupMotors() {
    TCNT0 = 0; // Timer 0 (Left 5F and 6R)
    TCNT1 = 0; // Timer 1 (Right 9F and 10R)
    DDRD |= (PIN5 | PIN6);
    DDRB |= (PIN9 | PIN10);
    TIMSK0 |= 0b110;
    TIMSK1 |= 0b110;
    OCR0A = 0; // PIN6  - Store PWM Value of Left Reverse
    OCR0B = 0; // PIN5  - Store PWM Value of Left Forward
    OCR1B = 0; // PIN10 - Store PWM Value of Right Reverse
    OCR1A = 0; // PIN9  - Store PWM Value of Right Forward
}

/**
 * Start Alex's motor with bare-metal programming.
 * CTC MODE PRESCALER 64
 */
void startMotors() {
    TCCR0B = 0b00000011;
    TCCR1B = 0b00000011;
}

/**
 * Utility function to convert percentages to PWM values.
 * @param speed The speed in percentages, taken from command sent from RPi
 * @return The PWM value to be sent to the motor.
 */
int pwmVal(float speed) {
    if (speed < 0.0) speed = 0;
    if (speed > 100.0) speed = 100.0;
    return (int)((speed / 100.0) * 255.0);
}

/**
 * Utility function to get the number of ticks corresponding to a given angle
 * @param ang The angle in degrees
 * @return The number of ticks corresponding to the angle
 */
unsigned long computeDeltaTicks(float ang) {
    return (unsigned long)((ang * ALEX_CIRC * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
}

/**
 * Alex motor control pin description:
 * 5 and 6 LEFT WHEEL => OCR0
 * PIN 5 - OCR0B (FORWARD)
 * PIN 6 - OCR0A (REVERSE)
 * 9 and 10 RIGHT WHEEL => OCR1
 * PIN 9 - OCR1B (REVERSE)
 * PIN 10 - OCR1A (FORWARD)
 */

/**
 * Move Alex forward `dist` cm at speed `speed`.
 * @param dist The distance to move forward in cm. If 0, move forward indefinitely.
 * @param speed The speed to move forward in percentages
 */
void forward(float dist, float speed) {
    deltaDist = (dist == 0) ? 999999 : dist;
    newDist = forwardDist + deltaDist;
    dir = FORWARD;
    int val = pwmVal(speed);
    OCR0B = val * LEFT_FORWARD;
    OCR1A = val * RIGHT_FORWARD;
    OCR1B = 0;
    OCR0A = 0;
    TCCR0A = 0b00100001;
    TCCR1A = 0b10000001;
}

/**
 * Move Alex backward "dist" cm at speed "speed".
 * @param dist The distance to move backward in cm. If 0, move backward indefinitely.
 * @param speed The speed to move backward in percentages
 */
void reverse(float dist, float speed) {
    deltaDist = (dist == 0) ? 999999 : dist;
    newDist = reverseDist + deltaDist;
    dir = BACKWARD;
    int val = pwmVal(speed);
    OCR0A = val * LEFT_REVERSE;
    OCR1B = val * RIGHT_REVERSE;
    OCR1A = 0;
    OCR0B = 0;
    TCCR0A = 0b10000001;
    TCCR1A = 0b00100001;
}

/**
 * Turn Alex left `ang` degrees at speed `speed`. To turn left we reverse the
 * left wheel and move the right wheel forward.
 * @param ang The angle to turn left in degrees. If 0, turn left indefinitely.
 * @param speed The speed to turn left in percentages
 */
void left(float ang, float speed) {
    deltaTicks = (ang == 0) ? 999999 : computeDeltaTicks(ang);
    targetTicks = leftReverseTicksTurns + deltaTicks;
    dir = LEFT;
    int val = pwmVal(speed);
    OCR0A = val * TURN_LEFT_PWM;
    OCR1A = val * TURN_LEFT_PWM;
    OCR0B = 0;
    OCR1B = 0;
    TCCR0A = 0b10000011;
    TCCR1A = 0b10000011;
}

/**
 * Turn Alex right `ang` degrees at speed `speed`. To turn right we reverse the
 * right wheel and move the left wheel forward.
 * @param ang The angle to turn right in degrees. If 0, turn right indefinitely.
 * @param speed The speed to turn right in percentages
 */
void right(float ang, float speed) {
    deltaTicks = (ang == 0) ? 999999 : computeDeltaTicks(ang);
    targetTicks = leftForwardTicksTurns + deltaTicks;
    dir = RIGHT;
    int val = pwmVal(speed);
    OCR0B = val * RIGHT_ROTATE_PWM;
    OCR1B = val * RIGHT_ROTATE_PWM;
    OCR0A = 0;
    OCR1A = 0;
    TCCR0A = 0b00100011;
    TCCR1A = 0b00100011;
}

/**
 * Stop Alex from moving.
 */
void stop() {
    dir = STOP;
    OCR0A = 0;
    OCR0B = 0;
    OCR1A = 0;
    OCR1B = 0;
}





/****************************************
 * Handle communication with RPi
 ****************************************/

/**
 * Handle the command sent from RPi.
 * @param command The command sent from RPi
 */
void handleCommand(TPacket *command) {
    switch (command->command){
        case COMMAND_FORWARD:
            sendOK();
            forward((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_REVERSE:
            sendOK();
            reverse((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_TURN_LEFT:
            sendOK();
            left((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_GET_STATS:
            sendStatus();
            break;
        case COMMAND_CLEAR_STATS:
            sendOK();
            clearCounters();
            break;
        case COMMAND_STOP:
            sendOK();
            stop();
            break;
        default:
            sendBadCommand();
    }
}

/**
 * When RPi and Arduino are connected, RPi will send a hello packet. We will
 * keep trying until we can verify the packet received is, indeed, a non-corrupted
 * hello packet. If we find the packet to be corrupted, we send error packets
 * accordingly.
 */
void waitForHello() {
    int exit = 0;
    while (!exit) {
        TPacket hello;
        TResult result;
        do {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK) {
            if (hello.packetType == PACKET_TYPE_HELLO) {
                sendOK();
                exit = 1;
            }
            else sendBadResponse();
        }
        else if (result == PACKET_BAD) sendBadPacket();
        else if (result == PACKET_CHECKSUM_BAD) sendBadChecksum();
    }
}

/**
 * Handle a packet sent from RPi. This is essentially a wrapper around handleCommand.
 * @param packet The packet sent from RPi
 */
void handlePacket(TPacket *packet) {
    freeBuffer(&RBuffer);
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;
        default:
            // All other packet types are not expected, so ignored.
            break;
    }
}





/****************************************
 * Main Arduino functions
 ****************************************/

void setup() {
    cli();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    clearCounters(); // initialize the state variables
    freeBuffer(&RBuffer);
    freeBuffer(&XmitBuffer);
    sei();
}

void loop() {
    TPacket recvPacket;
    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK) handlePacket(&recvPacket);
    else {
        if (result == PACKET_BAD) sendBadPacket();
        else if (result == PACKET_CHECKSUM_BAD) sendBadChecksum();
        freeBuffer(&RBuffer);
    }

    // Keep moving if we still need to
    if (deltaDist > 0) {
        switch (dir) {
            case FORWARD:
                if (forwardDist > newDist) {
                    deltaDist = 0;
                    newDist = 0;
                    stop();
                }
                break;
            case BACKWARD:
                if (reverseDist > newDist) {
                    deltaDist = 0;
                    newDist = 0;
                    stop();
                }
                break;
            case STOP:
                deltaDist = 0;
                newDist = 0;
                stop();
                break;
        }
    }
    if (deltaTicks > 0) {
        switch (dir) {
            case LEFT:
                if (leftReverseTicksTurns >= targetTicks) {
                    deltaTicks = 0;
                    targetTicks = 0;
                    stop();
                }
                break;
            case RIGHT:
                if (leftForwardTicksTurns >= targetTicks) {
                    deltaTicks = 0;
                    targetTicks = 0;
                    stop();
                }
                break;
            case STOP:
                deltaTicks = 0;
                targetTicks = 0;
                stop();
        }
    }
}
