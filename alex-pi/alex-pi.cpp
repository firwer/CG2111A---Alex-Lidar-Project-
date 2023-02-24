/**
 * Handle connection between RPi and Alex.
 * This program doesn't require any additional flags, so to run simply do ./alex-pi
 */

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

#define PORT_NAME "/dev/ttyACM0"
#define BAUD_RATE B9600

int exitFlag = 0;
sem_t _xmitSema;

/****************************************
 * Functions to receive packets from Alex
 ****************************************/

/**
 * Handle any errors caused by communication issues, by printing the error message.
 * @param error The error code
 */
void handleError(TResult error) {
    switch (error) {
        case PACKET_BAD:
            printf("ERROR: Bad Magic Number\n");
            break;
        case PACKET_CHECKSUM_BAD:
            printf("ERROR: Bad checksum\n");
            break;
        default:
            printf("ERROR: UNKNOWN ERROR\n");
    }
}

/**
 * Print the current Alex's reading
 * @param packet The packet received from Alex where the reading is contained in the params field.
 */
void handleStatus(TPacket *packet) {
    printf("\n--------- ALEX STATUS REPORT ----------\n\n");
    printf("Left Forward Ticks:             %d\n", packet->params[0]);
    printf("Right Forward Ticks:            %d\n", packet->params[1]);
    printf("Left Reverse Ticks:             %d\n", packet->params[2]);
    printf("Right Reverse Ticks:            %d\n", packet->params[3]);
    printf("Left Forward Ticks Turns:       %d\n", packet->params[4]);
    printf("Right Forward Ticks Turns:      %d\n", packet->params[5]);
    printf("Left Reverse Ticks Turns:       %d\n", packet->params[6]);
    printf("Right Reverse Ticks Turns:      %d\n", packet->params[7]);
    printf("Forward Distance:               %d\n", packet->params[8]);
    printf("Reverse Distance:               %d\n", packet->params[9]);
    printf("\n---------------------------------------\n\n");
}

/**
 * Handle a response from Alex based on its type (stored in command)
 * @param packet The packet received from Alex.
 */
void handleResponse(TPacket *packet) {
    switch (packet->command) {
        case RESP_OK:
            printf("Command OK\n");
            break;
        case RESP_STATUS:
            handleStatus(packet);
            break;
        default:
            printf("Arduino is confused\n");
    }
}

/**
 * Handle an error packet received from Alex. The error type is stored in command.
 * @param packet The packet received from Alex.
 */
void handleErrorResponse(TPacket *packet) {
    switch (packet->command) {
        case RESP_BAD_PACKET:
            printf("Arduino received bad magic number\n");
            break;
        case RESP_BAD_CHECKSUM:
            printf("Arduino received bad checksum\n");
            break;
        case RESP_BAD_COMMAND:
            printf("Arduino received bad command\n");
            break;
        case RESP_BAD_RESPONSE:
            printf("Arduino received unexpected response\n");
            break;
        default:
            printf("Arduino reports a weird error\n");
    }
}

/**
 * Print a message from Alex if we receive one.
 * @param packet The packet received from Alex.
 */
void handleMessage(TPacket *packet) {
    printf("Message from Alex: %s\n", packet->data);
}

/**
 * Handle a packet received from Alex.
 * @param packet The packet received from Alex.
 */
void handlePacket(TPacket *packet) {
    switch (packet->packetType) {
        case PACKET_TYPE_COMMAND:
            // Only we send command packets, so ignore
            break;
        case PACKET_TYPE_RESPONSE:
            handleResponse(packet);
            break;
        case PACKET_TYPE_ERROR:
            handleErrorResponse(packet);
            break;
        case PACKET_TYPE_MESSAGE:
            handleMessage(packet);
            break;
    }
}

/**
 * Set up a thread to receive packets from Alex.
 */
void *receiveThread(void *p) {
    char buffer[PACKET_SIZE];
    int len;
    TPacket packet;
    TResult result;
    int counter = 0;

    // Always listening to packets
    while (1) {
        len = serialRead(buffer);
        counter += len;
        if (len > 0) {
            result = deserialize(buffer, len, &packet);
            if (result == PACKET_OK) {
                counter = 0;
                handlePacket(&packet);
            } else if(result != PACKET_INCOMPLETE) {
                printf("PACKET ERROR\n");
                handleError(result);
            }
        }
    }
}

/****************************************
 * Functions to send packets to Alex
 ****************************************/

/**
 * Send a command packet to Alex. We serialize the input packet and send it to the serial port.
 * @param packet The packet to send to Alex.
 */
void sendPacket(TPacket *packet) {
    char buffer[PACKET_SIZE];
    int len = serialize(buffer, packet, sizeof(TPacket));
    serialWrite(buffer, len);
}

/**
 * Flush stdin to prevent the user from accidentally perform unintended actions by redundant characters
 */
void flushInput() {
    char c;
    while ((c = getchar()) != '\n' && c != EOF);
}

/**
 * Get parameters for the parameterized commands: F, B, L and R. We ask for two numbers: the distance
 * or angle and the power to use, then put them inside the packet that we are going to send to Alex.
 * @param commandPacket The packet to send to Alex, whose parameters we are asking users to input.
 */
void getParams(TPacket *commandPacket) {
    printf("Input distance (cm)/angle (deg) and power in %% as integers: ");
    scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
    flushInput();
}

/**
 * Send a command packet to Alex. We ask the user to input the parameters for the command, then send
 * the packet to Alex.
 *
 * The following commands are supported:
 *
 * - 'f': Move forward with parameters from stdin
 * - 'w': Move forward shortcut by 5cm with 80% power
 * - 'b': Move backward with parameters from stdin
 * - 's': Move backward shortcut by 5cm with 80% power
 * - 'l': Turn left with parameters from stdin
 * - 'a': Turn left shortcut by 30 degrees with 80% power
 * - 'r': Turn right with parameters from stdin
 * - 'd': Turn right shortcut by 30 degrees with 80% power
 * - 'x': Stop (we cannot use s since it is already used as a backward shortcut)
 * - 'c': Clear current readings in Alex
 * - 'g': Get current readings in Alex
 * - 'q': Quit the program
 *
 * Uppercase version of these commands are *not* supported since we never use them anyway.
 *
 * @param command The command to send to Alex, as a character as described above
 */
void sendCommand(char command) {
    TPacket commandPacket;
    commandPacket.packetType = PACKET_TYPE_COMMAND;
    switch (command) {
        case 'f':
            getParams(&commandPacket);
            commandPacket.command = COMMAND_FORWARD;
            sendPacket(&commandPacket);
            break;
        case 'w':
            commandPacket.command = COMMAND_FORWARD;
            commandPacket.params[0] = 5;
            commandPacket.params[1] = 80;
            sendPacket(&commandPacket);
            break;
        case 'b':
            getParams(&commandPacket);
            commandPacket.command = COMMAND_REVERSE;
            sendPacket(&commandPacket);
            break;
        case 's':
            commandPacket.command = COMMAND_REVERSE;
            commandPacket.params[0] = 5;
            commandPacket.params[1] = 80;
            sendPacket(&commandPacket);
            break;
        case 'l':
            getParams(&commandPacket);
            commandPacket.command = COMMAND_TURN_LEFT;
            sendPacket(&commandPacket);
            break;
        case 'a':
            commandPacket.command = COMMAND_TURN_LEFT;
            commandPacket.params[0] = 30;
            commandPacket.params[1] = 80;
            sendPacket(&commandPacket);
            break;
        case 'r':
            getParams(&commandPacket);
            commandPacket.command = COMMAND_TURN_RIGHT;
            sendPacket(&commandPacket);
            break;
        case 'd':
            commandPacket.command = COMMAND_TURN_RIGHT;
            commandPacket.params[0] = 30;
            commandPacket.params[1] = 80;
            sendPacket(&commandPacket);
            break;
        case 'x':
            commandPacket.command = COMMAND_STOP;
            sendPacket(&commandPacket);
            break;
        case 'c':
            commandPacket.command = COMMAND_CLEAR_STATS;
            commandPacket.params[0] = 0;
            sendPacket(&commandPacket);
            break;
        case 'g':
            commandPacket.command = COMMAND_GET_STATS;
            sendPacket(&commandPacket);
            break;
        case 'q':
            exitFlag = 1;
            break;
        default:
            printf("Bad command\n");
    }
}


int main() {
    // Initialize connection to Arduino
    startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
    printf("Waiting 2 seconds for Arduino to reboot\n");
    sleep(2);
    printf("Done\n");

    // Spawn receiver thread
    pthread_t recv;
    pthread_create(&recv, NULL, receiveThread, NULL);

    // Send a hello packet to see if the connection is working
    TPacket helloPacket;
    helloPacket.packetType = PACKET_TYPE_HELLO;
    sendPacket(&helloPacket);

    // We always listen to user input, except when 'q' is received
    while (!exitFlag) {
        char ch;
        printf("Input command: ");
        scanf("%c", &ch);
        flushInput();
        sendCommand(ch);
    }

    printf("Closing connection to Arduino.\n");
    endSerial();
}
