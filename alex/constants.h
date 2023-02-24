/**
 * Constants used to identify the different types of packets, responses and
 * commands
 */

#ifndef __CG_CONSTANTS_H__
#define __CG_CONSTANTS_H__

/**
 * Packet type constants.
 */
typedef enum {
    PACKET_TYPE_COMMAND = 0,
    PACKET_TYPE_RESPONSE = 1,
    PACKET_TYPE_ERROR = 2,
    PACKET_TYPE_MESSAGE = 3,
    PACKET_TYPE_HELLO = 4
} TPacketType;

/**
 * Response type constant, used in the command field.
 */
typedef enum {
    RESP_OK = 0,
    RESP_STATUS=1,
    RESP_BAD_PACKET = 2,
    RESP_BAD_CHECKSUM = 3,
    RESP_BAD_COMMAND = 4,
    RESP_BAD_RESPONSE = 5 
} TResponseType;

/**
 * Commands constants
 *
 * For direction commands, param[0] = distance in cm to move and
 * param[1] = speed in percentage. For turn commands, param[0] = angle in
 * degrees to turn instead
 */
typedef enum {
    COMMAND_FORWARD = 0,
    COMMAND_REVERSE = 1,
    COMMAND_TURN_LEFT = 2,
    COMMAND_TURN_RIGHT = 3,
    COMMAND_STOP = 4,
    COMMAND_GET_STATS = 5,
    COMMAND_CLEAR_STATS = 6
} TCommandType;

#endif
