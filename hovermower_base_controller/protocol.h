#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#define START_FRAME 0xDCBA
#define CMD_FRAME 0x2323
#define CHARGING 2
#define IN_STAITON 1
#define NOT_CONNNECTED 0

#define CMD_CALIBRATE 10
#define CMD_SETMOTOR 20
#define CMD_SWITCH1 30
#define CMD_SWITCH2 40
#define CMD_SWITCH3 50

typedef struct
{
   uint16_t start;
   int16_t cmd;   // 10 = calibrate, 20 = set motor, 30 = switch 1, 40 = switch 2
   int16_t value; // value of command
   uint16_t checksum;
} SerialCommand;

typedef struct
{
   uint16_t start;
   int16_t left_mag;
   int16_t right_mag;
   int16_t left_smag;
   int16_t right_smag; // 10 byte
   bool left_inside;
   bool right_inside;
   bool left_timeout;
   bool right_timeout;
   bool calibrated;
   bool bumperLeft;
   bool bumperRight;
   byte buttonCount; // 18 bytes
   int16_t batVoltage;
   int16_t chgVoltage;
   int16_t chgCurrent; // 24 bytes
   byte chgStatus;     // 0 = not connected, 1 = in Station, 2 = charging
   int16_t mowCurrent;
   int16_t mowSpeed;
   int16_t mowPower;
   bool mowAlarm;     //32 bytes
   uint16_t checksum; // 34 bytes
} SerialFeedback;
#endif
