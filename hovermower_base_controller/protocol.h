#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#define START_FRAME 0xDCBA
#define CMD_FRAME 0x2324
#define CHARGING 2
#define IN_STAITON 1
#define NOT_CONNNECTED 0

typedef struct
{
   uint16_t start;
   uint16_t mow_rpm;   // RPM of mow motor
   uint8_t switch1; // value of switch 1
   uint8_t switch2; // value of switch 1
   uint8_t switch3; // value of switch 1
   bool calibrate; // start calibration
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
   bool mowAlarm;     
   int16_t mowCurrent;
   int16_t mowSpeed;
   int16_t mowPower;
   uint8_t switch1; // value of switch 1
   uint8_t switch2; // value of switch 1
   uint8_t switch3; // value of switch 1
   bool calibrate; // start calibration
   uint16_t checksum; // 34 bytes
} SerialFeedback;
#endif
