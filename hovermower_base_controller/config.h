
#ifndef CONFIG_H
#define CONFIG_H

/*------- Serial Interface -----*/
#define SERIAL_BAUDRATE 115200
#define SERIAL_RATE 50 // publish rate of messages in Hz
#define SERIAL_READ_TIMEOUT 1000 // at least one CMD message per second should occur

/*-------- DEBUG Section ----------*/
#define DEBUG_OUTPUT false        // Activate this for tests to get human readable data, DISABLE when using as ROS Node

/*------- Perimeter Definitions ------*/
// ---- choose only one perimeter signal code ----
#define SIGCODE_1  // Ardumower default perimeter signal
//#define SIGCODE_2  // Ardumower alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal
#define pinPerimeterLeft A5        
#define pinPerimeterRight A4        
// #define pinLED 13               // Indicator if left is in/out
#define SWAP_COIL_POLARITY_LEFT false;
#define SWAP_COIL_POLARITY_RIGHT true;

/*----- user Button ---------*/
#define BUTTON true
#define pinButton 6

/*----- user Switches -------*/
#define pinSwitch1 3
#define pinSwitch2 8
#define pinSwitch3 A7

/*----- Bumper -----------*/
#define BUMPER true
#define pinBumperLeft  4
#define pinBumperRight 5

/*----- BatteryMonitor -----------*/
#define BATMON true
#define BAT_SWITCH_OFF 30.5     // switch off if bat voltage below
#define BAT_FULL_CURRENT 0.2    // current flowing into system, when battery is fully charged
#define BAT_START_CHARGE 41.0   // start charging if battery voltage below

#define pinBatterySwitch  13
#define pinBatteryVoltage A3
#define pinChargeCurrent A0
#define pinChargeVoltage A1
#define pinChargeEnable A2

/*------ Mow Motor --------- */
#define MOW true
#define pinMowCurrent A6
#define pinMowPWM 12
#define pinMowDirection 11
#define pinMowEnable 10
#define pinMowBreak 9
#define pinMowSpeed 2
#define pinMowAlarm 7

#endif
