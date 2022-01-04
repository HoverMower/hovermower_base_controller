/*
Base controller firmware for HoverMower

As stated in documentation of HoverMower project, firmware depends on your needs. 
However, my build of HoverMower uses ROS (robot operating system). 
Tasks like path planning, move commands and behavior gets managet by ROS nodes.

But some really basic parts can't be handeled by ROS, this is where hovermower_bare_controller comes
in play. It is a firmware which runs on Arduino Nano and will perform thinks like:

- battery monitor, sense battery voltage and init shutdown/disconnect battery before drawing it to death
- charge monitor, sense if robot is attached to charger and charge battery, if needed.
  Also disconnect charger when battery is full
- perimeter sensor, detect perimeter fence and detect, if robot leaves the mowing area
- bump sensor, simple bumper to detect collisions
- mow motor, control mow motor, sense power usage of mow motor
- user switches, control MOSFET switches to toggle headlights or other payload on/off
- button, user controlled button, analyze if button has been pressed and for how long

All values gets reported to a ROS node,m which will publish corresponding messages. 
This firmware will not take any decisions beside toggle charger on/off. Instead, 
it is just a wrapper to abstract hardware specific implementation to ROS

THIS SOFTWARE IS BASED ON ARDUMOWER AZURIT, www.ardumower.de

  Private-use only! (you need to ask for a commercial-use)

  The code is open: you can modify it under the terms of the
  GNU General Public License as published by the Free Software Foundation,
  either version 3 of the License, or (at your option) any later version.

  The code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)


*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include "drivers.h"
#include "adcman.h"
#include "perimeter.h"
#include "config.h"
#include "protocol.h"
#include "serial_if.h"
#include "battery.h"
#include "mow.h"

Perimeter perimeter;
unsigned long nextTime = 0;
int counter = 0;
boolean inside = true;
int mode = 0;
static SerialFeedback feedback;

// Button
byte buttonCounter;
byte lastButtonCount;
unsigned long nextTimeButtonCheck;
unsigned long nextTimeButton;
// Bumper sensor
boolean bumperLeft;
boolean bumperRight;
unsigned long nextTimeBumper;

// Battery
unsigned long nextTimeBatteryCheck;
Battery battery;

// Mow Motor
unsigned long nextTimeMowCheck;
//unsigned long enableMowTime; debug only
int mow_counter;
Mow mow;

// ISR routine for Mow motor, counting ticks of motor
void interruptHandler()
{
  mow_counter++;
}

/*--------------------------------------------
  Setup Arduino
 -------------------------------------------- */
void setup()
{
  // keep battery connected
  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

  Wire.begin();
  Serial.begin(SERIAL_BAUDRATE);
  //  pinMode(pinLED, OUTPUT);

  Serial.println("START");

  ADCMan.init();


  // init Perimeter
  perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);
  perimeter.useDifferentialPerimeterSignal = true;
  perimeter.speedTest();

  // Battery
  battery.init();
  
  // button
  pinMode(pinButton, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  pinMode(pinBumperLeft, INPUT);
  pinMode(pinBumperLeft, INPUT_PULLUP);
  pinMode(pinBumperRight, INPUT);
  pinMode(pinBumperRight, INPUT_PULLUP);

  // mow interrupt handler
  mow_counter = 0;
  attachInterrupt(digitalPinToInterrupt(pinMowSpeed), interruptHandler, RISING);
  mow.init();

  buttonCounter = 0;
  lastButtonCount = 0;
  bumperLeft = false;
  bumperRight = false;
  nextTimeButtonCheck = millis() + 50;
  nextTimeBumper = millis() + 50;
  nextTimeBatteryCheck = millis() + 50;
  nextTimeMowCheck = millis() + 50;

  // enableMowTime = millis() + 10000;  // debug only
  Serial.println("Setup complete");

    //if (DEBUG_OUTPUT) ADCMan.calibrate();
}
/*--------------------------------------------
  Main Loop
 -------------------------------------------- */
void loop()
{
  ADCMan.run();
  readSerial();

  if (BUTTON)
    checkButton();
  if (BUMPER)
    checkBumper();

  // Mow motor monitor
  if (MOW && millis() >= nextTimeMowCheck)
  {
    nextTimeMowCheck = millis() + 100;
    mow.set_count_of_ISR(mow_counter);
    mow_counter = 0;
    mow.run();
  }

  // debug mow
  /*if (millis() >= enableMowTime )
  {
    enableMowTime = millis() + 10000;
    if (mow.speed > 0)
    {
      mow.setSpeed(0);
    } 
    else
    {
    mow.setSpeed(3000);
    }
  } */

  // Battery monitor
  if (BATMON && millis() >= nextTimeBatteryCheck)
  {
    nextTimeBatteryCheck = millis() + 1000;
    battery.run();
  }

  /* Set info to ROS */
  if (millis() >= nextTime)
  {
    nextTime = millis() + 1000 / SERIAL_RATE;
    if (DEBUG_OUTPUT)
    {
     // printSerialPeri();
    //  printSerialOthers();
    }
    else
    {
      sendMessage();
    }
    lastButtonCount = 0;
  }
}

/*--------------------------------------------
  Read byte from serial console
 -------------------------------------------- */

void readSerial()
{

  unsigned char c;
  int i = 0, r = 0;
  while (Serial.available() > 0 && i++ < 128)
  {
    c = Serial.read();
    protocol_recv(c);
  }

  // Check if timeout of serial commands occured. If so, stop mow motor for safety reasons
  if (millis() - last_time_command >= SERIAL_READ_TIMEOUT )
  {
   //   if(MOW) mow.setSpeed(0);
  }
}
/*--------------------------------------------
  Collect incoming bytes to command structure
 -------------------------------------------- */
void protocol_recv(unsigned char byte)
{
  cmd_frame = ((uint16_t)(byte) << 8) | prev_byte;

  // Read the start frame of cmd structure
  if (cmd_frame == CMD_FRAME && msg_len == 0)
  {
    p = (unsigned char *)&cmd_msg;
    *p++ = prev_byte;
    *p++ = byte;
    msg_len = 2;
  }
  else if (msg_len >= 2 && msg_len < sizeof(SerialCommand))
  {
    // Otherwise just read the message content until the end
    *p++ = byte;
    msg_len++;
  }

  if (msg_len == sizeof(SerialCommand))
  {

    uint16_t checksum = (uint16_t)(cmd_msg.start ^
                                   cmd_msg.cmd ^
                                   cmd_msg.value);

Serial.print("Checksum: ");
Serial.println(checksum);
Serial.println(cmd_msg.cmd);
Serial.println(cmd_msg.value);
    //if (cmd_msg.start == CMD_FRAME && cmd_msg.checksum == checksum)
    if (cmd_msg.start == CMD_FRAME)
    {
      Serial.println("checksum ok");
      // valid command
      last_time_command = millis();

      // process command
      switch (cmd_msg.cmd)
      {
      case CMD_CALIBRATE:
        ADCMan.calibrate();
        break;
      case CMD_SETMOTOR:
        if(MOW) mow.setSpeed(int(cmd_msg.value));
        break;

      case CMD_SWITCH1:
        digitalWrite(pinSwitch1, bool(cmd_msg.value));
        break;
      case CMD_SWITCH2:
        digitalWrite(pinSwitch2, bool(cmd_msg.value));
        break;
      }
    }
    else
    {
      Serial.println("checksum error");
      // implement error handler here
    }
    msg_len = 0;
  }
  prev_byte = byte;
}

/*--------------------------------------------
  Check and count button press
 -------------------------------------------- */
void checkButton()
{
  if ((millis() < nextTimeButtonCheck))
    return;

  nextTimeButtonCheck = millis() + 50;
  boolean buttonPressed = (digitalRead(pinButton) == LOW);

  if (((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)))
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed)
    {
      buttonCounter++;
    }
    else
    {
      lastButtonCount = buttonCounter;
      // Button has been released, buttonCounter has correct value now
      buttonCounter = 0;
    }
  }
}

/*--------------------------------------------
  Check if bumoper gets triggered
 -------------------------------------------- */
void checkBumper()
{
  if ((millis() >= nextTimeBumper))
  {
    nextTimeBumper = millis() + 50;

    bumperLeft = (digitalRead(pinBumperLeft) == HIGH);
    bumperRight = (digitalRead(pinBumperRight) == HIGH);
  }
}

/*--------------------------------------------
  Send feedback message to ROS
 -------------------------------------------- */
void sendMessage()
{

  feedback.start = (uint16_t)START_FRAME;
  feedback.left_mag = (int16_t)perimeter.getMagnitude(0);
  feedback.right_mag = (int16_t)perimeter.getMagnitude(1);
  feedback.left_smag = (int16_t)perimeter.getSmoothMagnitude(0);
  feedback.right_smag = (int16_t)perimeter.getSmoothMagnitude(1);
  feedback.left_inside = (bool)perimeter.isInside(0);
  feedback.right_inside = (bool)perimeter.isInside(1);
  feedback.left_timeout = (bool)perimeter.signalTimedOut(0);
  feedback.right_timeout = (bool)perimeter.signalTimedOut(1);
  feedback.calibrated = (bool)ADCMan.calibrationDataAvail();
  feedback.bumperLeft = bumperLeft;
  feedback.bumperRight = bumperRight;
  feedback.buttonCount = lastButtonCount;
  feedback.batVoltage = (int16_t)(battery.batVoltage * 100);
  feedback.chgVoltage = (int16_t)(battery.chgVoltage * 100);
  feedback.chgCurrent = (int16_t)(battery.chgCurrent * 100);
  feedback.chgStatus = (byte)NOT_CONNNECTED;
  if (battery.charging == true)
    feedback.chgStatus = (byte)CHARGING;
  if (battery.charging == false && battery.inStation == true)
    feedback.chgStatus = (byte)IN_STAITON;
  feedback.mowCurrent = (int16_t)(mow.MowCurrent * 100);
  feedback.mowPower = (int16_t)(mow.MowPower * 100);
  feedback.mowSpeed = (int16_t)mow.speed;
  feedback.mowAlarm = mow.alarm;

  feedback.checksum = (uint16_t)(feedback.start ^ feedback.left_mag ^ feedback.right_mag ^ feedback.left_smag ^ feedback.right_smag ^ feedback.left_inside ^ feedback.right_inside ^
                                 feedback.left_timeout ^ feedback.right_timeout ^ feedback.calibrated ^ feedback.bumperLeft ^ feedback.bumperRight ^ feedback.buttonCount ^
                                 feedback.batVoltage ^ feedback.chgVoltage ^ feedback.chgCurrent ^ feedback.chgStatus ^
                                 feedback.mowCurrent ^ feedback.mowPower ^ feedback.mowSpeed ^ feedback.mowAlarm);

  Serial.write((uint8_t *)&feedback, sizeof(feedback));
}

/*--------------------------------------------
  Output Debug information about Perimeter
 -------------------------------------------- */
void printSerialPeri()
{
  Serial.print("mag ");
  Serial.print((int)perimeter.getMagnitude(0));
  Serial.print(",");
  Serial.print((int)perimeter.getMagnitude(1));
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("smag ");
  Serial.print((int)perimeter.getSmoothMagnitude(0));
  Serial.print(",");
  Serial.print((int)perimeter.getSmoothMagnitude(1));
  Serial.print("\t");
  Serial.print("in ");
  Serial.print((int)perimeter.isInside(0));
  Serial.print(",");
  Serial.print((int)perimeter.isInside(1));
  Serial.print("\t");
  Serial.print("on ");
  Serial.print((int)(!perimeter.signalTimedOut(0)));
  Serial.print(",");
  Serial.print((int)(!perimeter.signalTimedOut(1)));
  Serial.print("\t");
  Serial.print("adc ");
  Serial.print((int)(ADCMan.getCapturedChannels()));
  Serial.print("\t");
  Serial.print("bumL");
  Serial.print((int)bumperLeft);
  Serial.print("\t");
  Serial.print("bumR");
  Serial.print((int)bumperRight);
  Serial.print("\t");
  Serial.print("btn ");
  Serial.print((int)lastButtonCount);
  Serial.println();
}

/*--------------------------------------------
  Output Debug information about other values
 -------------------------------------------- */
void printSerialOthers()
{
  Serial.print("BatVcc ");
  Serial.print(battery.batVoltage);
  Serial.print("\t");
  Serial.print("ChgVcc ");
  Serial.print(battery.chgVoltage);
  Serial.print("\t");
  Serial.print("ChgA ");
  Serial.print(battery.chgCurrent);
  Serial.print("\t");
  Serial.print("Station ");
  Serial.print(battery.inStation);
  Serial.print("\t");
  Serial.print("Charging ");
  Serial.print(battery.charging);
  Serial.print("\t");
  Serial.print("MowTgtSpd ");
  Serial.print(mow.target_speed);
  Serial.print("\t");
  Serial.print("MowSpd ");
  Serial.print(mow.speed);
  Serial.print("\t");
  Serial.print("MowA ");
  Serial.print(mow.MowCurrent);
  Serial.print("\t");
  Serial.print("MowP ");
  Serial.print(mow.MowPower);
  Serial.print("\t");
  Serial.println();
}
