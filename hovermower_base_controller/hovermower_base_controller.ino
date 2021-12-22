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
#include "battery.h"

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


void setup()
{
  // keep battery connected  
  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);

  Wire.begin();
  Serial.begin(SERIAL_BAUDRATE);
  //  pinMode(pinLED, OUTPUT);

  delay(100);
  Serial.println("START");

  ADCMan.init();

  // init Perimeter
  perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);
  perimeter.useDifferentialPerimeterSignal = true;
  perimeter.speedTest();

  Serial.println("press...");
  Serial.println("  v to toggle between serial chart/Serial output");
  Serial.println("  c to calibrate zero point (sender must be off!)");
  delay(1000);

  // button
  pinMode(pinButton, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  pinMode(pinBumperLeft, INPUT);
  pinMode(pinBumperLeft, INPUT_PULLUP);
  pinMode(pinBumperRight, INPUT);
  pinMode(pinBumperRight, INPUT_PULLUP);

  // battery
  pinMode(pinBatteryVoltage, INPUT);        
  pinMode(pinChargeCurrent, INPUT);          
  pinMode(pinChargeVoltage, INPUT);            
  pinMode(pinChargeEnable, OUTPUT);


  buttonCounter = 0;
  lastButtonCount = 0;
  bumperLeft = false;
  bumperRight = false;
  nextTimeButtonCheck = millis() + 50;
  nextTimeBumper = millis() + 50;
  nextTimeBatteryCheck = millis() + 50;

}

void printSerial()
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
  feedback.checksum = (uint16_t)(feedback.start ^ feedback.left_mag ^ feedback.right_mag ^ feedback.left_smag ^ feedback.right_smag ^ feedback.left_inside ^ feedback.right_inside ^
                                 feedback.left_timeout ^ feedback.right_timeout ^ feedback.calibrated ^ feedback.bumperLeft ^ feedback.bumperRight ^ feedback.buttonCount);

  Serial.write((uint8_t *)&feedback, sizeof(feedback));
}

void loop()
{

  ADCMan.run();

  if (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == 'v')
      mode = !mode;
    if (ch == 'c')
    {
      Serial.println("calibrating ADC (power off sender for this!)...");
      Serial.flush();
      delay(5000);
      ADCMan.calibrate();
    }
  }

  if (BUTTON)
    checkButton();
  if (BUMPER)
    checkBumper();
  
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
      printSerial();
    }
    else
    {
      sendMessage();
    }
    lastButtonCount = 0;
  }
}

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

void checkBumper()
{
  if ((millis() >= nextTimeBumper))
  {
    nextTimeBumper = millis() + 50;

    bumperLeft = (digitalRead(pinBumperLeft) == HIGH);
    bumperRight = (digitalRead(pinBumperRight) == HIGH);
  }
}

