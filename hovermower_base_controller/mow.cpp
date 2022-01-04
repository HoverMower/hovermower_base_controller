/*
  Mow motor driver for BLD-300B motor driver


  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

#include "mow.h"
#include <Arduino.h>
#include "adcman.h"
#include "config.h"

Mow::Mow()
{
  // initialize pins
  pinMode(pinMowAlarm, INPUT_PULLUP);
  pinMode(pinMowBreak, OUTPUT);
  pinMode(pinMowCurrent, INPUT);
  pinMode(pinMowPWM, OUTPUT);
  pinMode(pinMowDirection, OUTPUT);
  pinMode(pinMowEnable, OUTPUT);
  pinMode(pinMowSpeed, INPUT);
  
  
  // Initialize attributes
  MowCurrent = 0.0;
  speed = 0;
  target_speed = 0;
  alarm = false;
  direction = 0;
  enable = 0;
  brake = 0;
}

void Mow::init()
{
  ADCMan.setCapture(pinMowCurrent, 1, true);
}

void Mow::run()
{
  check_current();
  check_alarm();
  if (alarm == true)
  {
   // target_speed = 0; 
  }
}

void Mow::setSpeed(int new_speed)
{
  int pwm = 0;
  Serial.println(new_speed);
  if (new_speed == 0)
  {
     Serial.println(direction);
    direction = !direction;
     Serial.println(direction);
    enable = false;
    brake = true;
  }
  else 
  {
    enable = true;
    brake = false;
  }
  
  target_speed = new_speed;
  pwm = map(target_speed, 1, 3500, 1, 255);
  digitalWrite(pinMowDirection, direction);
 
  analogWrite(pinMowPWM, pwm);
}

void Mow::check_current()
{
  int currentADC = ADCMan.read(pinMowCurrent);       // read raw value
  double sensor_volt = (currentADC / 1024.0) * 5000; // get sensor output voltage
  MowCurrent = abs((sensor_volt - zero_point) / VpA);   // calculate Ampere
}

void Mow::check_alarm()
{
  alarm = digitalRead(pinMowAlarm);
}

void Mow::set_count_of_ISR(int count)
{

  MowCount = count;

  // F = N*P/60; F = frequency, P num of poles
  // N = 60 * f / P
  //speed = count * (delta_t/1000);  // U/min
  speed = 60 * count;
  
}
