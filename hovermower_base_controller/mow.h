/*

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

#ifndef MOW_H
#define MOW_H

#include <Arduino.h>



class Mow
{
public:
  Mow();
  // set ADC pins
  float MowCurrent;              // current flowing through motor
  float MowPower;                // Watts of mow motor
  int16_t speed;                 // actual speed in RPM
  int16_t target_speed;          // target speed of mow motor
  bool alarm;                    // alarm occured
  void init();                   // init ADC manager for Mow
  void run();                    // run mow monitor once
  void setSpeed(int speed);  // set desired speed (RPM)
  void set_count_of_ISR(int count);     // set count of ticks which has been detected by ISR for further processing

private:
  bool direction;    // rotation direction (forward backward)
  bool enable;       // enable motor driver
  bool brake;        // brake signal to stop
  int16_t zero_point = 2500; // sensor value if no current flows (0A)
  int16_t VpA = 100;        // milli Volt per Ampere (185 for 5A module, 100 for 20A module, 66 for 30A module)
  int16_t MowCount;         // counter of ISR routine
  void check_current(); // measure current of mow motor
  void check_alarm();  // checks if alarm occured  
};

#endif
