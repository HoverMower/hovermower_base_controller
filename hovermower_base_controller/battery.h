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

#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

#ifdef __AVR__
  #define IOREF 5.0  // I/O reference voltage for Mega 2560
#else 
	#define IOREF 3.3   // I/O reference voltage for Due
#endif

class Battery
{
public:
  Battery();
  // set ADC pins
  float batVoltage;
  float batSwitchOffIfBelow;     // switch off if below voltage (Volt)
  int batSwitchOffIfIdle;        // switch off battery if idle for minutes
  //float batFull;                 // battery reference Voltage (fully charged)
  float batFullCurrent;          // current flowing when battery is fully charged
  float startChargingIfBelow;    // start charging if battery Voltage is below
  float chgVoltage;              // charge voltage (Volt)
  float chgCurrent;              // charge current  (Ampere)
  bool charging;                 // charging battery?
  bool inStation;                // robot in station?
  unsigned long chargingTimeout; // safety timer for charging
  void run();                    // run battery monitor once


private:
  float batFactor;    // battery conversion factor
  float batChgFactor; // battery conversion factor
  float chgFactor;    // charge current conversion factor
  unsigned long lastTimeChargeToggle; // 

  void check_battery_voltage();  // read current battery voltage
  void check_charger();          // read current and voltage of charge contacts
  float ADC2voltage(float ADCvalue);
  float voltageDividerUges(float R1, float R2, float U2);
};

#endif
