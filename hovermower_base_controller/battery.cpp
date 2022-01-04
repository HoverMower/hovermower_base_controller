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

#include "battery.h"
#include <Arduino.h>
#include "adcman.h"
#include "config.h"

Battery::Battery()
{

  // initialize pins
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeEnable, OUTPUT);

  // Initialize attributes
  batVoltage = 0.0;
  batSwitchOffIfBelow = BAT_SWITCH_OFF;
  batSwitchOffIfIdle = 60; // switch off battery if idle for minutes
  //batFull = 42.0;              // battery reference Voltage (fully charged)
  batFullCurrent = BAT_FULL_CURRENT;        // current flowing when battery is fully charged
  startChargingIfBelow = BAT_START_CHARGE; // start charging if battery Voltage is below

  batFactor = voltageDividerUges(150, 10, 1.0) * ADCMan.ADC2voltage(1); // ADC to battery voltage factor
  batChgFactor = voltageDividerUges(150, 10, 1.0) * ADCMan.ADC2voltage(1); // ADC to battery voltage factor ;               // charge current conversion factor
  chgFactor                  = ADCMan.ADC2voltage(1);
  charging = false;
  inStation = false;
  lastTimeChargeToggle = millis();

}

void Battery::init()
{
  ADCMan.setCapture(pinChargeCurrent, 1, true);//Aktivierung des LadeStrom Pins beim ADC-Managers
  ADCMan.setCapture(pinBatteryVoltage, 1, false);
  ADCMan.setCapture(pinChargeVoltage, 1, false);

}

void Battery::run()
{
  check_battery_voltage();
  check_charger();

  // check for drawn battery
  if (batVoltage <= batSwitchOffIfBelow)
  {
    digitalWrite(pinBatterySwitch, LOW);
  }

  // check if robot is in station
  if (chgVoltage > 10)
  {
    inStation = true;
  } 
  else
  { 
    inStation = false;
    charging = false;
  }

  // need to start charging?
  if (charging == false)
  {
    if (batVoltage < startChargingIfBelow && millis() - lastTimeChargeToggle > 30000)
    {
      charging = true;
      digitalWrite(pinChargeEnable, HIGH);
      lastTimeChargeToggle = millis();
    }
  }
  // already charging, check if battery is full
  else
  {
    if (chgCurrent <= batFullCurrent && millis() - lastTimeChargeToggle > 30000)
    {
      charging = false;
      digitalWrite(pinChargeEnable, LOW);
    }
  }
}

void Battery::check_battery_voltage()
{
  // convert to double
  int batADC =  ADCMan.read(pinBatteryVoltage);
  double batvolt = ((double)batADC) * batFactor;

  // low-pass filter
  double accel = 0.01;
  //double accel = 1.0;
  if (abs(batVoltage - batvolt) > 5)
    batVoltage = batvolt;
  else
    batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;

}

void Battery::check_charger()
{
  int currentADC = ADCMan.read(pinChargeCurrent);
  int chgADC = ADCMan.read(pinChargeVoltage);

  double chgvolt = ((double)chgADC) * batChgFactor;
  double curramp = ((double)currentADC) * chgFactor;

  // low-pass filter
  double accel = 0.01;
  //double accel = 1.0;
  if (abs(chgVoltage - chgvolt) > 5)
    chgVoltage = chgvolt;
  else
    chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
  if (abs(chgCurrent - curramp) > 0.5)
    chgCurrent = curramp;
  else
    chgCurrent = (1.0 - accel) * chgCurrent + accel * curramp;
}

// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float Battery::voltageDividerUges(float R1, float R2, float U2)
{
  return (U2 / R2 * (R1 + R2)); // Uges
}
