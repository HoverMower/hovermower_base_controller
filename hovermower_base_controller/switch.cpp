/*
  Class to handle user switches


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

#include "switch.h"
#include <Arduino.h>
#include "config.h"

Switch::Switch()
{
  // initialize pins
  pinMode(pinSwitch1, OUTPUT);
  pinMode(pinSwitch2, OUTPUT);
  pinMode(pinSwitch3, OUTPUT);

  // Initialize attributes
  switch1 = 0;
  switch2 = 0;
  switch3 = 0;
}

void Switch::run()
{
  analogWrite(pinSwitch1, switch1);
  analogWrite(pinSwitch2, switch2);
  analogWrite(pinSwitch3, switch3);
}

void Switch::setSwitch(int switchID, int value)
{
  if (value < 0 || value > 255)
  {
    value = 0;
  }

  switch (switchID)
  {
    case SWITCH1_ID:
      switch1 = value;
      break;

    case SWITCH2_ID:
      switch2 = value;
      break;

    case SWITCH3_ID:
      switch3 = value;
      break;
  }
}
