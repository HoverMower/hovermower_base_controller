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

#ifndef SWITCH_H
#define SWITCH_H

#include <Arduino.h>

#define SWITCH1_ID 1
#define SWITCH2_ID 2
#define SWITCH3_ID 3


class Switch
{
public:
  Switch();
  void run();
  void setSwitch(int switchID, int value);
  uint8_t switch1;
  uint8_t switch2;
  uint8_t switch3;

};

#endif
