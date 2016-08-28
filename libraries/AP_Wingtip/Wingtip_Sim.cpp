/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
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
 */

/*
  backend driver for the Wingtip X4 Board
 */
#include "Wingtip_Sim.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
//#include <utility>

extern const AP_HAL::HAL &hal;

AP_Wingtip_Sim::AP_Wingtip_Sim(AP_Wingtip &_wingtip, uint8_t instance, AP_Wingtip::Wingtip_State &_state)
    : AP_Wingtip_Backend(_wingtip, instance, _state)
{
    state.instance = instance;

    state.enabled = init();

    if (state.enabled) {
        hal.console->printf("    Wingtip_x4 sensor %u [ x ]\n",state.instance);
    } else {
        hal.console->printf("    Wingtip_x4 sensor %u [   ]\n",state.instance);
    }
}

AP_Wingtip_Sim::~AP_Wingtip_Sim()
{
  // do nothing
}

bool AP_Wingtip_Sim::init()
{
    // Initialise all states
    state.rpm[0] = 0;
    state.rpm[1] = 0;
    state.rpm[2] = 0;
    state.rpm[3] = 0;

    state.de_raw[0] = 0;
    state.de_raw[1] = 0;
    state.de_raw[2] = 0;
    state.de_raw[3] = 0;

    state.de[0] = 0.0f;
    state.de[1] = 0.0f;
    state.de[2] = 0.0f;
    state.de[3] = 0.0f;

    state.healthy = true;

    return true;
}

void AP_Wingtip_Sim::update()
{
    // Increment all states
    state.rpm[0]++;
    state.rpm[1]++;
    state.rpm[2]++;
    state.rpm[3]++;

    state.de_raw[0]++;
    state.de_raw[1]++;
    state.de_raw[2]++;
    state.de_raw[3]++;

    state.de[0]++;
    state.de[1]++;
    state.de[2]++;
    state.de[3]++;
}

