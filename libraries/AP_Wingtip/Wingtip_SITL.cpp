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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "Wingtip_SITL.h"

extern const AP_HAL::HAL &hal;

AP_Wingtip_SITL::AP_Wingtip_SITL(AP_Wingtip &_wingtip, uint8_t instance, AP_Wingtip::Wingtip_State &_state)
    : AP_Wingtip_Backend(_wingtip, instance, _state)
{
    state.enabled = init();
}


AP_Wingtip_SITL::~AP_Wingtip_SITL()
{
  // do nothing
}

bool AP_Wingtip_SITL::init()
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (sitl == nullptr) {
        return false;
    }

    // Everything seems to be ok
    return true;
}


void AP_Wingtip_SITL::update()
{

    if (!state.enabled) {
        return;
    }

    // Take RPMs from sitl and put into state struct
    state.last_reading_ms = AP_HAL::micros64();
    state.rpm[0] = sitl->state.rpm1;
    state.rpm[1] = sitl->state.rpm2;
    state.rpm[2] = 0;
    state.rpm[3] = 0;

    state.healthy = true;

}

#endif // CONFIG_HAL_BOARD
