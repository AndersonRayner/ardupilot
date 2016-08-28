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
#pragma once

/*
  backend driver for simulating the wingtip boards
 */

#include "AP_Wingtip.h"
#include "Wingtip_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>

class AP_Wingtip_Sim : public AP_Wingtip_Backend
{
public:
    // constructor
    AP_Wingtip_Sim(AP_Wingtip &wingtip_board,  uint8_t instance, AP_Wingtip::Wingtip_State &_state);

    // destructor
    ~AP_Wingtip_Sim(void);
    
    // initialise board
    bool init();
    
    // update state
    void update();
    
protected:

private:

};
