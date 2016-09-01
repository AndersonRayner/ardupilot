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
  backend driver for wingtip x2 board from I2C
 */

#include "AP_Wingtip.h"
#include "Wingtip_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

class AP_Wingtip_x2 : public AP_Wingtip_Backend
{
public:
    // constructor
    AP_Wingtip_x2(AP_Wingtip &wingtip_board,  uint8_t instance, AP_Wingtip::Wingtip_State &_state);

    // destructor
    ~AP_Wingtip_x2(void);

    // initialise board
    bool init(uint8_t i2c_address);

    // update state
    void update();

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // Struct for receiving data via I2C
    union wingtip_data {
       uint8_t rxBuffer[5];
       uint16_t data[2];
    } data1;

};
