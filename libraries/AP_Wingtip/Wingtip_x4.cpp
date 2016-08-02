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
#include "Wingtip_x4.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
//#include <AP_HAL/I2CDevice.h>
//#include <AP_Math/AP_Math.h>
#include <stdio.h>
//#include <utility>

extern const AP_HAL::HAL &hal;

/*
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
AP_HAL::DigitalSource *_cs;
#endif
*/

AP_Wingtip_x4::AP_Wingtip_x4(AP_Wingtip &_wingtip, uint8_t instance, AP_Wingtip::Wingtip_State &_state)
    : AP_Wingtip_Backend(_wingtip, instance, _state)
{
    init();
}


AP_Wingtip_x4::~AP_Wingtip_x4()
{
  // do nothing
}

bool AP_Wingtip_x4::init()
{

    hal.console->printf("Initialising wingtip board\n");
    /*
    // Reset the external boards
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    _cs = hal.gpio->channel(BBB_P9_15);
    if (_cs == NULL) {
        AP_HAL::panic("Unable to reset wingtip boards");
    }

    _cs->mode(HAL_GPIO_OUTPUT);
    _cs->write(WINGTIP_BOARD_RESET_LEVEL);       // high resets the board
    hal.scheduler->delay(5);
    _cs->write(!WINGTIP_BOARD_RESET_LEVEL);       // go low to let it do it's thing
    hal.scheduler->delay(5);
#endif

    _dev = hal.i2c_mgr->get_device(WINGTIP_I2C_BUS, WINGTIP_I2C_ADDR0);
*/
    return true;
}

void AP_Wingtip_x4::update()
{
    hal.console->printf("Updating data for wingtip_x4 board\n");
    // do nothing
 /*   union wingtip_data data1;
    uint8_t CRC;

    // take i2c bus sempahore
    if (!_dev || !_dev->get_semaphore()->take(15)) {
        return;
    }

    // Read data from board
    if (!_dev->transfer(nullptr, 0, data1.rxBuffer, sizeof(data1.rxBuffer))) {
        return;
    }

    // Return semaphore
    _dev->get_semaphore()->give();

    // Calculate checksum
    CRC = 0;
    for (uint8_t ii = 0; ii<8; ii++) {
        CRC = CRC ^ data1.rxBuffer[ii];
    }

    if (data1.rxBuffer[8] == CRC) {
        _RPM[0] = data1.data[0];
        _RPM[1] = data1.data[1];
        _RPM[2] = data1.data[2];
        _RPM[3] = data1.data[3];

        _healthy[0] = 1; // rpm1, rpm2, de1
        _healthy[1] = 1; // rpm3, rpm4, de2

    } else {
        // mark sensor unhealthy
        _healthy[0] = 0; // rpm1, rpm2, de1
        _healthy[1] = 0; // rpm3, rpm4, de2
    }*/
}
