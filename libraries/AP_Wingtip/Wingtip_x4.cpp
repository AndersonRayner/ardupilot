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
#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

extern const AP_HAL::HAL &hal;

AP_Wingtip_x4::AP_Wingtip_x4(AP_Wingtip &_wingtip, uint8_t instance, AP_Wingtip::Wingtip_State &_state)
    : AP_Wingtip_Backend(_wingtip, instance, _state)
{
    state.instance = instance;

    state.enabled = init(WINGTIP_I2C_ADDR0+instance);

    if (state.enabled) {
        hal.console->printf("    Wingtip_x4 sensor %u [ x ]\n",state.instance);
    } else {
        hal.console->printf("    Wingtip_x4 sensor %u [   ]\n",state.instance);
    }
}


AP_Wingtip_x4::~AP_Wingtip_x4()
{
  // do nothing
}

bool AP_Wingtip_x4::init(uint8_t i2c_address)
{

    hal.console->printf("Initialising wingtip_x4 board %d\n",state.instance);

    // create i2c bus object
    _dev = std::move(hal.i2c_mgr->get_device(WINGTIP_I2C_BUS, i2c_address));

    // take i2c bus semaphore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // return semaphore so others can use it
    _dev->get_semaphore()->give();

    // give the board 20 tries (1 s) to become alive
    uint8_t counter = 0;
    while (counter<20) {
        update();
        if (state.rpm[0] > 0) {
            // board is ok, return
            return true;
        }
        counter ++;
        hal.scheduler->delay(50);
    };

    // no valid response from the board, disable it
    return false;
}

void AP_Wingtip_x4::update()
{
    uint8_t CRC;

    // take i2c bus sempahore
    if (!_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    // Read data from board
    if (!_dev->transfer(nullptr, 0, data1.rxBuffer, sizeof(data1.rxBuffer))) {
        // Return semaphore (transfer unsuccessful)
        _dev->get_semaphore()->give();
        state.i2c_lockups++;
        return;
    }

    // Return semaphore (transfer successful)
    _dev->get_semaphore()->give();

    // store time of last reading
    state.last_reading_us = AP_HAL::micros64();

    // Calculate checksum
    CRC = 0;
    for (uint8_t ii = 0; ii<8; ii++) {
        CRC = CRC ^ data1.rxBuffer[ii];
    }

    if (data1.rxBuffer[8] == CRC) {
        // data has passed checksum
        state.rpm[0] = data1.data[0];
        state.rpm[1] = data1.data[1];
        state.rpm[2] = data1.data[2];
        state.rpm[3] = data1.data[3];

        state.healthy = true;

    } else {
        // mark sensor unhealthy
        state.healthy = false;
    }

    // Other data fields - set to zero
    state.de_raw[0] = 0;
    state.de_raw[1] = 0;
    state.de_raw[2] = 0;
    state.de_raw[3] = 0;

    state.de[0] = 0.0f;
    state.de[1] = 0.0f;
    state.de[2] = 0.0f;
    state.de[3] = 0.0f;

}
