// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "AP_Wingtip.h"
#include "Wingtip_x2.h"
#include "Wingtip_x4.h"
#include "Wingtip_Sim.h"
#include "Wingtip_SITL.h"

#include <AP_HAL/GPIO.h>
#include <AP_HAL_Linux/GPIO_BBB.h>

extern const AP_HAL::HAL& hal;
AP_HAL::DigitalSource *_reset_pin;
AP_HAL::DigitalSource *_sync_pin;

// Define default board type
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define _TYPE_default WINGTIP_TYPE_X2
#else
#define _TYPE_default WINGTIP_TYPE_DISABLED
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
        // @Param: _TYPE
        // @DisplayName: Wingtip sensor type
        // @Description: What type of wingtip sensor is connected
        // @Values: 0:Disabled,1:Sim,2:I2C_Wingtip,3:I2C_Wingtip x4
        AP_GROUPINFO("_TYPE0",    0, AP_Wingtip, _type[0], _TYPE_default),
        AP_GROUPINFO("_TYPE1",    1, AP_Wingtip, _type[1], _TYPE_default),

        AP_GROUPEND
};

AP_Wingtip::AP_Wingtip(void) :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

//  initialise the AP_Wingtip class.
void AP_Wingtip::init(void)
{
    // Check if init has been called a second time
    for (uint8_t ii=0; ii<WINGTIP_MAX_BACKENDS; ii++) {
        if (drivers[ii] != NULL) {
            hal.console->printf("AP_Wingtip::init called a second time!\n");
            return;
        }
    }

// SITL
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        state[0].instance = 0;
        drivers[0] = new AP_Wingtip_SITL(*this, 0, state[0]);
        return;
#endif


// BBBMini with I2C connection
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        // Set up sync pin
        _sync_pin = hal.gpio->channel(BBB_P8_11);
        _sync_pin->mode(HAL_GPIO_OUTPUT);
        _sync_level = 0;
        _sync_pin->write(_sync_level);

        // Reset the external boards
        _reset_pin = hal.gpio->channel(BBB_P9_15);
        if (_reset_pin == NULL) {
            AP_HAL::panic("Unable to reset wingtip boards");
        }

        _reset_pin->mode(HAL_GPIO_OUTPUT);
        _reset_pin->write(WINGTIP_BOARD_RESET_LEVEL);       // high resets the board
        hal.scheduler->delay(5);
        _reset_pin->write(!WINGTIP_BOARD_RESET_LEVEL);       // go low to let it do it's thing
        hal.scheduler->delay(250);
#endif

        // Loop through each of the available backends and see what we can start
        for (uint8_t ii=0; ii<WINGTIP_MAX_BACKENDS; ii++) {
            switch (_type[ii]) {
            case WINGTIP_TYPE_DISABLED :
                break;

            case WINGTIP_TYPE_SIM :
                drivers[ii] = new AP_Wingtip_Sim(*this, ii, state[ii]);
                break;

            case WINGTIP_TYPE_X2 :
                drivers[ii] = new AP_Wingtip_x2(*this, ii, state[ii]);
                break;

            case WINGTIP_TYPE_X4 :
                drivers[ii] = new AP_Wingtip_x4(*this, ii, state[ii]);
                break;

            default :
                AP_HAL::panic("Wingtip board type not recognised!\n");
            }
        }
}

// Update all the wingtip data
void AP_Wingtip::update(void)
{
    for (uint8_t ii=0; ii<WINGTIP_MAX_BACKENDS; ii++) {
        if (drivers[ii] != NULL) {
            if (state[ii].enabled) {
                drivers[ii]->update();
            }
        }
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    // Toggle the sync line
    _sync_level = !_sync_level;
    _sync_pin->write(_sync_level);
#endif // CONFIG_HAL_BOARD_SUBTYPE

}


//  Check if an instance is healthy
bool AP_Wingtip::healthy(uint8_t instance) const
{
    if (instance >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[instance].healthy;
}

// Check if an instance is activated
bool AP_Wingtip::enabled(uint8_t instance) const
{
    if (instance >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[instance].enabled;
}

// return the rpm reading for a particular board and channel.  Return 0 if not healthy
uint16_t AP_Wingtip::get_rpm(uint8_t board, uint8_t channel) const
{
    if (board >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[board].rpm[channel];
}

// return the raw de reading for a particular board and channel.  Return 0 if not healthy
uint16_t AP_Wingtip::get_de_raw(uint8_t board, uint8_t channel) const
{
    if (board >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[board].de_raw[channel];
}

// return the de reading for a particular board and channel.  Return 0 if not healthy
float AP_Wingtip::get_de(uint8_t board, uint8_t channel) const
{
    if (board >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[board].de[channel];
}

uint64_t AP_Wingtip::get_last_reading_us(uint8_t board) const
{
    if (board >= WINGTIP_MAX_BACKENDS) {
        return false;
    }

    return state[board].last_reading_us;
}


