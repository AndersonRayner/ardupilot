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
#include "Wingtip_x4.h"
#include "Wingtip_SITL.h"

extern const AP_HAL::HAL& hal;

// Define default board type
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#define _TYPE_default 2
#else
#define _TYPE_default 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
        // @Param: _TYPE
        // @DisplayName: Wingtip sensor type
        // @Description: What type of wingtip sensor is connected
        // @Values: 0:Faked,1:I2C_Wingtip,2:I2C_Wingtip x4
        AP_GROUPINFO("_TYPE",    0, AP_Wingtip, _type, _TYPE_default),

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
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    uint8_t instance = num_instances;

// SITL
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        state[instance].instance = instance;
        drivers[instance] = new AP_Wingtip_SITL(*this, instance, state[instance]);
#endif


// BBBMini with I2C connection
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        switch (_type) {
        case WINGTIP_TYPE_NONE :
            break;

        case WINGTIP_TYPE_X2 :
           // drivers[instance] = new AP_Wingtip_x4(*this, instance, state[instance]);
           // drivers[instance] = new AP_Wingtip_x4(*this, instance, state[instance]);
            break;

        case WINGTIP_TYPE_X4 :
            drivers[instance] = new AP_Wingtip_x4(*this, instance, state[instance]);
            break;

        default :
            AP_HAL::panic("Wingtip board type not recognised!\n");
        }

#endif

    if (drivers[instance] != NULL) {
        // we loaded a driver for this instance, so it must be
        // present (although it may not be healthy)
        num_instances = instance+1;
    }

}

// Update all the wingtip data
void AP_Wingtip::update(void)
{
    for (uint8_t ii=0; ii<num_instances; ii++) {
        if (drivers[ii] != NULL) {
            drivers[ii]->update();
        }
    }
}


//  Check if an instance is healthy
bool AP_Wingtip::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }

    return state[instance].healthy;
}

// Check if an instance is activated
bool AP_Wingtip::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }

    return state[instance].enabled;
}

// return the rpm reading for a particular board and channel.  Return 0 if not healthy
uint16_t AP_Wingtip::get_rpm(uint8_t board, uint8_t channel) const
{
    if (board >= num_instances) {
        return false;
    }

    return state[board].rpm[channel];
}

// return the raw de reading for a particular board and channel.  Return 0 if not healthy
uint16_t AP_Wingtip::get_de_raw(uint8_t board, uint8_t channel) const
{
    if (board >= num_instances) {
        return false;
    }

    return state[board].de_raw[channel];
}

// return the de reading for a particular board and channel.  Return 0 if not healthy
float AP_Wingtip::get_de(uint8_t board, uint8_t channel) const
{
    if (board >= num_instances) {
        return false;
    }

    return state[board].de[channel];
}


