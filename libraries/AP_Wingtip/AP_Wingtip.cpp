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
//#include "Wingtip_SITL.h"

#include <AP_HAL/I2CDevice.h>

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

    // Eventually will be a for loop
    //uint8_t ii;
    uint8_t instance = num_instances;

    // Start with just doing an x4 board
    drivers[instance] = new AP_Wingtip_x4(*this, instance, state[instance]);


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
    return true;
    //return _healthy[instance];
}

// Check if an instance is activated
bool AP_Wingtip::enabled(uint8_t instance) const
{
    return true;
    //return _enabled[instance];
}

/*
// Return the RPM for an instance.  Return 0 if not healthy
uint16_t AP_Wingtip::get_rpm(uint8_t instance) const {
    return 1;
    //return _RPM[instance];
}

// return raw de for a sensor.  Return 0 if not healthy
uint16_t AP_Wingtip::get_de_raw(uint8_t instance) const {
    return 1;
    //return _de_raw[instance];
}


// return de for a sensor.  Return 0 if not healthy
float AP_Wingtip::get_de(uint8_t instance) const {
    return 1.0f;
    //return _de[instance];
}
*/
