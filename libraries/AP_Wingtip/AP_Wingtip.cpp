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

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM
    AP_GROUPINFO("_TYPE",    0, AP_Wingtip, _type[0], 0),

    AP_GROUPINFO("_TYPE1",    0, AP_Wingtip, _type[1], 0),

    AP_GROUPEND
};

AP_Wingtip::AP_Wingtip(void) 
{
    AP_Param::setup_object_defaults(this, var_info);
    hal.console->printf("AP_Wingtip::constructing...\n");
    // init state and drivers
  //  memset(state,0,sizeof(state));
  //  memset(drivers,0,sizeof(drivers));
}

/*
  initialise the AP_Wingtip class. 
 */
void AP_Wingtip::init(void)
{
    hal.console->printf("AP_Wingtip::init - testing...\n");
            /*
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        uint8_t type = _type[num_instances];
        uint8_t instance = num_instances;

        if (type == RPM_TYPE_PX4_PWM) {
            state[instance].instance = instance;
            drivers[instance] = new AP_RPM_PX4_PWM(*this, instance, state[instance]);
        }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        uint8_t instance = num_instances;
        state[instance].instance = instance;
        drivers[instance] = new AP_RPM_SITL(*this, instance, state[instance]);
#endif
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
    }*/
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_Wingtip::update(void)
{
    hal.console->printf("AP_Wingtip::update - testing...\n");
}
    
/*
  check if an instance is healthy
 */
bool AP_Wingtip::healthy(uint8_t instance) const
{
    hal.console->printf("AP_Wingtip::healthy - testing...\n");
    return true;
}

/*
  check if an instance is activated
 */
bool AP_Wingtip::enabled(uint8_t instance) const
{
    hal.console->printf("AP_Wingtip::enabled - testing...\n");
    return true;
}
