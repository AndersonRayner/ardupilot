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

    // @Param: _SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    AP_GROUPINFO("_SCALING", 1, AP_Wingtip, _scaling[0], 1.0f),

    // @Param: _MAX
    // @DisplayName: Maximum RPM
    // @Description: Maximum RPM to report
    // @Increment: 1
    AP_GROUPINFO("_MAX", 2, AP_Wingtip, _maximum[0], 0),

    // @Param: _MIN
    // @DisplayName: Minimum RPM
    // @Description: Minimum RPM to report
    // @Increment: 1
    AP_GROUPINFO("_MIN", 3, AP_Wingtip, _minimum[0], 0),

    // @Param: _MIN_QUAL
    // @DisplayName: Minimum Quality
    // @Description: Minimum data quality to be used
    // @Increment: 0.1
    AP_GROUPINFO("_MIN_QUAL", 4, AP_Wingtip, _quality_min[0], 0.5),

    AP_GROUPEND
};

AP_Wingtip::AP_Wingtip(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
/*
  initialise the AP_Wingtip class. 
 */
void AP_Wingtip::init(void)
{
   
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_Wingtip::update(void)
{

}
    
/*
  check if an instance is healthy
 */
bool AP_Wingtip::healthy(uint8_t instance) const
{
    return true;
}

/*
  check if an instance is activated
 */
bool AP_Wingtip::enabled(uint8_t instance) const
{
    return true;
}
