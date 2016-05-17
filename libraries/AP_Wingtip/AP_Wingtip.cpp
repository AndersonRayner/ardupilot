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
    // @DisplayName: Wingtip sensor type
    // @Description: What type of wingtip sensor is connected
    // @Values: 0:Faked,1:Ardu-Board
    AP_GROUPINFO("_TYPE",    0, AP_Wingtip, _type, 0),


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
    RPM[0] =   0;
    RPM[1] = 100;
    RPM[2] = 200;
    RPM[3] = 300;

    de[0] =   0.0f;
    de[1] = 100.0f;
}

/*
  update wingtip for all instances. This should be called by main loop
 */
void AP_Wingtip::update(void)
{
    RPM[0]++;
    RPM[1]++;
    RPM[2]++;
    RPM[3]++;

    de[0]++;
    de[1]++;
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
