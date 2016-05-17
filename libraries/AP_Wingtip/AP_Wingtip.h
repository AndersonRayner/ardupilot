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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Wingtip
{
public:

    AP_Wingtip(void);

    // parameters for each instance
    AP_Int8  _type;
    bool _healthy[6];  // RPM1 RPM2 RPM3 RPM4 de1 de2
    bool _enabled[6];  // RPM1 RPM2 RPM3 RPM4 de1 de2

    static const struct AP_Param::GroupInfo var_info[];

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    //  return RPM for a sensor.
    uint16_t get_rpm(uint8_t instance) const;

    // return de for a sensor.
    float get_de(uint8_t instance) const;

    // return if an instance is healthy
    bool healthy(uint8_t instance) const;

    // return if an instance is enabled
    bool enabled(uint8_t instance) const;

private:
    uint16_t RPM[4];
    float    de[2];

    union wingtip_data {
       uint8_t rxBuffer[7];
       uint16_t data[3];
    };

};
