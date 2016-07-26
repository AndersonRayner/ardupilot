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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#include <AP_HAL/GPIO.h>
#include <AP_HAL_Linux/GPIO_BBB.h>
#endif

class AP_Wingtip
{
public:

    AP_Wingtip(void);

    // parameters for each instance
    AP_Int8  _type;
    bool _healthy[2];  // RPM1 RPM2 RPM3 RPM4 de1 de2
    bool _enabled[2];  // RPM1 RPM2 RPM3 RPM4 de1 de2

    static const struct AP_Param::GroupInfo var_info[];

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    //  return RPM for a sensor.
    uint16_t get_rpm(uint8_t instance) const;

    // return de for a sensor.
    uint16_t get_de_raw(uint8_t instance) const;
    float    get_de(uint8_t instance) const;

    // return if an instance is healthy
    bool healthy(uint8_t instance) const;

    // return if an instance is enabled
    bool enabled(uint8_t instance) const;

private:
    uint16_t _RPM[4];
    uint16_t _de_raw[2];
    float    _de[2];

    union wingtip_data {
       uint8_t rxBuffer[9];
       uint16_t data[4];
    };

};
