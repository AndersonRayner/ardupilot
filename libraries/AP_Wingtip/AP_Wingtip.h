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

#ifndef __WINGTIP_H__
#define __WINGTIP_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/GPIO.h>
#include <AP_HAL_Linux/GPIO_BBB.h>

class AP_Wingtip 
{
public:
    // constructor
    AP_Wingtip(void)
    {
		AP_Param::setup_object_defaults(this, var_info);
		
		// init _RPM and _de values
		memset(_RPM,0,sizeof(_RPM));
    memset(_de,0,sizeof(_de));
    };
	
    static const struct AP_Param::GroupInfo var_info[];
    
    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    /*
      return RPM for a sensor. Return -1 if not healthy
     */
    uint16_t get_rpm(uint8_t instance) const {
       /* if (!healthy(instance)) {
            return -1;
        } */
        return  _RPM[instance];
    }

    float get_de(uint8_t instance) const {
        /* if (!healthy(instance*2)) {
             return -1;
        } */
        return _de[instance];
    }

    /*
      return signal quality for a sensor.
     */
    float get_signal_quality(uint8_t instance) const {
      //  return state[instance].signal_quality;
        return 10.0f;
    }

    bool healthy(uint8_t instance) const;

    bool enabled(uint8_t instance) const;

private:
    union wingtip_data {
    uint8_t rxBuffer[7];
    uint16_t data[3];
    };

    AP_Int8  _type;
    uint16_t _RPM[4];
    float    _de[2];
    bool     _healthy[6];
  
};
#endif // __WINGTIP_H__