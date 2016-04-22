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
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/GPIO.h>
#include <AP_HAL_Linux/GPIO_BBB.h>

// Maximum number of RPM measurement instances available on this platform
#define RPM_MAX_INSTANCES 4

class AP_Wingtip 
{
public:
    AP_Wingtip(void);

    AP_Int8  _type[RPM_MAX_INSTANCES];

    //static const struct AP_Param::GroupInfo var_info[];
    
    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    /*
      return RPM for a sensor. Return -1 if not healthy
     */
    float get_rpm(uint8_t instance) const {
       /* if (!healthy(instance)) {
            return -1;
        }
        return state[instance].rate_rpm;*/
        return 2.0f;
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

//private:
  
};
